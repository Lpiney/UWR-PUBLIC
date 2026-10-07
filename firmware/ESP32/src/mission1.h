// ============================================================================
//  mission1.h - Mission 1: join Wi-Fi and fetch data over HTTP.
//
//  The task rules require the Wi-Fi association and the fetch to be performed
//  by the MCU on the ROV itself. Data obtained through a laptop, a phone or a
//  browser scores nothing. The retrieved text has to be displayable to the
//  referee. SSID is UR_Field, password 12345678, and the tower URL is only
//  published on the day.
//
//  Everything network-related runs in its own FreeRTOS task on core 0.
//  HTTP requests block for up to HTTP_TIMEOUT_MS, and doing that in the main
//  loop would trip the link timeout and stop the thrusters every time the
//  network hiccups.
//
//  The task and the control loop share no variables; they communicate only
//  through a FreeRTOS queue, so no locking is required.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "config.h"
#include "params.h"

namespace mission1 {

// One event to be reported upstream.
struct Ev {
  char tag[8];
  char msg[200];
};

inline QueueHandle_t& q() { static QueueHandle_t h = nullptr; return h; }

// Written by the control loop, read by the network task. All single-word
// volatile accesses, so the worst case is one tick of staleness.
inline volatile bool& reqConnect() { static volatile bool v = true;  return v; }
inline volatile bool& reqFetch()   { static volatile bool v = false; return v; }
inline volatile int&  wifiState()  { static volatile int  v = 0;     return v; }
inline volatile int&  rssi()       { static volatile int  v = 0;     return v; }
inline bool&          hasData()    { static bool v = false; return v; }

// 0 = idle / 1 = connecting / 2 = connected / 3 = lost
inline const char* wifiStateName() {
  switch (wifiState()) {
    case 1: return "CONNECTING";
    case 2: return "CONNECTED";
    case 3: return "LOST";
    default: return "IDLE";
  }
}

inline String localIp() {
  return (wifiState() == 2) ? WiFi.localIP().toString() : String("-");
}

// Gateway address. With a phone hotspot the gateway is the phone itself,
// which makes it a handy target for bench testing.
inline String gatewayIp() {
  return (wifiState() == 2) ? WiFi.gatewayIP().toString() : String("-");
}

// Result of the last HTTP request: 0 = never fetched, positive = status code,
// negative = error code.
inline volatile int& lastHttp() { static volatile int v = 0; return v; }

// Push an event onto the queue. If the queue is full it is dropped rather than
// blocking the network task.
inline void post(const char *tag, const String &msg) {
  if (!q()) return;

  // Suppress a repeat of the identical previous event. Without this, a server
  // that is down produces "connection failed" every two seconds and drowns
  // everything else. DATA is exempt: it is the line shown to the referee, so
  // every successful fetch is reported.
  static char lastTag[8]   = {0};
  static char lastMsg[200] = {0};
  if (strcmp(tag, "DATA") != 0 &&
      strcmp(tag, lastTag) == 0 && strcmp(msg.c_str(), lastMsg) == 0) {
    return;
  }
  strncpy(lastTag, tag, sizeof(lastTag) - 1);
  strncpy(lastMsg, msg.c_str(), sizeof(lastMsg) - 1);

  Ev e;
  strncpy(e.tag, tag, sizeof(e.tag) - 1);
  e.tag[sizeof(e.tag) - 1] = 0;
  strncpy(e.msg, msg.c_str(), sizeof(e.msg) - 1);
  e.msg[sizeof(e.msg) - 1] = 0;
  xQueueSend(q(), &e, 0);
}

inline void fetchOnce(const String &url) {
  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.setConnectTimeout(HTTP_TIMEOUT_MS);
  // A server root very often redirects; follow it.
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  if (!http.begin(url)) {
    lastHttp() = -999;
    post("EV", "http: bad URL: " + url);
    return;
  }

  const int code = http.GET();
  lastHttp() = code;

  if (code == HTTP_CODE_OK) {
    String body = http.getString();
    body.trim();
    // If the server returned a whole page, keep only the head of it so the
    // sentence meant for the referee is not pushed off the end.
    if (body.length() > 180) body = body.substring(0, 180) + " ...";
    hasData() = true;
    post("DATA", body);
  } else if (code > 0) {
    post("EV", "http: status " + String(code));
  } else {
    post("EV", "http: " + http.errorToString(code));
  }
  http.end();
}

inline void task(void *) {
  uint32_t nextRetry = 0;
  uint32_t nextPoll  = 0;

  for (;;) {
    // A P command can rewrite these at any moment, so take one locked copy at
    // the top of the loop and never touch the globals again.
    params::Snapshot p = params::snap();

    // ---- 1. connect on request ----
    if (reqConnect()) {
      reqConnect() = false;
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);          // no power save: lower round-trip latency
      WiFi.begin(p.ssid.c_str(), p.pass.c_str());
      wifiState() = 1;
      nextRetry = millis() + p.wifiRetryMs;
      post("EV", "wifi: connecting to " + p.ssid);
    }

    // ---- 2. maintain the association ----
    if (WiFi.status() == WL_CONNECTED) {
      if (wifiState() != 2) {
        wifiState() = 2;
        rssi() = WiFi.RSSI();
        post("EV", "wifi: connected ip=" + WiFi.localIP().toString() +
                   " rssi=" + String(WiFi.RSSI()));
        nextPoll = 0;                // fetch immediately once connected
      }
    } else {
      if (wifiState() == 2) {
        wifiState() = 3;
        post("EV", "wifi: disconnected");
      }
      if ((int32_t)(millis() - nextRetry) >= 0) {
        WiFi.disconnect();
        WiFi.begin(p.ssid.c_str(), p.pass.c_str());
        nextRetry = millis() + p.wifiRetryMs;
      }
    }

    // ---- 3. periodic fetch ----
    if (wifiState() == 2 && (reqFetch() || (int32_t)(millis() - nextPoll) >= 0)) {
      reqFetch() = false;
      nextPoll = millis() + p.pollMs;
      // With no URL configured, fall back to the gateway root. That makes a
      // phone hotspot with a server on the phone work with no configuration,
      // while competition day uses the URL the organisers publish.
      const String target = p.url.length()
                          ? p.url
                          : ("http://" + WiFi.gatewayIP().toString() + "/");
      fetchOnce(target);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

inline void begin() {
  q() = xQueueCreate(12, sizeof(Ev));
  xTaskCreatePinnedToCore(task, "mission1", M1_TASK_STACK, nullptr, 1, nullptr,
                          M1_TASK_CORE);
}

}  // namespace mission1
