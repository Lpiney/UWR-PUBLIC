// ============================================================================
//  mission1.h - Mission 1: join Wi-Fi and fetch data over HTTP.
//
//  The task rules require the Wi-Fi association and the fetch to be performed
//  by the MCU on the ROV itself. Data obtained through a laptop, a phone or a
//  browser scores nothing. The retrieved text has to be displayable to the
//  referee. The SSID and password are compiled into params.h; the tower URL is
//  released on the day and set with P url. The tower URL is only
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
inline volatile bool& reqScan()    { static volatile bool v = false; return v; }
inline volatile int&  wifiState()  { static volatile int  v = 0;     return v; }
inline volatile int&  rssi()       { static volatile int  v = 0;     return v; }
// Last WiFi.status() seen by the network task, so the reason a connection is
// not coming up can be reported without the other core touching the WiFi
// stack. -1 until the task has looked.
inline volatile int&  wifiReason() { static volatile int  v = -1;    return v; }

// 0 = idle / 1 = connecting / 2 = connected / 3 = lost
//
// While connecting, say why rather than just that it is happening. "CONNECTING"
// forever is the one report that cannot be acted on: a wrong password and an
// access point that is not in range look identical, and they need opposite
// responses - type the password again, or go and find the organisers.
inline const char* wifiStateName() {
  switch (wifiState()) {
    case 1:
      switch (wifiReason()) {
        case WL_NO_SSID_AVAIL:   return "NO_SSID";    // not in range, or 5 GHz
        case WL_CONNECT_FAILED:  return "BAD_PASS";
        case WL_CONNECTION_LOST: return "LOST";
        default:                 return "CONNECTING";
      }
    case 2: return "CONNECTED";
    case 3: return "LOST";
    default: return "IDLE";
  }
}

inline String localIp() {
  return (wifiState() == 2) ? WiFi.localIP().toString() : String("-");
}

// Gateway address, reported in the status line. It says which network the
// board actually landed on, which is the first thing to check when the tower
// cannot be reached.
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
      if (p.ssid.length() == 0) {
        // Nothing compiled in and nothing set. Say so once rather than sit in
        // CONNECTING for ever: P ssid and P pass reconnect by themselves.
        wifiState() = 0;
        post("EV", "wifi: no SSID set - send  P ssid <name>  then  P pass <password>");
      } else {
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);        // no power save: lower round-trip latency
        WiFi.begin(p.ssid.c_str(), p.pass.c_str());
        wifiState() = 1;
        nextRetry = millis() + p.wifiRetryMs;
        post("EV", "wifi: connecting to " + p.ssid);
      }
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
      wifiReason() = WiFi.status();
      if (wifiState() == 2) {
        wifiState() = 3;
        post("EV", "wifi: disconnected");
      }
      if ((int32_t)(millis() - nextRetry) >= 0) {
        nextRetry = millis() + p.wifiRetryMs;
        if (p.ssid.length()) {       // an unset SSID is not worth retrying
          WiFi.disconnect();
          WiFi.begin(p.ssid.c_str(), p.pass.c_str());
        }
      }
    }

    // ---- 2b. scan on request ----
    //
    // This is how a failed association gets diagnosed. "CONNECTING" cannot
    // distinguish a wrong password from an access point that is not there,
    // and a list of what the board can actually see settles it in one look:
    // the name is in the list and it will not join, or the name is not in the
    // list at all and no amount of retyping the password will help.
    //
    // A scan takes the radio off channel for a second or two, so it is done
    // only when asked for. It may blip a working connection; the normal retry
    // brings it back.
    if (reqScan()) {
      reqScan() = false;
      const int n = WiFi.scanNetworks();
      post("NET", "BEGIN");
      for (int i = 0; i < n && i < M1_SCAN_MAX; ++i) {
        // SSID last: it may contain spaces, and the reader takes the rest of
        // the line for it.
        post("NET", String(WiFi.RSSI(i)) + " " + String(WiFi.channel(i)) + " " +
                    (WiFi.SSID(i).length() ? WiFi.SSID(i) : String("<hidden>")));
      }
      post("NET", String(n) + " END");
      WiFi.scanDelete();
    }

    // ---- 3. periodic fetch ----
    if (wifiState() == 2 && (reqFetch() || (int32_t)(millis() - nextPoll) >= 0)) {
      reqFetch() = false;
      nextPoll = millis() + p.pollMs;
      // The URL is published on the day and has no default worth guessing. Say
      // so instead: a fetch that quietly goes somewhere else is worse than one
      // that plainly did not happen.
      if (p.url.length() == 0) {
        post("EV", "url not set - send  P url http://...");
      } else {
        fetchOnce(p.url);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

inline void begin() {
  q() = xQueueCreate(M1_QUEUE_LEN, sizeof(Ev));
  xTaskCreatePinnedToCore(task, "mission1", M1_TASK_STACK, nullptr, 1, nullptr,
                          M1_TASK_CORE);
}

}  // namespace mission1
