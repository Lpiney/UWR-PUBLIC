// ============================================================================
//  params.h - runtime-adjustable parameters, persisted in NVS.
//
//  Why this layer exists: after the safety inspection the rules forbid touching
//  the hardware, but software updates and code changes are explicitly
//  exempted. So anything that might need adjusting on site - the tower URL,
//  ESC neutral, per-side trim, timeouts - lives in NVS and is changed with one
//  serial command. No reflash, no rule violation.
//
//  Side benefit: the Wi-Fi password and the URL never enter the source tree.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"

namespace params {

struct Data {
  String ssid        = "UR_Field";        // given in the competition manual, public
  String pass        = "12345678";        // same
  String url         = "";                // tower URL, published on the day:
                                          //   P url http://192.168.4.1/xxx
  int    neutral     = ESC_US_NEUTRAL;    // ESC neutral pulse width
  int    trimL       = 0;                 // left thruster trim (us), mechanical offset
  int    trimR       = 0;                 // right thruster trim
  int    trimV       = 0;                 // vertical thruster trim
  int    fsMs        = LINK_TIMEOUT_MS;   // link timeout before forcing neutral
  int    pollMs      = 2000;              // how often to fetch from HTTP
  int    wifiRetryMs = WIFI_RETRY_MS;     // association retry interval
};

inline Preferences& nvs() { static Preferences p; return p; }
inline Data&        d()   { static Data x; return x; }

// ---------------------------------------------------------------------------
//  Concurrency
//
//  The control loop (core 1) reassigns Strings inside Data when a P command
//  arrives, while the network task (core 0) may be reading the same String -
//  that is a use-after-free, ranging from garbage reads to a crash. Every
//  cross-core access to a String goes through the mutex.
//
//  Plain int fields need no lock: aligned word-sized reads and writes are
//  atomic on Xtensa, so the worst case is one tick of staleness.
// ---------------------------------------------------------------------------
inline SemaphoreHandle_t& mtx() { static SemaphoreHandle_t m = nullptr; return m; }

struct Guard {
  Guard()  { if (mtx()) xSemaphoreTake(mtx(), portMAX_DELAY); }
  ~Guard() { if (mtx()) xSemaphoreGive(mtx()); }
};

// A copy for the network task to work from, so it never touches the globals.
struct Snapshot {
  String ssid, pass, url;
  int    pollMs;
  int    wifiRetryMs;
};

inline Snapshot snap() {
  Guard g;
  Snapshot s;
  s.ssid        = d().ssid;
  s.pass        = d().pass;
  s.url         = d().url;
  s.pollMs      = d().pollMs;
  s.wifiRetryMs = d().wifiRetryMs;
  return s;
}

// Clamp every value that reaches the ESCs or a timer.
//
// On load as well as on set. Without it a neutral left over in NVS from an
// earlier session goes straight into the PWM register on the first frame after
// boot - and a unidirectional ESC reading 2000 us is full throttle. That is
// the one failure mode where cutting the power is already too late.
inline void clampAll() {
  Data &p = d();
  p.neutral     = constrain(p.neutral, ESC_US_MIN, ESC_US_MAX);
  p.trimL       = constrain(p.trimL, -200, 200);
  p.trimR       = constrain(p.trimR, -200, 200);
  p.trimV       = constrain(p.trimV, -200, 200);
  p.fsMs        = constrain(p.fsMs, 100, 5000);
  p.pollMs      = constrain(p.pollMs, 200, 60000);
  p.wifiRetryMs = constrain(p.wifiRetryMs, 3000, 120000);
}

inline void begin() {
  mtx() = xSemaphoreCreateMutex();
  nvs().begin("uwr", false);   // false = read/write
  Data &p = d();
  p.ssid        = nvs().getString("ssid",     p.ssid);
  p.pass        = nvs().getString("pass",     p.pass);
  p.url         = nvs().getString("url",      p.url);
  p.neutral     = nvs().getInt("neutral",     p.neutral);
  p.trimL       = nvs().getInt("trimL",       p.trimL);
  p.trimR       = nvs().getInt("trimR",       p.trimR);
  p.trimV       = nvs().getInt("trimV",       p.trimV);
  p.fsMs        = nvs().getInt("fsMs",        p.fsMs);
  p.pollMs      = nvs().getInt("pollMs",      p.pollMs);
  p.wifiRetryMs = nvs().getInt("wifiRetry",   p.wifiRetryMs);
  clampAll();
}

// Wipe NVS and go back to the values compiled into this file.
//
// Needed because a P command change is permanent: NVS sits in its own flash
// partition, so it survives a power cycle and a reflash. Testing on a home
// network leaves the board trying to join that network on competition day,
// which costs the whole of Mission 1 and is very hard to spot on site.
// Send R on power-up to get back to UR_Field / 12345678.
inline void reset() {
  Guard g;
  nvs().clear();
  d() = Data();
}

inline bool known(const String &key) {
  return key == "ssid" || key == "pass" || key == "url" || key == "neutral" ||
         key == "trimL" || key == "trimR" || key == "trimV" || key == "fsMs" ||
         key == "pollMs" || key == "wifiRetryMs";
}

inline bool set(const String &key, const String &val) {
  Guard g;
  Data &p = d();
  if (key == "ssid") {
    p.ssid = val;            nvs().putString("ssid", p.ssid);
  } else if (key == "pass") {
    p.pass = val;            nvs().putString("pass", p.pass);
  } else if (key == "url") {
    p.url = val;             nvs().putString("url", p.url);
  } else if (key == "neutral") {
    p.neutral = val.toInt(); nvs().putInt("neutral", p.neutral);
  } else if (key == "trimL") {
    p.trimL = val.toInt();   nvs().putInt("trimL", p.trimL);
  } else if (key == "trimR") {
    p.trimR = val.toInt();   nvs().putInt("trimR", p.trimR);
  } else if (key == "trimV") {
    p.trimV = val.toInt();   nvs().putInt("trimV", p.trimV);
  } else if (key == "fsMs") {
    p.fsMs = val.toInt();    nvs().putInt("fsMs", p.fsMs);
  } else if (key == "pollMs") {
    p.pollMs = val.toInt();  nvs().putInt("pollMs", p.pollMs);
  } else if (key == "wifiRetryMs") {
    p.wifiRetryMs = val.toInt(); nvs().putInt("wifiRetry", p.wifiRetryMs);
  } else {
    return false;
  }
  clampAll();
  return true;
}

inline String get(const String &key) {
  Data &p = d();
  if (key == "ssid")        return p.ssid;
  if (key == "pass")        return p.pass;
  if (key == "url")         return p.url;
  if (key == "neutral")     return String(p.neutral);
  if (key == "trimL")       return String(p.trimL);
  if (key == "trimR")       return String(p.trimR);
  if (key == "trimV")       return String(p.trimV);
  if (key == "fsMs")        return String(p.fsMs);
  if (key == "pollMs")      return String(p.pollMs);
  if (key == "wifiRetryMs") return String(p.wifiRetryMs);
  return "";
}

inline String dump() {
  Data &p = d();
  return "ssid=" + p.ssid + " pass=" + p.pass +
         " url=" + (p.url.length() ? p.url : String("(unset)")) +
         " neutral=" + String(p.neutral) +
         " trimL=" + String(p.trimL) + " trimR=" + String(p.trimR) +
         " trimV=" + String(p.trimV) +
         " fsMs=" + String(p.fsMs) + " pollMs=" + String(p.pollMs) +
         " wifiRetryMs=" + String(p.wifiRetryMs);
}

}  // namespace params
