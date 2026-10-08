// ============================================================================
//  comms.h - line protocol between the surface laptop and the ROV.
//
//  Physical link: the laptop may use either the ESP32-S3 native USB port or
//  the on-board USB-UART ("COM" port). Both accept the same commands and
//  output is written to both.
//
//  Why a plain ASCII line protocol instead of a packed binary one? A serial
//  monitor or pyserial is enough to send a command by hand and read the reply,
//  so debugging costs nothing. The bandwidth a binary protocol would save is
//  bandwidth this project does not need.
//
//  ---- laptop -> ROV ----
//      CMD nx=<f> ny=<f> [vz=<f>] en=<0|1>
//                        Drive. nx/ny/vz are -1..+1: +x right, +y forward,
//                        +z up. en=1 means the safety key is unlocked; en=0
//                        forces neutral. Sent every 20 ms; silence for
//                        LINK_TIMEOUT_MS also forces neutral.
//      P <k> <v>         Write a parameter (persisted in NVS)
//      G [<k>]           Read one parameter, or all of them with no argument
//      R                 Restore default parameters
//      W                 Connect Wi-Fi now
//      F                 Fetch from HTTP now
//      S                 Report status
//      B                 Reboot
//
//  ---- ROV -> laptop ----
//      OK <text>         Command accepted
//      ERR <text>        Command rejected
//      EV <text>         Event (Wi-Fi up, HTTP failed, ...)
//      DATA <text>       Mission 1 payload, shown to the referee
//      JOY ...           Drive telemetry, every 50 ms
//      ST ...            Mission 1 status, every 500 ms
// ============================================================================
#pragma once

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "params.h"
#include "thrusters.h"
#include "mission1.h"

// The namespace cannot be called "link": POSIX <unistd.h> already declares
// int link(...) and the collision is a hard compile error.
namespace comms {

// ---------------------------------------------------------------------------
//  Latest drive command from the laptop
// ---------------------------------------------------------------------------
struct Command {
  float    nx         = 0.0f;   // -1..+1, +x is right
  float    ny         = 0.0f;   // -1..+1, +y is forward
  float    vz         = 0.0f;   // -1..+1, +z is up (RT up, LT down)
  bool     enable     = false;  // safety key state, sent by the host
  uint32_t lastCmdMs  = 0;      // arrival time of the last *valid* line
  uint32_t lines      = 0;      // accepted lines (diagnostics)
  uint32_t badLines   = 0;      // rejected lines (diagnostics)
};

inline Command& cmd() { static Command c; return c; }

// Defined below; handle() reports it for the S command.
inline String statusLine();

// True while a drive command is arriving fast enough to be trusted.
inline bool linkOk(uint32_t now) {
  const Command &c = cmd();
  return c.lastCmdMs != 0 && (int32_t)(now - c.lastCmdMs) < params::d().fsMs;
}

// ---------------------------------------------------------------------------
//  Output helpers
// ---------------------------------------------------------------------------

// Both wires at once: native USB (laptop) and UART0 (COM port, for a serial
// monitor). Seeing the same log on the debug port is worth a lot on site.
inline void emit(const char *tag, const String &msg) {
  Serial.print(tag);   Serial.print(' ');   Serial.println(msg);
  Serial0.print(tag);  Serial0.print(' ');  Serial0.println(msg);
}

inline void ok(const String &m)    { emit("OK", m); }
inline void err(const String &m)   { emit("ERR", m); }
inline void event(const String &m) { emit("EV", m); }
inline void data(const String &m)  { emit("DATA", m); }

// ---------------------------------------------------------------------------
//  CMD parsing
//
//  Parse token by token, looking for key=value. Never split on a substring
//  like " X=": the line contains both nx= and X is derivable, and a fixed
//  split eventually cuts in the wrong place. Unknown keys are skipped, so
//  adding fields later cannot break an older parser.
//
//  A line missing nx, ny or en is rejected outright. Better to hold position
//  than to drive the motors from half a command.
//
//  vz is optional and defaults to 0, so a host that predates the vertical
//  thruster keeps working - it simply leaves it at neutral.
// ---------------------------------------------------------------------------
inline bool parseCmd(const char *s) {
  if (strncmp(s, "CMD", 3) != 0) return false;

  Command &c = cmd();
  float nx = c.nx, ny = c.ny, vz = c.vz;
  bool  en = c.enable;
  bool  gotNx = false, gotNy = false, gotEn = false;

  const char *p = s + 3;
  while (*p) {
    while (*p == ' ') p++;
    if (!*p) break;
    const char *eq = strchr(p, '=');
    if (eq == nullptr) return false;          // half a token: drop the line
    const size_t klen = (size_t)(eq - p);
    const char  *val  = eq + 1;

    if      (klen == 2 && !strncmp(p, "nx", 2)) { nx = strtof(val, nullptr); gotNx = true; }
    else if (klen == 2 && !strncmp(p, "ny", 2)) { ny = strtof(val, nullptr); gotNy = true; }
    else if (klen == 2 && !strncmp(p, "vz", 2)) { vz = strtof(val, nullptr); }
    else if (klen == 2 && !strncmp(p, "en", 2)) { en = (atoi(val) != 0);      gotEn = true; }

    p = val;
    while (*p && *p != ' ') p++;
  }

  if (!(gotNx && gotNy && gotEn)) return false;

  c.nx        = constrain(nx, -1.0f, 1.0f);
  c.ny        = constrain(ny, -1.0f, 1.0f);
  c.vz        = constrain(vz, -1.0f, 1.0f);
  c.enable    = en;
  c.lastCmdMs = millis();
  c.lines++;
  return true;
}

// ---------------------------------------------------------------------------
//  Command dispatch
// ---------------------------------------------------------------------------
inline void handle(const String &raw) {
  String s = raw;
  s.trim();
  if (!s.length()) return;

  if (s.startsWith("CMD")) {
    if (!parseCmd(s.c_str())) {
      cmd().badLines++;
      err("malformed CMD line");
    }
    return;   // silent on success: this is a high-rate control line
  }

  const int sp = s.indexOf(' ');
  String name = (sp < 0) ? s : s.substring(0, sp);
  String arg  = (sp < 0) ? String("") : s.substring(sp + 1);
  arg.trim();
  name.toUpperCase();

  // ---------- parameters ----------
  if (name == "P") {
    const int sp2 = arg.indexOf(' ');
    if (sp2 < 0) { err("usage: P <key> <value>"); return; }
    String k = arg.substring(0, sp2);  k.trim();
    String v = arg.substring(sp2 + 1); v.trim();
    if (!params::set(k, v)) { err("unknown parameter: " + k); return; }
    thrusters::syncParams();   // neutral may have changed; take effect at once
    ok("P " + k + " = " + params::get(k));
    return;
  }
  if (name == "G") {
    if (!arg.length())       { ok(params::dump()); return; }
    if (!params::known(arg)) { err("unknown parameter: " + arg); return; }
    ok(arg + " = " + params::get(arg));
    return;
  }
  if (name == "R") {
    params::reset();
    thrusters::syncParams();
    ok("params reset: " + params::dump());
    return;
  }

  // ---------- Mission 1 ----------
  if (name == "W") { mission1::reqConnect() = true; ok("connecting wifi"); return; }
  if (name == "F") { mission1::reqFetch()   = true; ok("fetching now");    return; }

  // ---------- status / reboot ----------
  if (name == "S") { ok(statusLine()); return; }
  if (name == "B") { ok("rebooting"); delay(100); ESP.restart(); return; }

  err("unknown command: " + name);
}

// ---------------------------------------------------------------------------
//  Receive
// ---------------------------------------------------------------------------

// Separate buffer per input so half a line on one port cannot be spliced onto
// half a line from the other.
inline void pollInput(Stream &in, char *buf, size_t &len) {
  // Bounded per call: a continuous stream must not stall the motor update.
  for (size_t i = 0; i < LINK_LINE_MAX && in.available() > 0; ++i) {
    const int c = in.read();
    if (c < 0) break;

    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; handle(String(buf)); len = 0; }
    } else if (len < LINK_LINE_MAX - 1) {
      buf[len++] = (char)c;
    } else {
      len = 0;   // line too long: discard it whole
    }
  }
}

inline void poll() {
  static char   usbBuf[LINK_LINE_MAX];
  static char   uartBuf[LINK_LINE_MAX];
  static size_t usbLen = 0;
  static size_t uartLen = 0;
  pollInput(Serial,  usbBuf,  usbLen);
  pollInput(Serial0, uartBuf, uartLen);
}

// ---------------------------------------------------------------------------
//  Telemetry
// ---------------------------------------------------------------------------

inline String statusLine() {
  const thrusters::State &t = thrusters::st();
  return String("state=") + thrusters::stateName() +
         " outL="  + String(t.outL) +
         " outR="  + String(t.outR) +
         " outV="  + String(t.outV) +
         " enc="   + String(cmd().enable ? 1 : 0) +
         " link="  + String(linkOk(millis()) ? 1 : 0) +
         " wifi="  + mission1::wifiStateName() +
         " ip="    + mission1::localIp() +
         " rssi="  + String(mission1::rssi()) +
         " gw="    + mission1::gatewayIp() +
         " http="  + String(mission1::lastHttp()) +
         " url="   + (params::d().url.length() ? params::d().url : String("(unset)"));
}

// Drive telemetry. Every field the on-screen display needs, and nothing else.
// New fields go on the end so older parsers keep working.
inline void tickTelemetry() {
  static uint32_t nextJoy = 0;
  static uint32_t nextSt  = 0;
  const uint32_t now = millis();

  if ((int32_t)(now - nextJoy) >= 0) {
    nextJoy = now + TELEMETRY_MS;
    const Command &c = cmd();
    const thrusters::State &t = thrusters::st();
    const float mag = sqrtf(c.nx * c.nx + c.ny * c.ny);

    // Built into a buffer and sent through emit() rather than Serial.printf:
    // emit() writes to both serial ports, and a board whose only connection is
    // the UART bridge would otherwise never see a single telemetry line.
    char buf[224];
    snprintf(buf, sizeof(buf),
             "X=%+.3f Y=%+.3f Z=%+.3f MAG=%.3f %s READY=%d "
             "L=%d R=%d V=%d TL=%d TR=%d EN=%d LINK=%d",
             c.nx, c.ny, c.vz, mag, (mag < PULSE_DEADZONE) ? "DEAD" : "OUT",
             t.ready ? 1 : 0,
             (int)lroundf(t.outL), (int)lroundf(t.outR), (int)lroundf(t.outV),
             t.targetL, t.targetR,
             c.enable ? 1 : 0, linkOk(now) ? 1 : 0);
    emit("JOY", String(buf));
  }

  if ((int32_t)(now - nextSt) >= 0) {
    nextSt = now + STATUS_MS;
    emit("ST", statusLine());
  }
}

inline void begin() {
  // HWCDC blocks up to 100 ms by default when the host is not reading, which
  // is far too long for a control loop. Cap it so the link timeout (200 ms)
  // can never be starved by a serial write.
  Serial.setTxTimeoutMs(10);
}

}  // namespace comms
