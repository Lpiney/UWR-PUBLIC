// ============================================================================
//  thrusters.h - three ESC outputs.
//
//  Responsibilities:
//    1) Generate three 50 Hz / 1000-2000 us PWM channels with LEDC
//    2) Hold neutral through the boot window
//    3) Slew-limit every throttle change
//
//  Nothing here blocks. update() must be called at a high rate from the main
//  loop (the control tick runs at ~125 Hz).
// ============================================================================
#pragma once

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "params.h"

namespace thrusters {

struct ReverseGuard {
  int lastDir = 0;              // retained even while the target is neutral
  bool neutralActive = false;
  uint32_t neutralSince = 0;
  bool waiting = false;        // decelerating or holding before reversal
};

struct State {
  uint32_t readyAt    = 0;      // ignore input until this timestamp
  bool     ready      = false;  // boot window elapsed (also reported upstream)
  int      targetL    = ESC_US_NEUTRAL;   // mixer output, before slew
  int      targetR    = ESC_US_NEUTRAL;
  int      targetV    = ESC_US_NEUTRAL;   // vertical, straight from the triggers
  float    outL       = ESC_US_NEUTRAL;   // after slew; float keeps it smooth
  float    outR       = ESC_US_NEUTRAL;
  float    outV       = ESC_US_NEUTRAL;
  int      writtenL   = -1;     // last value actually pushed to LEDC
  int      writtenR   = -1;
  int      writtenV   = -1;
  int      neutral    = ESC_US_NEUTRAL;   // mirrored from params
  bool     safetyStop = true;
  ReverseGuard reverseL, reverseR, reverseV;
};

inline State& st() { static State s; return s; }

// Pulse width (us) -> LEDC duty. At 14 bits a 20000 us period is 16384 steps.
inline uint32_t usToDuty(int us) {
  return (uint32_t)us * (1u << ESC_PWM_RES_BITS) / ESC_PERIOD_US;
}

// Only touch LEDC when the value actually changed.
inline void writePWM(int l, int r, int v) {
  State &s = st();
  if (l != s.writtenL) { ledcWrite(ESC_CH_L, usToDuty(l)); s.writtenL = l; }
  if (r != s.writtenR) { ledcWrite(ESC_CH_R, usToDuty(r)); s.writtenR = r; }
  if (v != s.writtenV) { ledcWrite(ESC_CH_V, usToDuty(v)); s.writtenV = v; }
}

// Configure LEDC and put neutral on the wire *immediately*.
//
// The ESC watches the signal line during its own power-up: a valid neutral
// pulse lets it finish self-test and sit quietly, while a floating line makes
// it beep forever or refuse to arm at all. So this is the first thing setup()
// does.
inline void begin() {
  State &s = st();
  s.neutral = params::d().neutral;

  ledcSetup(ESC_CH_L, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  ledcSetup(ESC_CH_R, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  ledcSetup(ESC_CH_V, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  ledcAttachPin(PIN_ESC_L, ESC_CH_L);
  ledcAttachPin(PIN_ESC_R, ESC_CH_R);
  ledcAttachPin(PIN_ESC_V, ESC_CH_V);

  s.safetyStop = true;
  s.reverseL = s.reverseR = s.reverseV = ReverseGuard{};
  s.writtenL = s.writtenR = s.writtenV = -1;
  s.targetL = s.targetR = s.targetV = s.neutral;
  s.outL = s.outR = s.outV = (float)s.neutral;
  s.readyAt = millis() + BOOT_IGNORE_MS;
  s.ready = false;
  writePWM(s.neutral, s.neutral, s.neutral);
}

// Re-read neutral from params (call after a P or R command).
inline void syncParams() {
  State &s = st();
  if (s.neutral != params::d().neutral) {
    s.neutral = params::d().neutral;
    s.reverseL = s.reverseR = s.reverseV = ReverseGuard{};
  }
}

// Clamp a pulse width to the throttle cap, measured from neutral so the limit
// stays symmetric when the neutral itself has been moved. At maxPct=100 the
// span is the whole travel and this changes nothing.
inline int capped(int us) {
  const params::Data &p = params::d();
  const int span = (int)lroundf((ESC_US_MAX - ESC_US_NEUTRAL) * (p.maxPct / 100.0f));
  return constrain(us, p.neutral - span, p.neutral + span);
}

// Mixer output -> targets. Trims are applied here, then constrained, so a bad
// trim can never push the pulse width outside the ESC's travel. The throttle
// cap goes last: a trim large enough to reach past the cap is exactly the case
// it has to catch.
inline void setTarget(int l, int r, int v) {
  State &s = st();
  s.safetyStop = false;
  s.targetL = capped(constrain(l + params::d().trimL, ESC_US_MIN, ESC_US_MAX));
  s.targetR = capped(constrain(r + params::d().trimR, ESC_US_MIN, ESC_US_MAX));
  s.targetV = capped(constrain(v + params::d().trimV, ESC_US_MIN, ESC_US_MAX));
}

// Safety stop: no link or safety key locked. Stick centring uses the drive ramp.
inline void setNeutral() {
  State &s = st();
  s.safetyStop = true;
  s.targetL = s.targetR = s.targetV = s.neutral;
}

inline const char* stateName() {
  return st().ready ? "READY" : "WAIT";
}

inline int pulseDirection(float pulse, int neutral) {
  return (pulse > neutral) - (pulse < neutral);
}

inline void moveAxis(float &out, int target, ReverseGuard &g,
                     uint32_t now, float dtMs, bool safetyStop) {
  const int neutral = st().neutral;
  const int outDir = pulseDirection(out, neutral);
  if (outDir != 0) {
    g.lastDir = outDir;
    g.neutralActive = false;
  } else if (!g.neutralActive) {
    g.neutralActive = true;
    g.neutralSince = now;
  }

  const int targetDir = pulseDirection((float)target, neutral);
  const bool reversing = g.lastDir != 0 && targetDir != 0 &&
                         targetDir != g.lastDir;
  const bool held = g.neutralActive &&
      (uint32_t)(now - g.neutralSince) >= REVERSE_NEUTRAL_MS;
  g.waiting = reversing && !held;
  const int goal = g.waiting ? neutral : target;
  const float rate = safetyStop ? SLEW_US_PER_MS : DRIVE_SLEW_US_PER_MS;
  const float step = rate * dtMs;
  const float delta = (float)goal - out;
  if (fabsf(delta) <= step) out = (float)goal;
  else out += delta > 0.0f ? step : -step;

  if (out != (float)neutral) {
    g.lastDir = pulseDirection(out, neutral);
    g.neutralActive = false;
  } else if (!g.neutralActive) {
    // Start timing on the tick that actually writes neutral, not on the
    // earlier tick that first requested it. No delay() in the control loop.
    g.neutralActive = true;
    g.neutralSince = now;
  }
}

inline unsigned reverseMask() {
  const State &s = st();
  return (s.reverseL.waiting ? 1u : 0u) |
         (s.reverseR.waiting ? 2u : 0u) |
         (s.reverseV.waiting ? 4u : 0u);
}

// Hardware holds the previous pulse during a stall. Do not spend that stalled
// time as a single large slew step when execution resumes.
inline void update(float dtMs) {
  State &s = st();
  const uint32_t now = millis();
  if (!s.ready && (int32_t)(now - s.readyAt) >= 0) s.ready = true;
  if (!s.ready) setNeutral();

  if (!isfinite(dtMs) || dtMs <= 0.0f) dtMs = 1.0f;
  if (dtMs > CONTROL_MAX_DT_MS) dtMs = CONTROL_MAX_DT_MS;
  moveAxis(s.outL, s.targetL, s.reverseL, now, dtMs, s.safetyStop);
  moveAxis(s.outR, s.targetR, s.reverseR, now, dtMs, s.safetyStop);
  moveAxis(s.outV, s.targetV, s.reverseV, now, dtMs, s.safetyStop);
  writePWM((int)lroundf(s.outL), (int)lroundf(s.outR), (int)lroundf(s.outV));
}

}  // namespace thrusters
