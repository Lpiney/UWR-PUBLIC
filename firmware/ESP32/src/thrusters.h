// ============================================================================
//  thrusters.h - dual ESC output.
//
//  Responsibilities:
//    1) Generate two 50 Hz / 1000-2000 us PWM channels with LEDC
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

struct State {
  uint32_t readyAt    = 0;      // ignore input until this timestamp
  bool     ready      = false;  // boot window elapsed (also reported upstream)
  int      targetL    = ESC_US_NEUTRAL;   // mixer output, before slew
  int      targetR    = ESC_US_NEUTRAL;
  float    outL       = ESC_US_NEUTRAL;   // after slew; float keeps it smooth
  float    outR       = ESC_US_NEUTRAL;
  int      writtenL   = -1;     // last value actually pushed to LEDC
  int      writtenR   = -1;
  int      neutral    = ESC_US_NEUTRAL;   // mirrored from params
};

inline State& st() { static State s; return s; }

// Pulse width (us) -> LEDC duty. At 14 bits a 20000 us period is 16384 steps.
inline uint32_t usToDuty(int us) {
  return (uint32_t)us * (1u << ESC_PWM_RES_BITS) / ESC_PERIOD_US;
}

// Only touch LEDC when the value actually changed.
inline void writePWM(int l, int r) {
  State &s = st();
  if (l != s.writtenL) { ledcWrite(ESC_CH_L, usToDuty(l)); s.writtenL = l; }
  if (r != s.writtenR) { ledcWrite(ESC_CH_R, usToDuty(r)); s.writtenR = r; }
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
  ledcAttachPin(PIN_ESC_L, ESC_CH_L);
  ledcAttachPin(PIN_ESC_R, ESC_CH_R);

  s.targetL = s.targetR = s.neutral;
  s.outL = s.outR = (float)s.neutral;
  s.readyAt = millis() + BOOT_IGNORE_MS;
  s.ready = false;
  writePWM(s.neutral, s.neutral);
}

// Re-read neutral from params (call after a P or R command).
inline void syncParams() {
  State &s = st();
  s.neutral = params::d().neutral;
}

// Mixer output -> targets. Trims are applied here, then constrained, so a bad
// trim can never push the pulse width outside the ESC's travel.
inline void setTarget(int l, int r) {
  State &s = st();
  s.targetL = constrain(l + params::d().trimL, ESC_US_MIN, ESC_US_MAX);
  s.targetR = constrain(r + params::d().trimR, ESC_US_MIN, ESC_US_MAX);
}

// Force neutral: no link, safety key locked, or the stick is centred.
inline void setNeutral() {
  State &s = st();
  s.targetL = s.targetR = s.neutral;
}

inline const char* stateName() {
  return st().ready ? "READY" : "WAIT";
}

// High-rate tick. dtMs is the time since the previous call.
inline void update(float dtMs) {
  State &s = st();
  const uint32_t now = millis();

  if (!s.ready && (int32_t)(now - s.readyAt) >= 0) {
    s.ready = true;
  }

  // Through the boot window the input is ignored entirely.
  if (!s.ready) {
    s.targetL = s.targetR = s.neutral;
  }

  // Slew limit: move at most SLEW_US_PER_MS per millisecond toward the target.
  const float maxStep = SLEW_US_PER_MS * (dtMs > 0.0f ? dtMs : 1.0f);
  float dL = (float)s.targetL - s.outL;
  float dR = (float)s.targetR - s.outR;
  if (dL >  maxStep) dL =  maxStep;
  if (dL < -maxStep) dL = -maxStep;
  if (dR >  maxStep) dR =  maxStep;
  if (dR < -maxStep) dR = -maxStep;
  s.outL += dL;
  s.outR += dR;

  writePWM((int)lroundf(s.outL), (int)lroundf(s.outR));
}

}  // namespace thrusters
