#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include "thrusters.h"
#include "diagnostics.h"

uint32_t fakeNow = 0;
uint32_t duties[3] = {};
unsigned writes[3] = {};
using namespace thrusters;

static void reset(uint32_t now = 0) {
  fakeNow = now;
  params::d() = params::Data{};
  st() = State{};
  for (int i = 0; i < 3; ++i) duties[i] = writes[i] = 0;
  begin();
}
static void tick(int l, int r = 1500, int v = 1500, uint32_t dt = CONTROL_MS) {
  fakeNow += dt;
  setTarget(l, r, v);
  update((float)dt);
}
static void ready() {
  fakeNow += BOOT_IGNORE_MS;
  setNeutral();
  update(CONTROL_MS);
  assert(st().ready);
}
static void drive(int l = 2000, int r = 1500, int v = 1500) {
  for (int i = 0; i < 126; ++i) tick(l, r, v);
  assert(st().outL == l && st().outR == r && st().outV == v);
}
static void reachNeutral(int target) {
  unsigned count = 0;
  while (st().outL != st().neutral) {
    assert(++count < 200);
    tick(target);
  }
}
static void bootTest() {
  reset();
  assert(duties[0] == usToDuty(1500));
  for (int i = 0; i < 374; ++i) {
    tick(2000, 1000, 2000);
    assert(st().outL == 1500 && st().outR == 1500 && st().outV == 1500);
    assert(!st().ready);
  }
  tick(2000, 1000, 2000);
  assert(st().ready && st().outL == 1504 && st().outR == 1496);
}
static void driveTest() {
  reset(); ready();
  for (int i = 1; i <= 125; ++i) {
    tick(2000, 1000, 1750);
    assert(st().outL == 1500 + 4 * i);
    assert(st().outR == 1500 - 4 * i);
    assert(st().outV == std::min(1500 + 4 * i, 1750));
  }
}
static void reverseTest() {
  for (int direction : {-1, 1}) {
    reset(); ready();
    const int from = 1500 + 500 * direction;
    const int to = 1500 - 500 * direction;
    drive(from, to, from);
    const uint32_t start = fakeNow;
    for (int i = 0; i < 125; ++i) tick(to, from, to);
    assert(st().outL == 1500 && st().outR == 1500 && st().outV == 1500);
    assert(reverseMask() == 7);
    const uint32_t neutralAt = fakeNow;
    while ((uint32_t)(fakeNow + CONTROL_MS - neutralAt) < REVERSE_NEUTRAL_MS) {
      tick(to, from, to);
      assert(st().outL == 1500 && st().outR == 1500 && st().outV == 1500);
    }
    tick(to, from, to);
    assert(st().outL == 1500 - 4 * direction);
    assert((uint32_t)(fakeNow - neutralAt) >= REVERSE_NEUTRAL_MS);
    assert(reverseMask() == 0);
    for (int i = 0; i < 124; ++i) tick(to, from, to);
    assert(st().outL == to && st().outR == from && st().outV == to);
    // Slew starts on the tick that releases the hold, so allow one tick
    // when comparing with the continuous-time 2.52 s operator estimate.
    assert((uint32_t)(fakeNow - start) >= 2520 - CONTROL_MS);
    assert((uint32_t)(fakeNow - start) <= 2520 + CONTROL_MS);
  }
}
static void centreTest(bool longWait) {
  reset(); ready(); drive();
  reachNeutral(1500);
  const uint32_t at = fakeNow;
  if (longWait) {
    for (int i = 0; i < 66; ++i) tick(1500);
    tick(1000);
    assert(st().outL == 1496 && reverseMask() == 0);
  } else {
    tick(1000);
    assert(st().outL == 1500 && reverseMask() == 1);
    while ((uint32_t)(fakeNow + CONTROL_MS - at) < REVERSE_NEUTRAL_MS) {
      tick(1000);
      assert(st().outL == 1500);
    }
    tick(1000);
    assert(st().outL == 1496);
  }
}
static void cancelTest(bool duringHold) {
  reset(); ready(); drive();
  tick(1000);
  if (duringHold) {
    reachNeutral(1000);
    for (int i = 0; i < 100; ++i) {
      tick(1500);
      assert(st().outL == 1500 && reverseMask() == 0);
    }
  }
  const float previous = st().outL;
  tick(2000);
  assert(st().outL == previous + 4 && reverseMask() == 0);
}
static void safetyTest(bool resume) {
  reset(); ready(); drive(2000, 1000, 2000);
  tick(1000, 2000, 1000);  // a reversal is already pending
  for (int i = 0; i < 32; ++i) {
    const float before = st().outL;
    fakeNow += CONTROL_MS;
    setNeutral(); update(CONTROL_MS);
    assert(st().outL == std::max(1500.0f, before - 16.0f));
    assert(reverseMask() == 0);
  }
  assert(st().outL == 1500 && st().outR == 1500 && st().outV == 1500);
  if (resume) {
    tick(1000, 2000, 1000);
    assert(st().outL == 1500 && st().outR == 1500 && st().outV == 1500);
    assert(reverseMask() == 7);
  }
}
static void stallTest() {
  reset(); ready();
  tick(2000, 1000, 2000, 10000);
  assert(st().outL == 1504 && st().outR == 1496 && st().outV == 1504);
  drive();
  tick(1000, 1500, 1500, 10000);
  assert(st().outL == 1996);
  fakeNow += 10000; setNeutral(); update(10000);
  assert(st().outL == 1980);
}
static void wrapTest() {
  reset(UINT32_MAX - 1000);
  fakeNow += 2992; setTarget(2000, 1500, 1500); update(2992);
  assert(!st().ready && st().outL == 1500);
  tick(2000);
  assert(st().ready && st().outL == 1504);
  drive();
  fakeNow = UINT32_MAX - 1200;
  reachNeutral(1000);
  const uint32_t at = fakeNow;
  while ((uint32_t)(fakeNow + CONTROL_MS - at) < REVERSE_NEUTRAL_MS) {
    tick(1000); assert(st().outL == 1500);
  }
  tick(1000);
  assert(st().outL == 1496 && fakeNow < 1000);
}
static void paramsTest() {
  reset(); ready();
  params::d().trimL = 200; params::d().trimR = -200;
  tick(2000, 1000, 1500);
  assert(st().targetL == 2000 && st().targetR == 1000);
  for (int i = 0; i < 130; ++i) tick(2000, 1000, 1500);
  assert(st().outL == 2000 && st().outR == 1000);
  params::d().trimL = params::d().trimR = 0;
  params::d().neutral = 1510; syncParams();
  for (int i = 0; i < 32; ++i) {
    fakeNow += 8; setNeutral(); update(8);
  }
  assert(st().outL == 1510 && st().outR == 1510 && st().outV == 1510);
  tick(1000, 1510, 1510);
  assert(st().outL == 1510 && st().reverseL.waiting);
}
static void invalidDtTest() {
  reset(); ready();
  for (float dt : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(),
                   std::numeric_limits<float>::infinity()}) {
    const float before = st().outL;
    setTarget(2000, 1500, 1500); update(dt);
    assert(std::isfinite(st().outL) && st().outL == before + 0.5f);
  }
}
static void idleTest(bool reinit) {
  reset(); ready();
  if (reinit) {
    drive();
    const unsigned before = writes[0];
    begin();
    assert(writes[0] == before + 1 && duties[0] == usToDuty(1500));
    assert(st().outL == 1500 && !st().ready && reverseMask() == 0);
  } else {
    const unsigned before = writes[0];
    for (int i = 0; i < 100; ++i) tick(1500);
    assert(writes[0] == before && duties[0] == usToDuty(1500));
  }
}
static void randomTest() {
  reset(); ready(); drive(2000, 1000, 2000);
  uint32_t random = 12345;
  int lastDir[3] = {1, -1, 1};
  bool neutral[3] = {};
  uint32_t since[3] = {};
  for (int i = 0; i < 15000; ++i) {
    random = random * 1664525u + 1013904223u;
    const bool stop = (random % 17 == 0);
    const int target = 1000 + (int)(random % 1001);
    const uint32_t dt = random % 97 == 0 ? 1000 : 8;
    const float previous[3] = {st().outL, st().outR, st().outV};
    fakeNow += dt;
    if (stop) setNeutral(); else setTarget(target, 3000 - target, target);
    update((float)dt);
    const float outputs[3] = {st().outL, st().outR, st().outV};
    for (int ch = 0; ch < 3; ++ch) {
      const float out = outputs[ch];
      assert(out >= 1000 && out <= 2000);
      assert(fabsf(out - previous[ch]) <= (stop ? 16 : 4));
      assert(duties[ch] == usToDuty((int)lroundf(out)));
      const int direction = pulseDirection(out, 1500);
      if (direction == 0) {
        if (!neutral[ch]) { since[ch] = fakeNow; neutral[ch] = true; }
      } else {
        if (lastDir[ch] != 0 && direction != lastDir[ch]) {
          assert(neutral[ch]);
          assert((uint32_t)(fakeNow - since[ch]) >= REVERSE_NEUTRAL_MS);
        }
        lastDir[ch] = direction;
        neutral[ch] = false;
      }
    }
  }
}
static void diagnosticsTest() {
  diagnostics::begin();
  assert(diagnostics::st().bootId == 0x12345678);
  assert(std::strcmp(diagnostics::resetName(), "BROWNOUT") == 0);
  diagnostics::noteControlGap(8);
  diagnostics::noteControlGap(40);
  diagnostics::noteControlGap(500);
  diagnostics::noteControlGap(48);
  assert(diagnostics::st().maxControlMs == 500 && diagnostics::st().stalls == 2);
}
int main(int argc, char **argv) {
  assert(argc == 2);
  const std::string name = argv[1];
  if (name == "boot") bootTest();
  else if (name == "drive") driveTest();
  else if (name == "reverse") reverseTest();
  else if (name == "centre") centreTest(false);
  else if (name == "long_neutral") centreTest(true);
  else if (name == "cancel_ramp") cancelTest(false);
  else if (name == "cancel_hold") cancelTest(true);
  else if (name == "safety") safetyTest(false);
  else if (name == "safety_resume") safetyTest(true);
  else if (name == "stall") stallTest();
  else if (name == "wrap") wrapTest();
  else if (name == "params") paramsTest();
  else if (name == "invalid_dt") invalidDtTest();
  else if (name == "idle") idleTest(false);
  else if (name == "reinit") idleTest(true);
  else if (name == "random") randomTest();
  else if (name == "diagnostics") diagnosticsTest();
  else assert(false);
}
