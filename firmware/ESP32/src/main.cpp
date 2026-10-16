// ============================================================================
//  CityUHK UR Fall Training 2026 - ROV main controller firmware
//
//  One ESP32-S3 does the whole job:
//      - receives line-protocol commands from the surface laptop
//      - mixes them into two thruster pulse widths and drives the ESCs
//      - joins Wi-Fi and fetches from HTTP by itself (Mission 1)
//
//  There is no arm/disarm state machine. Two gates protect the motors instead,
//  and both are enforced here:
//      1) the safety key, reported by the laptop as en=1 in every command
//      2) the link timeout, which forces neutral when commands stop arriving
//
//  System split:
//      core 1 (the Arduino loop)   comms + thrusters; must never block
//      core 0 (the mission1 task)  Wi-Fi + HTTP; may block freely
//      the two talk only through one FreeRTOS queue, so no locking is needed
// ============================================================================

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "params.h"
#include "thrusters.h"
#include "mission1.h"
#include "comms.h"

// ---------------------------------------------------------------------------
//  Mixer: normalised stick position -> left and right pulse widths.
//
//  MEASURED ON THE VEHICLE (2026-10-16), not derived from a drawing. The two
//  thrusters are mounted anti-parallel: driving them equally spins the boat,
//  and driving them oppositely moves it fore and aft. That is the reverse of
//  an ordinary differential drive, so the two channels swap roles here:
//
//      thrust = ny             turn = nx
//      left   = thrust + turn
//      right  = turn  - thrust
//      then linearly mapped onto [1000, 2000] us, neutral 1500
//
//      ny=+1, nx= 0 -> L=2000 R=1000   travels forward
//      ny=-1, nx= 0 -> L=1000 R=2000   travels back
//      ny= 0, nx=+1 -> L=2000 R=2000   spins, nose to the right
//      ny= 0, nx=-1 -> L=1000 R=1000   spins, nose to the left
//
//  The host guarantees the sign convention: nx>0 means the stick is to the
//  right, ny>0 means forward (pad_bridge.py owns the raw axis signs).
//
//  If the thrusters are ever remounted so that they match each other, this
//  comes out backwards and wants the ordinary differential form instead:
//      left  = thrust + turn
//      right = thrust - turn
//  Which of the two is in force is a property of the vehicle, so it is
//  recorded here with the date it was measured rather than guessed at.
//
//  There is no reversing-steering convention here. It used to flip the
//  steering sense when the forward channel went negative, which is how a car
//  reverses. With the channels swapped that would flip on left and right
//  instead, which is not the same thing at all. This is tank steering.
// ---------------------------------------------------------------------------

// Deadzone with linear rescaling.
//
// Rescaling matters: a hard cut would make the output jump from 0 straight to
// the deadzone value, which feels like a notch as the stick leaves centre.
// Rescaling restarts the output from zero continuously.
inline float applyDeadzone(float v) {
  const float a = fabsf(v);
  if (a <= PULSE_DEADZONE) return 0.0f;
  const float n = (a - PULSE_DEADZONE) / (1.0f - PULSE_DEADZONE);
  return (v < 0.0f) ? -n : n;
}

inline int unitToPulse(float v) {
  if (v >  1.0f) v =  1.0f;
  if (v < -1.0f) v = -1.0f;
  return (int)lroundf(ESC_US_NEUTRAL + v * (ESC_US_MAX - ESC_US_NEUTRAL));
}

inline void mixToTargets(float nx, float ny, int &targetL, int &targetR) {
  const float thrust = applyDeadzone(ny);
  const float turn   = applyDeadzone(nx);

  targetL = unitToPulse(thrust + turn);
  targetR = unitToPulse(turn - thrust);
}

// ---------------------------------------------------------------------------

// Translate events from the network task into protocol lines.
static void drainMission1() {
  if (!mission1::q()) return;

  mission1::Ev e;
  while (xQueueReceive(mission1::q(), &e, 0) == pdTRUE) {
    if (strcmp(e.tag, "DATA") == 0) comms::data(String(e.msg));
    else                            comms::event(String(e.msg));
  }
}

void setup() {
  // Both serial ports are opened:
  //   Serial  = native USB (the port labelled "USB"), the control link
  //   Serial0 = UART0 (the port labelled "COM"), same protocol, for a monitor
  Serial0.begin(SERIAL_BAUD);
  Serial.begin(SERIAL_BAUD);
  delay(200);   // let the ports settle, otherwise the first lines are lost

  params::begin();      // parameters first; other modules depend on them
  thrusters::begin();   // put neutral on the ESCs immediately
  mission1::begin();    // start the network task on core 0
  comms::begin();

  comms::event("UWR ROV firmware ready");
  comms::event("commands: CMD nx= ny= en= | P/G/R=params | W/F=mission1 | S=status | B=reboot");
  comms::event("boot window " + String(BOOT_IGNORE_MS) +
               " ms; link timeout " + String(params::d().fsMs) + " ms");
  comms::event("remove the propellers before the first powered test");
}

void loop() {
  const uint32_t now = millis();

  comms::poll();          // receive commands, parse, reply

  // ---- mixer ----
  // Two gates, both required to drive: the link must be alive, and the
  // safety key must be unlocked. Anything else means neutral.
  const comms::Command &c = comms::cmd();
  if (comms::linkOk(now) && c.enable) {
    int l = 0, r = 0;
    mixToTargets(c.nx, c.ny, l, r);
    // The host has already resolved the two triggers into one signed value -
    // including the rule that pressing both at once means stop - so vz needs
    // no mixing. It is negated on the way out because this vehicle's vertical
    // thruster is the other way round from its ESC: 2000 drives it down.
    // Measured 2026-10-16. vz itself stays "positive is up", which is what the
    // Z= field in the telemetry means and what the console labels from.
    thrusters::setTarget(l, r, unitToPulse(-c.vz));
  } else {
    thrusters::setNeutral();
  }

  // ---- motor update, fixed 8 ms tick ----
  static uint32_t lastControl = 0;
  if ((int32_t)(now - lastControl) >= CONTROL_MS) {
    const float dtMs = (float)(now - lastControl);
    lastControl = now;
    thrusters::update(dtMs);
  }

  drainMission1();          // report network events upstream
  comms::tickTelemetry();   // JOY + ST lines on their own schedule

  // Yield so the idle task (and the watchdog) keep running.
  vTaskDelay(pdMS_TO_TICKS(1));
}
