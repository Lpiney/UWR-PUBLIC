// ============================================================================
//  config.h - global configuration: pins, timing, defaults.
//  Every magic number lives here; no other file hardcodes one.
// ============================================================================
#pragma once

// ---------------------------------------------------------------------------
//  Thruster pins
//
//  Which ESP32-S3 pins are usable? Rule out these first:
//    26 ~ 32     wired to the internal SPI flash, never usable
//    33 ~ 37     taken by octal PSRAM on N8R8 / N16R8
//    19 / 20     native USB D- / D+, needed for comms
//    43 / 44     UART0 (the "COM" port), keep for debug output
//    0/3/45/46   strapping pins, they have a level requirement at boot
//  Of what is left, 4, 5 and 6 are the cleanest and all three are broken out.
//  GPIO1-10 are also the ADC1 channels, unused here since the stick lives on
//  the laptop.
// ---------------------------------------------------------------------------
#define PIN_ESC_L 4      // left/right thrusters
#define PIN_ESC_R 5
#define PIN_ESC_V 6      // vertical thruster, driven by the two triggers

// LEDC channels (the S3 has 8; pick three nobody else uses)
#define ESC_CH_L 0
#define ESC_CH_R 1
#define ESC_CH_V 2

// ---------------------------------------------------------------------------
//  ESC PWM
//  Industry standard: 50 Hz refresh, 1000-2000 us pulse width as the throttle
//  range, 1500 us neutral.
// ---------------------------------------------------------------------------
#define ESC_PWM_FREQ_HZ   50
#define ESC_PWM_RES_BITS  14        // S3 LEDC maximum; at 50 Hz one step ~= 1.22 us
#define ESC_PERIOD_US     20000     // 1 / 50 Hz = 20 ms

#define ESC_US_MIN 1000
#define ESC_US_MAX 2000

// ---------------------------------------------------------------------------
//  Check this before powering the ESCs for the first time.
//
//  Bidirectional ESC (can reverse, the usual choice for an ROV): neutral 1500
//  Unidirectional ESC (forward only)                          : neutral 1000
//
//  Getting it wrong means the propellers spin at full throttle the instant
//  power is applied. Always remove the propellers for a first power-up.
//
//  Changeable in the field without reflashing: send  P neutral 1500
// ---------------------------------------------------------------------------
#define ESC_US_NEUTRAL 1500

// After power-up, ignore the input for this long and hold neutral.
//
// The ESCs run a self-test and beep on power-up; if a hand is resting on the
// stick during that window the ROV would take off immediately. The window
// gives the ESCs time to finish and the operator time to let go.
#define BOOT_IGNORE_MS 3000

// Throttle slew limit: maximum change in pulse width per millisecond.
// A full 1000 us sweep takes 500 ms, so the motors never jump straight to
// full throttle. Raise it for a snappier feel, lower it to be gentler.
#define SLEW_US_PER_MS 2.0f

// ---------------------------------------------------------------------------
//  Input shaping
// ---------------------------------------------------------------------------
// Stick deadzone. The rescaling is linear rather than a hard cut, so the
// output starts from zero continuously instead of jumping as the stick leaves
// the deadzone.
#define PULSE_DEADZONE 0.15f

// ---------------------------------------------------------------------------
//  Link
// ---------------------------------------------------------------------------
// No valid command for this long means the link is dead -> force neutral.
// This is the only thing standing between a crashed laptop and an ROV running
// away at the last commanded throttle. Do not raise it much.
// 200 ms is about 10 command periods, so normal jitter never trips it.
#define LINK_TIMEOUT_MS 200

// Maximum length of a protocol line; longer lines are discarded whole so that
// garbage on the wire cannot overflow the buffer.
#define LINK_LINE_MAX 256

#define SERIAL_BAUD 115200

// ---------------------------------------------------------------------------
//  Loop timing
// ---------------------------------------------------------------------------
#define CONTROL_MS   8      // motor control tick, ~125 Hz (has to feel responsive)
#define TELEMETRY_MS 50     // telemetry line, ~20 Hz
#define STATUS_MS    500    // Mission 1 status line, ~2 Hz

// ---------------------------------------------------------------------------
//  Mission 1 (Wi-Fi + HTTP)
// ---------------------------------------------------------------------------
// How often to retry a failed association.
//
// Do not lower this. Many home routers damp repeated association attempts from
// the same device; the ESP32 then reports
//   "Association refused temporarily, comeback time ... too long"
// with a wrapped-around comeback time, and waits thousands of seconds. 5 s
// trips this on the routers we tested; 15 s is fine.
//
// If the tower cannot be reached on competition day, raise this
// (P wifiRetryMs 30000) instead of power-cycling the ROV.
#define WIFI_RETRY_MS   15000
#define HTTP_TIMEOUT_MS 2500      // single HTTP request timeout

#define M1_TASK_STACK 8192        // network task stack
#define M1_TASK_CORE  0           // network task is pinned to core 0 so the
                                  // control loop keeps core 1 to itself
