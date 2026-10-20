"""Compile the shipped ESC controller against hardware stand-ins, then exercise it.

Only the headers for hardware and saved parameters are replaced. The production
controller/configuration/diagnostics are copied unchanged into a temporary build.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"

ARDUINO = r"""
#pragma once
#include <stdint.h>
#include <algorithm>
extern uint32_t fakeNow;
extern uint32_t duties[3];
extern unsigned writes[3];
inline uint32_t millis() { return fakeNow; }
template<typename T> T constrain(T v, T lo, T hi) {
  return std::min(std::max(v, lo), hi);
}
inline uint32_t ledcSetup(int, int frequency, int) { return frequency; }
inline void ledcAttachPin(int, int) {}
inline void ledcWrite(unsigned ch, uint32_t duty) {
  duties[ch] = duty;
  ++writes[ch];
}
"""

PARAMS = r"""
#pragma once
namespace params {
struct Data {
  int neutral = ESC_US_NEUTRAL;
  int trimL = 0, trimR = 0, trimV = 0;
};
inline Data& d() { static Data value; return value; }
}
"""

SYSTEM = r"""
#pragma once
enum esp_reset_reason_t {
  ESP_RST_UNKNOWN, ESP_RST_POWERON, ESP_RST_EXT, ESP_RST_SW,
  ESP_RST_PANIC, ESP_RST_INT_WDT, ESP_RST_TASK_WDT, ESP_RST_WDT,
  ESP_RST_DEEPSLEEP, ESP_RST_BROWNOUT, ESP_RST_SDIO
};
inline esp_reset_reason_t esp_reset_reason() { return ESP_RST_BROWNOUT; }
inline unsigned esp_random() { return 0x12345678; }
"""


class ThrusterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("c++") or shutil.which("g++")
        if not compiler:
            raise unittest.SkipTest("Install a C++ compiler to run firmware regressions")
        cls.temp = tempfile.TemporaryDirectory(prefix="uwr-thrusters-")
        cls.addClassCleanup(cls.temp.cleanup)
        build = Path(cls.temp.name)
        for name in ("thrusters.h", "config.h", "diagnostics.h"):
            shutil.copyfile(SRC / name, build / name)
        (build / "Arduino.h").write_text(ARDUINO)
        (build / "params.h").write_text(PARAMS)
        (build / "esp_system.h").write_text(SYSTEM)
        cls.binary = build / "test_thrusters"
        subprocess.run(
            [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
             "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
             "-I", str(build), str(HERE / "thrusters_test.cpp"),
             "-o", str(cls.binary)],
            check=True, capture_output=True, text=True,
        )

    def run_case(self, name):
        result = subprocess.run([str(self.binary), name], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_boot_holds_neutral(self): self.run_case("boot")
    def test_normal_drive_rate_and_independent_channels(self): self.run_case("drive")
    def test_both_reversal_directions_hold_neutral(self): self.run_case("reverse")
    def test_centring_stick_cannot_bypass_hold(self): self.run_case("centre")
    def test_long_neutral_already_satisfies_hold(self): self.run_case("long_neutral")
    def test_cancel_reversal_during_deceleration(self): self.run_case("cancel_ramp")
    def test_cancel_reversal_during_hold(self): self.run_case("cancel_hold")
    def test_safety_stop_keeps_original_rate(self): self.run_case("safety")
    def test_reverse_after_safety_stop_still_waits(self): self.run_case("safety_resume")
    def test_stalled_control_cannot_jump_to_target(self): self.run_case("stall")
    def test_hold_and_boot_across_millis_wrap(self): self.run_case("wrap")
    def test_trims_custom_neutral_and_output_bounds(self): self.run_case("params")
    def test_invalid_dt_is_bounded(self): self.run_case("invalid_dt")
    def test_unchanged_outputs_keep_pwm_running(self): self.run_case("idle")
    def test_reinitialisation_rewrites_neutral(self): self.run_case("reinit")
    def test_random_commands_never_bypass_hold_or_limits(self): self.run_case("random")
    def test_diagnostics_capture_reset_and_control_gaps(self): self.run_case("diagnostics")


if __name__ == "__main__":
    unittest.main()
