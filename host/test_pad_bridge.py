"""pad_bridge behaviour: no controller, no pygame, no serial.

Run from host/:   python3 -m unittest test_pad_bridge -v

This is the layer both host programs sit on, and its failures are the quiet
kind - a wrong axis sign drives the ROV backwards, a misread port opens the
laptop's Bluetooth modem instead of the board. Every check runs against a fake
controller, so the suite needs no hardware at all.
"""

import os

from unittest import TestCase
from unittest.mock import patch
from types import SimpleNamespace

from pad_bridge import (DEFAULT_DEADMAN_BUTTON, TRIGGER_FULL, TRIGGER_REST,
                        Latch, Pad, Telemetry, btn_name, cmd_line,
                        diagnose_no_telemetry, parse_joy, pick_port,
                        trigger_amount, vertical_throttle)


# ---------------------------------------------------------------------------
#  Stand-ins for pygame and the controller
#
#  The bridge uses a very small slice of pygame - an event queue and a joystick
#  object with axes and buttons - so a few dozen lines cover all of it. Nothing
#  here initialises SDL, so this runs headless.
# ---------------------------------------------------------------------------

class FakeEvent:
    def __init__(self, type_, **kw):
        self.type = type_
        self.__dict__.update(kw)


class FakeJoystick:
    """Fake controller: axis 0 = stick X, axis 1 = stick Y."""

    def __init__(self, idx=0, axes=(0.0, 0.0), buttons=()):
        self.idx = idx
        self.axes = list(axes) + [0.0] * 6
        self.buttons = [1 if i in buttons else 0 for i in range(11)]

    def init(self):
        pass

    def get_name(self):
        return f"FakePad{self.idx}"

    def get_numaxes(self):
        return len(self.axes)

    def get_axis(self, i):
        return self.axes[i]

    def get_numbuttons(self):
        return len(self.buttons)

    def get_button(self, i):
        return self.buttons[i]


class FakePygame:
    JOYDEVICEADDED = 1536
    JOYDEVICEREMOVED = 1537

    def __init__(self, events=(), joystick=None):
        self._events = list(events)
        self._js = joystick
        self.joystick = FakePygame._JoystickModule(joystick)

    class _JoystickModule:
        def __init__(self, js):
            self._js = js

        def Joystick(self, idx):          # noqa: N802 (pygame's interface name)
            if self._js is None:
                raise RuntimeError("no controller")
            return self._js

    @property
    def event(self):
        return self

    def get(self):
        ev, self._events = self._events, []
        return ev


def make_pad(axes=(0.0, 0.0), buttons=(), **kw):
    """A connected Pad with no pygame behind it."""
    p = Pad(kw.get("axis_x", 0), kw.get("axis_y", 1),
            kw.get("deadman", DEFAULT_DEADMAN_BUTTON),
            kw.get("inv_x", False), kw.get("inv_y", False))
    p._pygame = FakePygame()
    p.js = FakeJoystick(axes=axes, buttons=buttons)
    p.name = "FakePad"
    return p


# ---------------------------------------------------------------------------

class StartupTests(TestCase):
    def open_pad(self, platform, arrives_after, background=None):
        clock = SimpleNamespace(now=0.0, pumps=0)

        def pump():
            clock.pumps += 1

        def sleep(seconds):
            clock.now += seconds

        def initialise():
            clock.background = os.environ.get("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS")

        pygame = SimpleNamespace(
            init=initialise,
            joystick=SimpleNamespace(
                init=lambda: None,
                get_count=lambda: int(arrives_after is not None and
                                      clock.pumps >= arrives_after),
                Joystick=FakeJoystick,
            ),
            event=SimpleNamespace(pump=pump),
        )
        pad = Pad(0, 1, DEFAULT_DEADMAN_BUTTON, False, False)
        environment = {} if background is None else {
            "SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS": background}
        with patch.dict("pad_bridge.os.environ", environment, clear=True), \
             patch.dict("sys.modules", {"pygame": pygame}), \
             patch("pad_bridge.sys.platform", platform), \
             patch("pad_bridge.time.monotonic", lambda: clock.now), \
             patch("pad_bridge.time.sleep", sleep):
            opened = pad.open()
        return pad, opened, clock

    def test_background_controller_input_is_enabled_before_sdl_initialises(self):
        for platform in ("darwin", "win32", "linux"):
            with self.subTest(platform=platform):
                _, opened, clock = self.open_pad(platform, arrives_after=0)
                self.assertTrue(opened)
                self.assertEqual(clock.background, "1")

    def test_explicit_background_input_environment_setting_is_preserved(self):
        _, opened, clock = self.open_pad("darwin", arrives_after=0, background="0")
        self.assertTrue(opened)
        self.assertEqual(clock.background, "0")

    def test_mac_waits_for_already_connected_controller_to_be_discovered(self):
        pad, opened, clock = self.open_pad("darwin", arrives_after=3)
        self.assertTrue(opened)
        self.assertEqual(pad.name, "FakePad0")
        self.assertEqual(clock.pumps, 3)

    def test_mac_does_not_delay_when_controller_is_already_enumerated(self):
        _, opened, clock = self.open_pad("darwin", arrives_after=0)
        self.assertTrue(opened)
        self.assertEqual(clock.now, 0.0)
        self.assertEqual(clock.pumps, 0)

    def test_mac_reports_missing_controller_after_bounded_wait(self):
        pad, opened, clock = self.open_pad("darwin", arrives_after=None)
        self.assertFalse(opened)
        self.assertIsNone(pad.js)
        self.assertGreaterEqual(clock.now, 2.0)
        self.assertLess(clock.now, 2.03)

    def test_windows_keeps_immediate_missing_controller_result(self):
        _, opened, clock = self.open_pad("win32", arrives_after=None)
        self.assertFalse(opened)
        self.assertEqual(clock.now, 0.0)
        self.assertEqual(clock.pumps, 0)


class HotPlugTests(TestCase):
    """Unplugging the controller mid-run used to crash the bridge."""

    def test_such_an_event_really_has_no_device_attribute(self):
        probe = FakeEvent(FakePygame.JOYDEVICEADDED, device_index=0)
        self.assertFalse(hasattr(probe, "device"))

    def test_an_add_event_attaches_with_either_attribute_name(self):
        """SDL2 uses device_index, SDL2 older builds used device. Reading the
        wrong one is an AttributeError in the middle of the control loop."""
        for attr in ("device_index", "device"):
            with self.subTest(attr=attr):
                p = make_pad()
                p.js = None
                p._pygame = FakePygame(
                    events=[FakeEvent(FakePygame.JOYDEVICEADDED, **{attr: 0})],
                    joystick=FakeJoystick(0))
                p.pump()
                self.assertIsNotNone(p.js)

    def test_an_add_event_with_neither_attribute_does_not_crash(self):
        p = make_pad()
        p.js = None
        p._pygame = FakePygame(
            events=[FakeEvent(FakePygame.JOYDEVICEADDED, device_index=None)],
            joystick=FakeJoystick(0))
        p.pump()

    def test_unplugging_marks_it_disconnected(self):
        p = make_pad()
        p._pygame = FakePygame(
            events=[FakeEvent(FakePygame.JOYDEVICEREMOVED, device_index=0)],
            joystick=FakeJoystick(0))
        p.pump()
        self.assertIsNone(p.js)
        self.assertFalse(p.connected)

    def test_unknown_event_types_are_ignored(self):
        p = make_pad()
        p.js = FakeJoystick(0)
        p._pygame = FakePygame(events=[FakeEvent(999, whatever=1)])
        p.pump()


class AxisTests(TestCase):
    """Right is +nx, up is +ny. The raw signs are a property of the hardware,
    so what is pinned here is the convention, not the numbers SDL reports."""

    def test_right_on_the_stick_is_positive_nx(self):
        self.assertAlmostEqual(make_pad(axes=(0.83, 0.0)).read().nx, 0.83)

    def test_up_on_the_stick_is_positive_ny(self):
        # Measured: up reads negative on raw axis 1.
        self.assertAlmostEqual(make_pad(axes=(0.0, -0.66)).read().ny, 0.66)

    def test_down_on_the_stick_is_negative_ny(self):
        self.assertAlmostEqual(make_pad(axes=(0.0, 0.66)).read().ny, -0.66)

    def test_invert_flags_negate_the_result(self):
        """Only the inversion is asserted, not the base signs: those change
        with hardware, and hardcoding them makes this go red for no reason."""
        base = make_pad(axes=(0.5, 0.5)).read()
        inv = make_pad(axes=(0.5, 0.5), inv_x=True, inv_y=True).read()
        self.assertGreater(abs(base.nx), 0.1)
        self.assertAlmostEqual(inv.nx, -base.nx)
        self.assertAlmostEqual(inv.ny, -base.ny)

    def test_readings_beyond_one_are_clamped(self):
        """Some controllers overshoot. Passing that on would ask the mixer for
        more than full throttle."""
        state = make_pad(axes=(1.5, -1.5)).read()
        self.assertLessEqual(state.nx, 1.0)
        self.assertLessEqual(state.ny, 1.0)


class TriggerTests(TestCase):
    """The two triggers resolve to one signed value, Z, where RT is up."""

    def test_a_resting_trigger_reads_zero(self):
        self.assertEqual(trigger_amount(TRIGGER_REST), 0.0)

    def test_a_fully_pressed_trigger_reads_one(self):
        self.assertAlmostEqual(trigger_amount(TRIGGER_FULL), 1.0)

    def test_halfway_reads_about_half(self):
        self.assertAlmostEqual(trigger_amount(0.0), 0.5, delta=0.05)

    def test_a_reading_past_the_end_is_clamped_not_extrapolated(self):
        self.assertEqual(trigger_amount(TRIGGER_FULL + 0.5), 1.0)

    def test_right_trigger_alone_gives_positive_z(self):
        self.assertGreater(vertical_throttle(TRIGGER_REST, TRIGGER_FULL), 0.9)

    def test_left_trigger_alone_gives_negative_z(self):
        self.assertLess(vertical_throttle(TRIGGER_FULL, TRIGGER_REST), -0.9)

    def test_neither_trigger_gives_zero(self):
        self.assertEqual(vertical_throttle(TRIGGER_REST, TRIGGER_REST), 0.0)

    def test_both_at_once_stops_rather_than_picking_a_direction(self):
        """Up and down together is a self-contradictory command. Guessing a
        direction would send the ROV the wrong way, which is worse than not
        moving."""
        self.assertEqual(vertical_throttle(TRIGGER_FULL, TRIGGER_FULL), 0.0)

    def test_both_slightly_pressed_also_stops(self):
        self.assertEqual(vertical_throttle(0.0, 0.0), 0.0)

    def test_z_stays_within_range(self):
        self.assertTrue(-1.0 <= vertical_throttle(TRIGGER_REST,
                                                  TRIGGER_FULL + 9) <= 1.0)


class SafetyKeyTests(TestCase):

    def test_not_pressed_means_not_enabled(self):
        self.assertIs(make_pad().read().deadman, False)

    def test_button_down_means_enabled(self):
        p = make_pad(buttons=(DEFAULT_DEADMAN_BUTTON,))
        self.assertIs(p.read().deadman, True)

    def test_an_out_of_range_key_number_does_not_crash(self):
        p = make_pad(buttons=(3,), deadman=99)
        self.assertIs(p.read().deadman, False)

    def test_an_unset_key_does_not_crash(self):
        """--show-input leaves deadman_button as None; reading it must not
        raise."""
        p = make_pad(axes=(0.5, -0.5))
        p.deadman_button = None
        state = p.read()
        self.assertAlmostEqual(state.nx, 0.5)
        self.assertIs(state.deadman, False)


class LatchTests(TestCase):
    """The safety key is a toggle, not a hold. A hold would mean the operator
    has to keep a thumb on it for the whole run."""

    def test_starts_locked(self):
        self.assertIs(Latch().update(False), False)

    def test_one_press_unlocks(self):
        la = Latch()
        self.assertIs(la.update(True), True)

    def test_holding_it_down_does_not_oscillate(self):
        la = Latch()
        la.update(True)
        self.assertIs(la.update(True), True)
        self.assertFalse(la.changed)

    def test_releasing_leaves_it_unlocked(self):
        la = Latch()
        la.update(True)
        self.assertIs(la.update(False), True)

    def test_pressing_again_locks_it(self):
        la = Latch()
        la.update(True)
        la.update(False)
        self.assertIs(la.update(True), False)

    def test_still_locked_after_release(self):
        la = Latch()
        la.update(True)
        la.update(False)
        la.update(True)
        self.assertIs(la.update(False), False)

    def test_pad_loss_re_locks(self):
        la = Latch()
        la.update(True)
        la.force_lock()
        self.assertTrue(la.locked)
        self.assertIs(la.update(False), False)

    def test_force_lock_when_already_locked_reports_no_event(self):
        self.assertIs(Latch().update(False), False)


class CommandFormatTests(TestCase):
    """The board parses this by hand, so the exact text is part of the
    interface."""

    def test_neutral_and_locked(self):
        self.assertEqual(cmd_line(0.0, 0.0, 0.0, False),
                         "CMD nx=+0.000 ny=+0.000 vz=+0.000 en=0\n")

    def test_forward_right_and_descending(self):
        self.assertEqual(cmd_line(0.42, -0.17, -0.5, True),
                         "CMD nx=+0.420 ny=-0.170 vz=-0.500 en=1\n")


class TelemetryParsingTests(TestCase):

    def test_a_well_formed_line_parses(self):
        t = Telemetry()
        ok = parse_joy("JOY X=+0.42 Y=-0.17 Z=-0.50 MAG=0.45 OUT READY=1 "
                       "L=1718 R=1282 V=1000 TL=1718 TR=1282 EN=1 LINK=1", t)
        self.assertTrue(ok and t.seen)

    def test_fields_land_where_they_should(self):
        t = Telemetry()
        parse_joy("JOY X=+0.42 Y=-0.17 Z=-0.50 MAG=0.45 OUT READY=1 "
                  "L=1718 R=1282 V=1000 TL=1718 TR=1282 EN=1 LINK=1", t)
        self.assertAlmostEqual(t.x, 0.42)
        self.assertAlmostEqual(t.z, -0.5)
        self.assertEqual(t.l_us, 1718)
        self.assertEqual(t.v_us, 1000)
        self.assertTrue(t.en)
        self.assertTrue(t.link)
        self.assertFalse(t.in_dead)

    def test_a_board_without_the_vertical_thruster_still_parses(self):
        """The V field was added later. An older board must not break the
        console, or the two halves can no longer be updated separately."""
        self.assertTrue(parse_joy("JOY X=0 Y=0 MAG=0 DEAD READY=1 L=1500 R=1500 "
                                  "EN=0 LINK=0", Telemetry()))

    def test_an_unknown_extra_field_does_not_break_it(self):
        self.assertTrue(parse_joy("JOY X=0 Y=0 Z=0 MAG=0 DEAD READY=1 L=1500 "
                                  "R=1500 V=1500 EN=0 LINK=0 FUTURE=7",
                                  Telemetry()))

    def test_a_non_joy_line_is_rejected(self):
        self.assertFalse(parse_joy("ST state=READY wifi=IDLE", Telemetry()))


class PortPickingTests(TestCase):
    """Picking the wrong port opens a Bluetooth modem or the laptop's own
    serial console and then looks like a dead board."""

    def test_picks_the_board_out_of_a_real_macos_list(self):
        mac_list = ["/dev/cu.debug-console", "/dev/cu.iFLYBUDSPro2",
                    "/dev/cu.Bluetooth-Incoming-Port",
                    "/dev/cu.usbserial-A5069RR4"]
        chosen, _ = pick_port(mac_list)
        self.assertEqual(chosen, "/dev/cu.usbserial-A5069RR4")

    def test_with_only_system_ports_it_refuses_to_guess(self):
        chosen, _ = pick_port(["/dev/cu.debug-console",
                               "/dev/cu.Bluetooth-Incoming-Port"])
        self.assertIsNone(chosen)

    def test_two_identical_looking_candidates_means_no_guess(self):
        chosen, _ = pick_port(["/dev/cu.usbserial-A", "/dev/cu.usbserial-B"])
        self.assertIsNone(chosen)

    def test_a_single_candidate_wins_even_if_the_name_looks_odd(self):
        chosen, _ = pick_port(["/dev/ttyS0"])
        self.assertEqual(chosen, "/dev/ttyS0")

    def test_bridge_chip_is_preferred_over_the_native_usb_port(self):
        """Both are the same board; the bridge one is the one that survives a
        reflash without the port disappearing."""
        chosen, _ = pick_port(["/dev/cu.debug-console", "/dev/cu.usbmodem101",
                               "/dev/cu.usbserial-A5069RR4"])
        self.assertEqual(chosen, "/dev/cu.usbserial-A5069RR4")

    # On Windows the device name is just COMx, so the chip can only be
    # identified from the description. These are the strings pyserial reports.

    def test_windows_ftdi_bridge_beats_the_motherboard_and_native_ports(self):
        chosen, _ = pick_port([("COM1", "Communications Port"),
                               ("COM5", "USB Serial Port"),
                               ("COM6", "USB Serial Device")])
        self.assertEqual(chosen, "COM5")

    def test_windows_a_lone_bluetooth_port_is_not_the_board(self):
        chosen, _ = pick_port([("COM3", "Bluetooth Serial Port")])
        self.assertIsNone(chosen)

    def test_windows_cp210x_bridge_is_recognised(self):
        chosen, _ = pick_port([("COM7", "Silicon Labs CP210x USB to UART Bridge")])
        self.assertEqual(chosen, "COM7")

    def test_windows_ch340_bridge_is_recognised(self):
        chosen, _ = pick_port([("COM4", "USB-SERIAL CH340 (COM4)")])
        self.assertEqual(chosen, "COM4")

    def test_windows_the_boards_own_usb_port_alone_is_still_usable(self):
        chosen, _ = pick_port([("COM2", "USB Serial Device")])
        self.assertEqual(chosen, "COM2")

    def test_a_silent_port_produces_an_actionable_message(self):
        text = diagnose_no_telemetry("/dev/cu.usbserial-A5069RR4")
        self.assertGreater(len(text), 20)


class ButtonLabelTests(TestCase):

    def test_sdl_numbers_map_to_the_expected_names(self):
        self.assertEqual(btn_name(0), "A")
        self.assertEqual(btn_name(1), "B")
        self.assertEqual(btn_name(2), "X")
        self.assertEqual(btn_name(3), "Y")
        self.assertEqual(btn_name(5), "RB")

    def test_an_unknown_number_is_shown_as_is_not_guessed(self):
        self.assertEqual(btn_name(42), "btn42")

    def test_the_safety_key_defaults_to_y(self):
        self.assertEqual(DEFAULT_DEADMAN_BUTTON, 3)


if __name__ == "__main__":
    import unittest
    unittest.main()
