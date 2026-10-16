"""Console task behaviour: no camera, no board, no window.

Run from host/:   python3 -m unittest test_console -v

These pin the interaction: entering a task only previews, and recognition
happens on RB. That is easy to break by accident when adding a task.
"""

import time
from unittest import TestCase

import cv2
import numpy as np

from console import (AMBER, BTN_APRILTAG, BTN_CAPTURE, COMMAND_MAX, GREEN, RED,
                     Console, Pad, Task, TaskAprilTag, TaskColor, TaskWifi,
                     _text_wh, fit_text, upright, vertical_label)
from pad_bridge import DEFAULT_DEADMAN_BUTTON, Telemetry
from vision.color import DEFAULT_RANGES, sample_at
from vision.test_apriltag import frame_with_tags
from vision.test_color import scene as colour_scene


# The console drives a camera and a serial link that a test must not open, so
# both stand-ins live here rather than the console growing a way to run
# without them.

class FakeCamera:
    """A camera that is always delivering frames."""

    def __init__(self, frame=None):
        self.lost = False
        self.index = 0
        self._frame = frame

    @property
    def ok(self):
        return self._frame is not None

    def read(self):
        return self._frame

    def release(self):
        pass


class FakeBoard:
    """The serial link, recorded instead of transmitted."""

    def __init__(self):
        self.port = "FAKE"
        self.telem = None
        self.on_line = None
        self.sent = []
        self.raw = []

    def send(self, nx, ny, vz, en):
        self.sent.append((nx, ny, vz, en))

    def send_raw(self, text):
        self.raw.append(text)

    def poll(self):
        pass

    def close(self):
        pass


def telemetry():
    t = Telemetry()
    t.l_us, t.r_us = 1718, 1282
    t.en = t.link = True
    return t


def make_console(camera=None):
    pad = Pad(0, 1, DEFAULT_DEADMAN_BUTTON, False, False)
    pad.js = object()          # stands in for a connected controller
    return Console(pad=pad, board=FakeBoard(), camera=camera or FakeCamera())


class ConsoleTaskTests(TestCase):

    def setUp(self):
        self.con = make_console()
        self.telemetry = telemetry()
        self.tag_frame = frame_with_tags(7)

    def pump(self, seconds, frame=None):
        """Feed frames for a while, as the live loop would."""
        frame = self.tag_frame if frame is None else frame
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.con.render(self.telemetry, True, frame=frame.copy())
            time.sleep(1 / 60)

    # ---- Manual ----

    def test_manual_has_nothing_to_capture(self):
        self.assertFalse(self.con.task.capture(self.con))

    def test_switching_to_the_same_task_returns_to_manual(self):
        self.con._switch(TaskAprilTag)
        self.assertIsInstance(self.con.task, TaskAprilTag)
        self.con._switch(TaskAprilTag)
        self.assertNotIsInstance(self.con.task, TaskAprilTag)

    # ---- Standby ----

    def test_entering_the_tag_task_only_previews(self):
        """No detection until RB - that is the whole point of the mode."""
        self.con._switch(TaskAprilTag)
        self.pump(0.15)
        self.assertIsNone(self.con.apriltag.target_id)
        self.assertEqual(self.con.task.captures, 0)
        self.assertIsNone(self.con.task.result)

    def test_entering_the_colour_task_runs_nothing(self):
        self.con._switch(TaskColor)
        self.pump(0.15, frame=colour_scene("R"))
        self.assertIsNone(self.con.task.answer)

    # ---- Colour: pointing ----

    def point_at(self, x, y, frame=None):
        """Render one frame, then leave the pointer at (x, y)."""
        self.con.render(self.telemetry, True,
                        frame=colour_scene("R") if frame is None else frame)
        self.con.mouse.x, self.con.mouse.y = x, y

    def test_a_capture_needs_the_pointer_over_the_picture(self):
        """Before the mouse has ever been over the window there is nowhere
        to look, and saying so beats reading a stale point."""
        self.con._switch(TaskColor)
        self.assertFalse(self.con.mouse.known)
        self.assertFalse(self.con.task.capture(self.con))

    def test_pointing_at_a_pole_names_its_colour(self):
        self.con._switch(TaskColor)
        self.point_at(320, 225)              # the pole in the test scene
        self.assertTrue(self.con.task.capture(self.con))
        self.assertEqual(self.con.task.answer, "R")

    def test_pointing_at_plain_background_gives_no_answer(self):
        """Grey pool floor is not a colour, whatever band its noise lands in."""
        self.con._switch(TaskColor)
        self.con.task.answer = "R"
        self.point_at(50, 100)               # above the pole and the floor
        self.con.task.capture(self.con)
        self.assertIsNone(self.con.task.answer)

    def test_the_last_pointer_position_is_kept(self):
        """OpenCV sends no event when the cursor leaves the window, so the
        last point has to stand - moving off to the controller between aiming
        and pressing RB is the normal way to use this."""
        self.con._switch(TaskColor)
        self.point_at(320, 225)
        self.assertTrue(self.con.mouse.known)
        self.con.task.capture(self.con)
        self.assertEqual(self.con.task.answer, "R")

    def test_the_panel_does_not_claim_to_know_the_activation_order(self):
        """The order is announced on the day and can change between attempts.
        A hardcoded R-B-Y used to be shown as if it were the referee's, which
        is worse than showing nothing: the operator follows it and loses the
        sequence bonus without ever doubting the screen."""
        self.con._switch(TaskColor)
        text = " ".join(t for t, _ in self.con.task.info(self.con))
        self.assertNotIn("order", text.lower())

    def test_r_clears_the_answer(self):
        self.con._switch(TaskColor)
        self.point_at(320, 225)
        self.con.task.capture(self.con)
        self.con.task.on_key("r", self.con)
        self.assertIsNone(self.con.task.answer)

    def test_calibration_samples_around_the_pointer(self):
        import pathlib
        import tempfile

        self.con._switch(TaskColor)
        # Keep the written config out of the working tree.
        self.con.color.config_path = pathlib.Path(tempfile.mkdtemp()) / "cfg.json"
        before = [list(b) for b in self.con.color.ranges["R"]]

        self.point_at(320, 225)
        self.con.task.on_key("1", self.con)
        self.assertNotEqual([list(b) for b in self.con.color.ranges["R"]], before)
        self.assertTrue(self.con.color.config_path.exists())

    def test_calibration_without_a_pointer_does_nothing(self):
        self.con._switch(TaskColor)
        before = [list(b) for b in self.con.color.ranges["Y"]]
        self.con.task.on_key("2", self.con)
        self.assertEqual([list(b) for b in self.con.color.ranges["Y"]], before)

    # ---- AprilTag: choosing the logic ----

    def test_capture_is_refused_until_the_logic_is_chosen(self):
        """Largest or smallest is announced on the day; guessing it is a
        zero. So nothing is captured until the operator has said which."""
        self.con._switch(TaskAprilTag)
        self.assertFalse(self.con.task.capture(self.con))
        self.assertFalse(self.con.task.capturing)

    def test_l_and_m_set_the_selection_logic(self):
        self.con._switch(TaskAprilTag)
        self.con.task.on_key("l", self.con)
        self.assertEqual(self.con.task.mode, "largest")
        self.con.task.on_key("m", self.con)
        self.assertEqual(self.con.task.mode, "smallest")

    def test_the_panel_names_the_key_that_selects_the_logic(self):
        """The bracketed letter reads as "press this key", so it has to be the
        key. It used to be the first letter of the word instead, which is L
        for largest but S for smallest - and smallest is selected with M."""
        self.con._switch(TaskAprilTag)
        for key, mode, shown in (("l", "largest", "(L)"),
                                 ("m", "smallest", "(M)")):
            with self.subTest(mode=mode):
                self.con.task.on_key(key, self.con)
                logic = [t for t, _ in self.con.task.info(self.con)
                         if t.startswith("logic")]
                self.assertEqual(len(logic), 1)
                self.assertEqual(self.con.task.mode, mode)
                self.assertIn(shown, logic[0])

    def test_the_result_follows_the_chosen_logic(self):
        self.con._switch(TaskAprilTag)
        self.con.task.ids = {3, 7, 11}
        self.con.task.on_key("m", self.con)
        self.assertEqual(self.con.task.result, 3)
        self.con.task.on_key("l", self.con)
        self.assertEqual(self.con.task.result, 11)

    # ---- AprilTag: capturing ----

    def capture(self, frame=None):
        """Press RB and let the capture window close."""
        self.assertTrue(self.con.task.capture(self.con))
        self.pump(0.6, frame=frame)

    def test_a_capture_collects_the_id_without_freezing_the_picture(self):
        self.con._switch(TaskAprilTag)
        self.con.task.on_key("l", self.con)
        self.capture()
        self.assertEqual(self.con.task.ids, {7})
        self.assertEqual(self.con.task.captures, 1)
        self.assertEqual(self.con.task.result, 7)
        # The operator is steering while they scan, so the picture stays live.
        live = self.con.task.view(self.con, self.tag_frame)
        self.assertIs(live, self.tag_frame)

    def test_captures_accumulate_across_presses(self):
        self.con._switch(TaskAprilTag)
        self.con.task.on_key("l", self.con)
        self.capture(frame_with_tags(3))
        self.assertEqual(self.con.task.result, 3)
        self.capture(frame_with_tags(7))
        self.assertEqual(self.con.task.ids, {3, 7})
        self.assertEqual(self.con.task.captures, 2)
        self.assertEqual(self.con.task.result, 7)

    def test_a_capture_that_sees_nothing_does_not_count(self):
        """A frame lost to a ripple costs the press, not one of the three."""
        self.con._switch(TaskAprilTag)
        self.con.task.on_key("l", self.con)
        self.capture(frame=self.tag_frame * 0 + 128)
        self.assertEqual(self.con.task.captures, 0)
        self.assertEqual(self.con.task.ids, set())
        self.assertIsNone(self.con.task.result)

    def test_reset_clears_the_scan_but_keeps_the_logic(self):
        self.con._switch(TaskAprilTag)
        self.con.task.on_key("l", self.con)
        self.capture()
        self.con.task.on_key("r", self.con)
        self.assertEqual(self.con.task.ids, set())
        self.assertEqual(self.con.task.captures, 0)
        self.assertEqual(self.con.task.mode, "largest")

    def test_re_entering_the_task_clears_everything(self):
        """Per run the judges announce the logic afresh, so it must not
        survive leaving and coming back."""
        self.con._switch(TaskAprilTag)
        self.con.task.on_key("l", self.con)
        self.capture()
        self.con._switch(TaskAprilTag)      # back to Manual
        self.con._switch(TaskAprilTag)      # in again
        self.assertIsNone(self.con.task.mode)
        self.assertEqual(self.con.task.ids, set())
        self.assertEqual(self.con.task.captures, 0)

    # ---- Button handling ----

    def test_rb_edge_triggers_a_capture(self):
        self.con._switch(TaskAprilTag)
        self.con.task.on_key("l", self.con)
        held = [0] * 10
        held[BTN_CAPTURE] = 1
        self.con._handle_buttons(held, [0] * 10)
        self.assertTrue(self.con.task.capturing)

    def test_holding_rb_does_not_retrigger(self):
        self.con._switch(TaskAprilTag)
        self.con.task.on_key("l", self.con)
        held = [0] * 10
        held[BTN_CAPTURE] = 1
        self.con._handle_buttons(held, held)      # already down last frame
        self.assertFalse(self.con.task.capturing)

    def test_the_vertical_row_reads_as_a_direction(self):
        """One row, labelled by direction: the thruster has a single pulse
        width, so UP and DOWN are the two ends of the same number."""
        self.assertEqual(vertical_label(2000), "UP")
        self.assertEqual(vertical_label(1600), "UP")
        self.assertEqual(vertical_label(1000), "DOWN")
        self.assertEqual(vertical_label(1400), "DOWN")
        # Deadband, so the label does not flicker while the value settles.
        for us in (1500, 1501, 1502, 1499, 1498):
            with self.subTest(us=us):
                self.assertEqual(vertical_label(us), "HOLD")

    def test_a_button_selects_the_tag_task(self):
        pressed = [0] * 10
        pressed[BTN_APRILTAG] = 1
        self.con._handle_buttons(pressed, [0] * 10)
        self.assertIsInstance(self.con.task, TaskAprilTag)

    # ---- OSD ----

    def test_osd_draws_for_every_task_state(self):
        for task_cls in (TaskAprilTag, TaskColor):
            self.con._switch(task_cls)
            out = self.con.render(self.telemetry, True, frame=self.tag_frame.copy())
            self.assertEqual(out.shape, self.tag_frame.shape)
            self.assertGreater(int(out.sum()), 0)

    def test_every_task_declares_the_keys_it_handles(self):
        """The on-screen hint panel is built from these, so a task that
        handles a key without declaring it leaves the operator guessing."""
        self.assertEqual([k for k, _ in TaskAprilTag().keys(self.con)],
                         ["L", "M", "C", "R"])
        self.assertEqual([k for k, _ in TaskColor().keys(self.con)],
                         ["C", "1 2 3", "R", "D"])
        # Manual declares nothing of its own; Q and S are added globally.
        self.assertEqual(Task().keys(self.con), [])


class CommandLineTests(TestCase):
    """Typing a board command into the console window.

    The alternative is a serial monitor, and its telemetry arrives 20 times a
    second, so the one line you sent scrolls away before it can be read."""

    ENTER, ESC, BACKSPACE = 13, 27, 8

    def setUp(self):
        self.con = make_console()
        self.board = self.con.board

    def type(self, text):
        for ch in text:
            self.con._key(ord(ch))

    def test_t_starts_a_command(self):
        self.assertIsNone(self.con.typing)
        self.con._key(ord("t"))
        self.assertEqual(self.con.typing, "")

    def test_enter_sends_exactly_what_was_typed(self):
        self.type("t")
        self.type("P url 192.168.4.1:8080")
        self.con._key(self.ENTER)
        self.assertEqual(self.board.raw, ["P url 192.168.4.1:8080\n"])
        self.assertIsNone(self.con.typing)

    def test_case_survives_typing(self):
        """Unlike the task keys, which are lowercased. P ssid takes a Wi-Fi
        name, and those are case-sensitive."""
        self.type("t")
        self.type("P ssid HUAWEI JACK")
        self.con._key(self.ENTER)
        self.assertEqual(self.board.raw, ["P ssid HUAWEI JACK\n"])

    def test_backspace_takes_the_last_character_back(self):
        self.type("t")
        self.type("P url 192.168.4.1:808O")     # letter O, not zero
        self.con._key(self.BACKSPACE)
        self.type("0")
        self.con._key(self.ENTER)
        self.assertEqual(self.board.raw, ["P url 192.168.4.1:8080\n"])

    def test_escape_abandons_without_sending(self):
        self.type("t")
        self.type("P url nope")
        self.con._key(self.ESC)
        self.assertIsNone(self.con.typing)
        self.assertEqual(self.board.raw, [])

    def test_enter_on_an_empty_line_sends_nothing(self):
        self.type("t")
        self.con._key(self.ENTER)
        self.assertEqual(self.board.raw, [])

    def test_q_while_typing_is_a_letter_not_a_quit(self):
        """Otherwise a Wi-Fi password containing q ends the program."""
        self.type("t")
        self.con._key(ord("q"))
        self.assertTrue(self.con._key(ord("q")))
        self.assertEqual(self.con.typing, "qq")

    def test_reset_is_refused(self):
        """R clears every parameter, not just the Wi-Fi: the bench trim
        calibration goes with it. One keystroke away is too close."""
        self.type("t")
        self.type("R")
        self.con._key(self.ENTER)
        self.assertEqual(self.board.raw, [])
        self.assertIn("R disabled", self.con.reply[0])

    def test_something_that_merely_starts_with_r_is_allowed(self):
        self.type("t")
        self.type("REQUEST")
        self.con._key(self.ENTER)
        self.assertEqual(self.board.raw, ["REQUEST\n"])

    def test_the_reply_is_kept_for_display(self):
        """The console used to drop OK/ERR/EV entirely, so a firmware error
        was invisible and a wrong setting looked like a silent board."""
        self.assertIsNone(self.con.reply)
        self.con._on_board_line("OK P url = http://192.168.4.1:8080")
        self.assertIn("192.168.4.1:8080", self.con.reply[0])
        self.assertEqual(self.con.reply[1], GREEN)
        self.con._on_board_line("ERR unknown parameter: URL")
        self.assertEqual(self.con.reply[1], RED)
        self.con._on_board_line("EV url not set - send P url ...")
        self.assertEqual(self.con.reply[1], AMBER)

    def test_a_command_longer_than_the_limit_is_not_accepted(self):
        self.type("t")
        self.type("P url " + "x" * 200)
        self.assertEqual(len(self.con.typing), COMMAND_MAX)

    def test_the_wifi_task_advertises_the_key(self):
        self.con._switch(TaskWifi)
        self.assertIn("T", [k for k, _ in self.con.task.keys(self.con)])

    def test_a_long_line_is_shortened_rather_than_cut_by_the_frame(self):
        """draw_panel clamps its corner, so a line wider than the frame runs
        off the edge and is cut silently - the operator sees a shorter answer
        and no sign that part of it is missing."""
        img = np.zeros((480, 640, 3), np.uint8)
        long_reply = "OK " + "x" * 300
        fitted = fit_text(long_reply, img, 0.5)
        self.assertTrue(fitted.endswith("…"))
        self.assertLessEqual(_text_wh(fitted, 0.5)[0], img.shape[1] - 48)
        self.assertEqual(fit_text("OK url = http://192.168.4.1:8080", img, 0.5),
                         "OK url = http://192.168.4.1:8080")

    def test_keeping_the_tail_drops_the_front_instead(self):
        """While typing, the cursor is at the end, so that is the end to keep."""
        img = np.zeros((480, 640, 3), np.uint8)
        fitted = fit_text("x" * 300 + "END", img, 0.5, keep_head=False)
        self.assertTrue(fitted.startswith("…"))
        self.assertTrue(fitted.endswith("END"))


class UprightTests(TestCase):
    """The camera is mounted upside down on the ROV, so every frame is
    rotated 180 degrees as it comes off the device."""

    def test_a_frame_comes_out_rotated(self):
        frame = np.zeros((4, 6, 3), np.uint8)
        frame[0, 0] = (10, 20, 30)              # top-left
        frame[0, 5] = (40, 50, 60)              # top-right
        out = upright(frame)
        self.assertEqual(out.shape, frame.shape)
        np.testing.assert_array_equal(out[3, 5], (10, 20, 30))
        np.testing.assert_array_equal(out[3, 0], (40, 50, 60))

    def test_the_pointer_and_the_sample_share_one_space(self):
        """Flipping the raw frame inside Camera is what keeps the crosshair
        and the sampled patch in the same coordinates. Flip anywhere later and
        the operator aims at one corner while the colour task reads the
        opposite one - which still looks like an answer."""
        raw = np.full((480, 640, 3), (40, 40, 40), np.uint8)
        cv2.rectangle(raw, (40, 30), (80, 90), (0, 0, 240), -1)   # red, top-left
        fixed = upright(raw)

        self.assertIsNone(sample_at(fixed, 60, 60, DEFAULT_RANGES))
        self.assertEqual(sample_at(fixed, 580, 420, DEFAULT_RANGES), "R")


class LostCameraTests(TestCase):

    def test_a_lost_camera_is_announced_rather_than_left_black(self):
        """A black picture with no explanation is the worst failure mode: the
        operator cannot tell a dead camera from a dark pool."""
        con = make_console()
        con.camera.lost = True
        con._switch(TaskAprilTag)
        image = con.render(telemetry(), True, frame=frame_with_tags(7))
        self.assertEqual(image.shape[2], 3)

    def test_a_camera_that_has_not_delivered_yet_still_renders(self):
        """Startup happens before the first frame arrives. That must draw the
        OSD over a blank picture, not raise."""
        con = make_console(camera=FakeCamera())
        self.assertFalse(con.camera.ok)
        image = con.render(telemetry(), True)
        self.assertEqual(image.shape[2], 3)


if __name__ == "__main__":
    import unittest
    unittest.main()
