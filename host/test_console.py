"""Console task behaviour: no camera, no board, no window.

Run from host/:   python3 -m unittest test_console -v

These pin the interaction: entering a task only previews, and recognition
happens on RB. That is easy to break by accident when adding a task.
"""

import time
from unittest import TestCase

import cv2

from console import (BTN_APRILTAG, BTN_CAPTURE, Console, Task, TaskAprilTag,
                     TaskColor, Camera, Pad)
from pad_bridge import DEFAULT_DEADMAN_BUTTON, Telemetry
from vision.test_apriltag import frame_with_tags
from vision.test_color import scene as colour_scene


def telemetry():
    t = Telemetry()
    t.l_us, t.r_us = 1718, 1282
    t.en = t.link = True
    return t


def make_console():
    pad = Pad(0, 1, DEFAULT_DEADMAN_BUTTON, False, False)
    pad.js = object()          # stands in for a connected controller
    return Console(pad=pad, board=None, camera=Camera(synthetic=True),
                   use_deadman=True)


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
                         ["C", "1 2 3", "N", "R", "O", "F", "D"])
        # Manual declares nothing of its own; Q and S are added globally.
        self.assertEqual(Task().keys(self.con), [])


if __name__ == "__main__":
    import unittest
    unittest.main()
