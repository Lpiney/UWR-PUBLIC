"""Console task behaviour: no camera, no board, no window.

Run from host/:   python3 -m unittest test_console -v

These pin the interaction: entering a task only previews, and recognition
happens on RB. That is easy to break by accident when adding a task.
"""

import time
from unittest import TestCase

import cv2

from console import (BTN_APRILTAG, BTN_CAPTURE, Console, TaskAprilTag,
                     TaskColor, Camera, Pad)
from pad_bridge import DEFAULT_DEADMAN_BUTTON, Telemetry
from vision.test_apriltag import frame_with_tags


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
        self.assertIsNone(self.con.task.frozen)

    def test_entering_the_colour_task_does_not_start_a_burst(self):
        self.con._switch(TaskColor)
        self.pump(0.15, frame=self.tag_frame)
        self.assertFalse(self.con.color.burst.active)
        self.assertIsNone(self.con.color.burst.result)

    # ---- Capture ----

    def test_capture_finds_a_tag_and_freezes_the_frame(self):
        self.con._switch(TaskAprilTag)
        self.assertIs(self.con.task.capture(self.con), True)
        self.pump(0.6)
        self.assertFalse(self.con.task.capturing)
        self.assertEqual(self.con.apriltag.target_id, 7)
        self.assertIsNotNone(self.con.task.frozen)

    def test_a_second_capture_replaces_the_first_result(self):
        self.con._switch(TaskAprilTag)
        self.con.task.capture(self.con)
        self.pump(0.6)
        first = self.con.task.frozen
        self.con.task.capture(self.con)
        self.pump(0.6)
        self.assertEqual(self.con.apriltag.target_id, 7)
        self.assertIsNot(self.con.task.frozen, first)

    def test_capture_with_nothing_in_view_still_freezes(self):
        """The operator gets a picture of what was there, not a hang."""
        self.con._switch(TaskAprilTag)
        self.con.task.capture(self.con)
        self.pump(0.6, frame=self.tag_frame * 0 + 128)
        self.assertIsNone(self.con.apriltag.target_id)
        self.assertIsNotNone(self.con.task.frozen)

    def test_leaving_the_task_clears_the_result(self):
        self.con._switch(TaskAprilTag)
        self.con.task.capture(self.con)
        self.pump(0.6)
        self.assertIsNotNone(self.con.task.frozen)
        self.con._switch(TaskAprilTag)      # back to Manual
        self.con._switch(TaskAprilTag)      # in again
        self.assertIsNone(self.con.task.frozen)
        self.assertIsNone(self.con.apriltag.target_id)

    # ---- Button handling ----

    def test_rb_edge_triggers_a_capture(self):
        self.con._switch(TaskAprilTag)
        held = [0] * 10
        held[BTN_CAPTURE] = 1
        self.con._handle_buttons(held, [0] * 10)
        self.assertTrue(self.con.task.capturing)

    def test_holding_rb_does_not_retrigger(self):
        self.con._switch(TaskAprilTag)
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


if __name__ == "__main__":
    import unittest
    unittest.main()
