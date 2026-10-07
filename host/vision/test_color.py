"""Colour recognition tests: no camera, no window, synthetic frames only.

Run from host/:   python3 -m unittest vision.test_color -v

Ported from the standalone color_recognize.py test suite, minus its CLI case
(the detector no longer has a command line - the console drives it).
"""

from unittest import TestCase
from unittest.mock import patch

import cv2
import numpy as np

from vision.color import Burst, DEFAULT_RANGES, Settings, calibrate, detect


def scene(color="R", tilt=0, floor=True):
    """A pole on a blue-ish floor, at the size and position the detector expects."""
    image = np.full((480, 640, 3), (40, 40, 40), np.uint8)
    if floor:
        cv2.rectangle(image, (0, 380), (639, 479), (255, 0, 0), -1)
    bgr = {"R": (0, 0, 240), "Y": (0, 220, 240), "B": (240, 70, 0)}[color]
    polygon = cv2.boxPoints(((320, 225), (24, 240), tilt)).astype(np.int32)
    cv2.fillConvexPoly(image, polygon, bgr)
    return image


class RecognitionTests(TestCase):

    def test_poles_with_blue_floor_and_tilt(self):
        for color in "RYB":
            for tilt in (0, 25, -25):
                with self.subTest(color=color, tilt=tilt):
                    result, _ = detect(scene(color, tilt), DEFAULT_RANGES, Settings())
                    self.assertEqual([c["color"] for c in result], [color])

    def test_floor_horizontal_stripe_and_large_background(self):
        for color in ((255, 0, 0), (0, 0, 255), (0, 255, 255)):
            image = np.zeros((480, 640, 3), np.uint8)
            cv2.rectangle(image, (0, 320), (639, 479), color, -1)
            cv2.rectangle(image, (70, 100), (500, 120), color, -1)
            self.assertEqual(detect(image, DEFAULT_RANGES, Settings())[0], [])
            image[:] = color
            self.assertEqual(detect(image, DEFAULT_RANGES, Settings())[0], [])

    def test_white_ball_optional(self):
        """The ball raises the score but must never be required."""
        image = scene()
        without, _ = detect(image, DEFAULT_RANGES, Settings())
        cv2.circle(image, (320, 88), 16, (255, 255, 255), -1)
        with_ball, _ = detect(image, DEFAULT_RANGES, Settings())
        self.assertFalse(without[0]["ball_hint"])
        self.assertTrue(with_ball[0]["ball_hint"])
        self.assertGreater(with_ball[0]["shape_score"], without[0]["shape_score"])

    def test_wide_base_and_overlapping_calibration(self):
        image = scene(floor=False)
        cv2.rectangle(image, (270, 345), (370, 363), (0, 0, 240), -1)
        self.assertEqual([c["color"] for c in detect(image, DEFAULT_RANGES, Settings())[0]], ["R"])
        # Two colours calibrated to the same range: refuse to guess rather
        # than reporting the object twice.
        ranges = {**DEFAULT_RANGES, "Y": DEFAULT_RANGES["R"]}
        self.assertEqual(detect(image, ranges, Settings())[0], [])

    def test_roi_excludes_target(self):
        self.assertEqual(detect(scene(), DEFAULT_RANGES, Settings(), (0, 0, 200, 300))[0], [])

    def test_idle_and_finished_never_detect(self):
        burst = Burst(Settings())
        with patch("vision.color.detect", side_effect=AssertionError("idle detection")):
            self.assertIsNone(burst.update(scene(), DEFAULT_RANGES, None, 0))
        burst.start(0)
        for i in range(11):
            burst.update(scene(), DEFAULT_RANGES, None, i / 10)
        self.assertEqual(burst.result["status"], "confirmed")
        self.assertEqual(burst.result["targets"][0]["color"], "R")
        with patch("vision.color.detect", side_effect=AssertionError("finished detection")):
            self.assertIsNone(burst.update(scene(), DEFAULT_RANGES, None, 2))

    def test_brief_hits_or_missing_final_frame_not_confirmed(self):
        blank = np.zeros((480, 640, 3), np.uint8)
        for count in (2, 8):
            burst = Burst(Settings())
            burst.start(0)
            for i in range(11):
                burst.update(scene() if i < count else blank, DEFAULT_RANGES, None, i / 10)
            self.assertEqual(burst.result["status"], "unconfirmed")

    def test_same_color_at_different_positions_cannot_accumulate(self):
        burst = Burst(Settings())
        burst.start(0)
        for i in range(11):
            image = np.roll(scene(floor=False), 180 if i % 2 else -180, axis=1)
            burst.update(image, DEFAULT_RANGES, None, i / 10)
        self.assertEqual(burst.result["status"], "unconfirmed")

    def test_red_calibration_wraps_and_mixed_sample_is_rejected(self):
        """Red straddles the 0/179 seam and needs two bands.

        A colour-mixed sample must be refused outright. The original returned
        bands built from it anyway and printed a warning that the caller
        ignored, so a bad calibration got saved silently.
        """
        hsv = np.zeros((30, 30, 3), np.uint8)
        hsv[:, :15] = (178, 220, 200)
        hsv[:, 15:] = (2, 220, 200)
        bands = calibrate(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), (0, 0, 30, 30))
        self.assertEqual(len(bands), 2)
        self.assertTrue(any(a == 0 for a, *_ in bands))

        hsv[:, 15:] = (90, 220, 200)          # now half red, half cyan
        self.assertIsNone(calibrate(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), (0, 0, 30, 30)))

    def test_calibrate_rejects_too_few_pixels(self):
        flat = np.full((30, 30, 3), (40, 40, 40), np.uint8)
        self.assertIsNone(calibrate(flat, (0, 0, 30, 30)))


if __name__ == "__main__":
    import unittest
    unittest.main()
