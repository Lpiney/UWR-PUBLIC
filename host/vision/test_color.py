"""Colour recognition tests: no camera, no window, synthetic frames only.

Run from host/:   python3 -m unittest vision.test_color -v
"""

from unittest import TestCase

import cv2
import numpy as np

from vision.color import DEFAULT_RANGES, calibrate, sample_at

# Where the pole is drawn in scene(): centred at (320, 225), 24 px wide.
POLE = (320, 225)


def scene(color="R", tilt=0, floor=True):
    """A pole on a blue-ish floor, in the pose the tests point at."""
    image = np.full((480, 640, 3), (40, 40, 40), np.uint8)
    if floor:
        cv2.rectangle(image, (0, 380), (639, 479), (255, 0, 0), -1)
    bgr = {"R": (0, 0, 240), "Y": (0, 220, 240), "B": (240, 70, 0)}[color]
    polygon = cv2.boxPoints(((320, 225), (24, 240), tilt)).astype(np.int32)
    cv2.fillConvexPoly(image, polygon, bgr)
    return image


class SampleTests(TestCase):

    def test_each_pole_reports_its_own_colour(self):
        for color in "RYB":
            with self.subTest(color=color):
                self.assertEqual(
                    sample_at(scene(color), *POLE, DEFAULT_RANGES), color)

    def test_a_tilted_pole_is_read_at_its_centre(self):
        for tilt in (25, -25):
            with self.subTest(tilt=tilt):
                self.assertEqual(
                    sample_at(scene("R", tilt), *POLE, DEFAULT_RANGES), "R")

    def test_plain_background_gives_no_answer(self):
        """Grey is not a colour, whatever band its noise lands nearest."""
        self.assertIsNone(sample_at(scene(), 50, 100, DEFAULT_RANGES))

    def test_the_seam_between_two_poles_gives_no_answer(self):
        """Half red and half yellow is an edge, not an answer."""
        image = np.full((480, 640, 3), (40, 40, 40), np.uint8)
        image[:, :320] = (0, 0, 240)
        image[:, 320:] = (0, 220, 240)
        self.assertIsNone(sample_at(image, 320, 240, DEFAULT_RANGES))

    def test_pointing_off_the_frame_gives_no_answer(self):
        for x, y in ((-50, 100), (10000, 100), (100, -50), (100, 10000)):
            with self.subTest(pos=(x, y)):
                self.assertIsNone(sample_at(scene(), x, y, DEFAULT_RANGES))

    def test_a_shadow_over_the_pole_gives_no_answer(self):
        """A dark patch has no hue to report, and must not inherit one."""
        image = scene()
        image[200:250, 300:340] = (10, 10, 10)
        self.assertIsNone(sample_at(image, *POLE, DEFAULT_RANGES))

    def test_blue_paint_is_reported_as_blue(self):
        """Documents the trade-off of sampling rather than detecting: aiming
        at blue pool paint gets BLUE. The operator picks the target, so the
        answer is only as good as the aim."""
        self.assertEqual(sample_at(scene(), 100, 450, DEFAULT_RANGES), "B")


class CalibrationTests(TestCase):

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
        self.assertIsNone(calibrate(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR),
                                    (0, 0, 30, 30)))

    def test_calibrate_rejects_too_few_pixels(self):
        flat = np.full((30, 30, 3), (40, 40, 40), np.uint8)
        self.assertIsNone(calibrate(flat, (0, 0, 30, 30)))

    def test_bands_from_a_pole_are_usable_straight_away(self):
        """Round trip: calibrate from a sample, then sample with the result."""
        frame = scene("Y")
        bands = calibrate(frame, (300, 200, 40, 50))
        self.assertIsNotNone(bands)
        self.assertEqual(sample_at(frame, *POLE, dict(DEFAULT_RANGES, Y=bands)),
                         "Y")

    def test_calibration_survives_a_round_trip_through_the_config(self):
        import json
        import pathlib
        import tempfile

        from vision.color import load_config, save_config

        frame = scene("B")
        bands = calibrate(frame, (300, 200, 40, 50))
        path = pathlib.Path(tempfile.mkdtemp()) / "local-config.json"
        save_config(dict(DEFAULT_RANGES, B=bands), path)
        reloaded = load_config(path)
        self.assertEqual(sample_at(frame, *POLE, reloaded), "B")

    def test_an_old_config_shape_is_still_read(self):
        """The file used to hold {"settings": ..., "ranges": ...}."""
        import json
        import pathlib
        import tempfile

        from vision.color import load_config

        path = pathlib.Path(tempfile.mkdtemp()) / "old.json"
        path.write_text(json.dumps({"settings": {"seconds": 1.0},
                                    "ranges": DEFAULT_RANGES}), encoding="utf-8")
        self.assertEqual(load_config(path), DEFAULT_RANGES)


if __name__ == "__main__":
    import unittest
    unittest.main()
