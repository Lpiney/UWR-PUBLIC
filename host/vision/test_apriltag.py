"""AprilTag tests: tags are generated in-process, so no camera or files.

Run from host/:   python3 -m unittest vision.test_apriltag -v
"""

from unittest import TestCase

import cv2
import numpy as np

from vision.apriltag import FAMILIES, AprilTagScanner, PreprocessConfig


def frame_with_tags(*ids, size=(640, 480), tag_px=100, quiet=40, margin=0.20):
    """Render a white frame with the given 36h11 tags laid out in a row.

    Spacing is computed so the tags never touch: overlapping markers corrupt
    each other's quiet zone and only the last one drawn stays readable.
    """
    width, height = size
    image = np.full((height, width, 3), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES["36h11"])

    box = tag_px + 2 * quiet
    if len(ids) * box > width:
        raise ValueError(f"{len(ids)} tags of {box}px do not fit in {width}px")
    gap = (width - len(ids) * box) / (len(ids) + 1)

    for i, tag_id in enumerate(ids):
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_px)
        marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        # aruco needs a quiet zone, which the marker generator does not add.
        marker = cv2.copyMakeBorder(marker, quiet, quiet, quiet, quiet,
                                    cv2.BORDER_CONSTANT, value=(255, 255, 255))
        x = int(gap * (i + 1) + box * i)
        y = int(height * margin)
        image[y:y + box, x:x + box] = marker
    return image


class AprilTagTests(TestCase):

    def test_detects_a_single_tag(self):
        scanner = AprilTagScanner()
        found = scanner.detect(frame_with_tags(7))
        self.assertEqual(list(found), [7])

    def test_confirmation_needs_min_hits_frames(self):
        """One frame must not be enough; a tag blurred by ripples would pass."""
        scanner = AprilTagScanner(min_hits=3)
        frame = frame_with_tags(7)
        for _ in range(2):
            scanner.detect(frame)
        self.assertEqual(scanner.confirmed_ids, [])
        self.assertIsNone(scanner.target_id)
        scanner.detect(frame)
        self.assertEqual(scanner.confirmed_ids, [7])
        self.assertEqual(scanner.target_id, 7)

    def test_target_is_the_largest_or_smallest_confirmed_id(self):
        frame = frame_with_tags(3, 11, 5)
        for mode, expected in (("largest", 11), ("smallest", 3)):
            with self.subTest(mode=mode):
                scanner = AprilTagScanner(mode=mode, min_hits=1)
                scanner.detect(frame)
                self.assertEqual(scanner.target_id, expected)

    def test_an_unconfirmed_tag_is_not_a_candidate(self):
        """min_hits=3 with two tags: the low-ID one seen once must not win."""
        scanner = AprilTagScanner(mode="smallest", min_hits=3)
        scanner.detect(frame_with_tags(4, 9))
        scanner.detect(frame_with_tags(9))
        scanner.detect(frame_with_tags(9))
        self.assertNotIn(4, scanner.confirmed_ids)
        self.assertEqual(scanner.target_id, 9)

    def test_result_reports_progress(self):
        scanner = AprilTagScanner(expected=3, min_hits=1)
        scanner.detect(frame_with_tags(2, 8))
        result = scanner.result()
        self.assertEqual(result["target"], 8)
        self.assertEqual(result["scanned"], 2)
        self.assertFalse(result["complete"])
        self.assertEqual(result["expected"], 3)

    def test_reset_clears_the_accumulation(self):
        scanner = AprilTagScanner(min_hits=2)
        frame = frame_with_tags(7)
        scanner.detect(frame)
        scanner.detect(frame)
        self.assertEqual(scanner.target_id, 7)
        scanner.reset()
        self.assertEqual(scanner.confirmed_ids, [])
        self.assertIsNone(scanner.target_id)

    def test_annotate_draws_without_error(self):
        scanner = AprilTagScanner(min_hits=1)
        frame = frame_with_tags(7)
        scanner.detect(frame)
        out = scanner.annotate(frame.copy())
        self.assertEqual(out.shape, frame.shape)

    def test_uniform_frame_produces_nothing(self):
        """A blank pool floor must not yield a phantom tag."""
        scanner = AprilTagScanner()
        self.assertEqual(scanner.detect(np.full((480, 640, 3), 128, np.uint8)), {})

    def test_unknown_family_is_rejected(self):
        with self.assertRaises(ValueError):
            AprilTagScanner(families=("nonsense",))

    def test_preprocessing_is_off_by_default(self):
        # Measured against degraded frames, none of the options beat doing
        # nothing, so enabling one has to be a deliberate act.
        config = PreprocessConfig()
        self.assertEqual(config.denoise, "off")
        self.assertEqual(config.gamma, 1.0)
        self.assertFalse(config.clahe)
        gray = np.full((32, 32), 100, np.uint8)
        self.assertTrue(np.array_equal(config.apply(gray), gray))


if __name__ == "__main__":
    import unittest
    unittest.main()
