import json
import tempfile
from pathlib import Path
from unittest import TestCase

import numpy as np

from vision.lab_experiment import LabExperiment, record
from vision.color import DEFAULT_RANGES, inspect_hsv


def solid(bgr):
    return np.full((40, 40, 3), bgr, dtype=np.uint8)


class LabExperimentTests(TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)
        self.model = LabExperiment(self.path / 'lab.json')
        self.frames = {'R': solid((0, 0, 230)), 'Y': solid((0, 220, 230)),
                       'B': solid((230, 50, 0))}

    def train(self):
        for c, frame in self.frames.items():
            self.model.add(c, frame, 20, 20, 7)

    def test_requires_all_classes_and_replaces_only_selected_colour(self):
        self.assertIsNone(self.model.classify(self.frames['B'], 20, 20, 7)['answer'])
        self.train()
        replacement = solid((180, 60, 10))
        self.model.add('B', replacement, 20, 20, 7)
        loaded = LabExperiment(self.model.path)
        self.assertEqual(len(loaded.samples['B']), 1)
        self.assertEqual(loaded.samples, self.model.samples)
        for c, frame in dict(self.frames, B=replacement).items():
            self.assertEqual(loaded.classify(frame, 20, 20, 7)['answer'], c)

    def test_legacy_config_uses_only_latest_sample_per_colour(self):
        self.model.path.write_text(json.dumps({c: [[40, 10, 20], [50, 30, 40]] for c in 'RYB'}))
        loaded = LabExperiment(self.model.path)
        self.assertEqual(loaded.samples, {c: [[50, 30, 40]] for c in 'RYB'})

    def test_rejects_grey_and_unseen_green(self):
        self.train()
        for frame in [solid((120, 120, 120)), solid((0, 230, 0))]:
            self.assertIsNone(self.model.classify(frame, 20, 20, 7)['answer'])

    def test_ambiguous_calibration_is_not_forced_to_a_class(self):
        for c in 'RYB':
            self.model.add(c, self.frames['B'], 20, 20, 7)
        report = self.model.classify(self.frames['B'], 20, 20, 7)
        self.assertIsNone(report['answer'])
        self.assertIn('ambiguous', report['reason'])

    def test_small_box_can_sample_thin_pole(self):
        self.train()
        frame = solid((90, 90, 90))
        frame[:, 18:23] = (230, 50, 0)
        self.assertIsNone(self.model.classify(frame, 20, 20, 7)['answer'])
        self.assertEqual(self.model.classify(frame, 20, 20, 2)['answer'], 'B')

    def test_malformed_config_recovers_with_message(self):
        self.model.path.write_text('{broken')
        loaded = LabExperiment(self.model.path)
        self.assertTrue(loaded.load_error)
        self.assertFalse(any(loaded.samples.values()))

    def test_rejected_capture_records_image_and_reason(self):
        report = self.model.classify(self.frames['B'], 20, 20, 7)
        image = record(self.frames['B'], report, self.path / 'logs')
        self.assertTrue(image.exists())
        row = json.loads((image.parent / 'events.jsonl').read_text())
        self.assertIsNone(row['answer'])
        self.assertEqual(row['reason'], report['reason'])
        self.assertEqual(row['image'], image.name)

    def test_hsv_reports_gate_that_rejected_sample(self):
        report = inspect_hsv(solid((100, 100, 100)), 20, 20, DEFAULT_RANGES)
        self.assertEqual(report['reason'], 'too few saturated pixels')
