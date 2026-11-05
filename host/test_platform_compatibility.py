"""Cross-platform contracts; real Windows execution also runs in Actions."""
import os
from pathlib import Path
import runpy
import sys
import tempfile
import types
from unittest import TestCase
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

from console import camera_backends
from pad_bridge import Pad, PadState
from test_console import make_console
from test_pad_bridge import FakeJoystick
from vision.image_io import write_png
from vision.lab_experiment import LabExperiment, record

HOST = Path(__file__).resolve().parent


class PlatformCompatibilityTests(TestCase):
    def test_both_platforms_keep_their_controller_mapping(self):
        for platform, y, x, rb, lt, rest in [('win32', 3, 2, 5, 4, -1.),
                                              ('darwin', 7, 6, 15, 2, 0.)]:
            with self.subTest(platform=platform), patch.object(sys, 'platform', platform):
                bridge = runpy.run_path(str(HOST / 'pad_bridge.py'))
                module = types.ModuleType('pad_bridge')
                module.__dict__.update(bridge)
                with patch.dict(sys.modules, {'pad_bridge': module}):
                    console = runpy.run_path(str(HOST / 'console.py'))
                self.assertEqual(console['BTN_CAPTURE'], rb)
                self.assertEqual(console['BTN_WIFI'], x)
                self.assertEqual(console['DEFAULT_DEADMAN_BUTTON'], y)
                axes = [0.] * 6
                axes[lt] = axes[5] = rest
                pad = bridge['Pad'](0, 1, y, False, False)
                pad.js = FakeJoystick(buttons=(y,))
                pad.js.axes = axes
                self.assertTrue(pad.read().deadman)
                self.assertEqual(pad.read().vz, 0.)
                axes[5] = 1.
                self.assertEqual(pad.read().vz, 1.)
                axes[lt] = 1.
                self.assertEqual(pad.read().vz, 0.)
                axes[5] = rest
                self.assertEqual(pad.read().vz, -1.)

    def test_windows_camera_prefers_directshow(self):
        with patch.object(sys, 'platform', 'win32'):
            self.assertEqual(camera_backends(), (cv2.CAP_DSHOW, cv2.CAP_ANY))

    def test_controller_input_enabled_when_camera_window_has_focus(self):
        pygame = MagicMock()
        pygame.joystick.get_count.return_value = 1
        with patch.dict(sys.modules, {'pygame': pygame}), patch.dict(os.environ, {}, clear=True):
            pad = Pad(0, 1, 3, False, False)
            self.assertTrue(pad.open())
            self.assertEqual(os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'], '1')

    def test_unicode_and_spaces_in_calibration_and_image_paths(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / '颜色 测试'
            folder.mkdir()
            frame = np.full((20, 20, 3), (0, 0, 230), np.uint8)
            path = folder / '截图.png'
            write_png(path, frame)
            decoded = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
            np.testing.assert_array_equal(decoded, frame)
            model = LabExperiment(folder / '标定.json')
            model.add('R', frame, 10, 10, 3)
            self.assertEqual(LabExperiment(model.path).samples, model.samples)
            image = record(frame, {'answer': None, 'reason': 'test rejection'}, folder)
            self.assertTrue(image.exists())
            self.assertTrue((folder / 'events.jsonl').exists())

    def test_serial_read_disconnect_locks_and_exits_without_traceback(self):
        con = make_console()
        con.latch.locked = False
        neutral = PadState(0., 0., 0., False, [], [])
        with patch.object(con.pad, 'pump'), patch.object(con.pad, 'read', return_value=neutral), \
             patch.object(con.board, 'poll', side_effect=OSError(6, 'Device not configured')), \
             patch('console.cv2.namedWindow'), patch('console.cv2.resizeWindow'), \
             patch('console.cv2.setMouseCallback'), patch('console.cv2.destroyAllWindows'), \
             patch.object(con.camera, 'release') as release:
            self.assertEqual(con.run(), 2)
        self.assertTrue(con.latch.locked)
        release.assert_called_once()
