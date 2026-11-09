"""Check requested capture profiles without opening a physical camera."""
import sys
import time
from unittest import TestCase
from unittest.mock import MagicMock, patch

import cv2

from console import Camera, CAM_WIDTH, CAM_HEIGHT, CAM_FPS


class CameraProfileTests(TestCase):
    def test_default_profile_requests_720p30_on_both_platforms(self):
        self.assertEqual((CAM_WIDTH, CAM_HEIGHT, CAM_FPS), (1280, 720, 30))
        for platform in ('win32', 'darwin'):
            with self.subTest(platform=platform):
                camera = Camera.__new__(Camera)
                camera.index, camera.width, camera.height = 0, CAM_WIDTH, CAM_HEIGHT
                camera.requested_fps = CAM_FPS
                camera.backend, camera.exposure = "default", None
                cap = MagicMock()
                cap.isOpened.return_value = True
                cap.get.return_value = 30.0
                with patch.object(sys, 'platform', platform), patch('console.cv2.VideoCapture', return_value=cap):
                    self.assertTrue(camera._open())
                for prop, value in [(cv2.CAP_PROP_FRAME_WIDTH, 1280),
                                    (cv2.CAP_PROP_FRAME_HEIGHT, 720),
                                    (cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_BUFFERSIZE, 1)]:
                    cap.set.assert_any_call(prop, value)
                self.assertFalse(any(c.args[0] == cv2.CAP_PROP_GAIN for c in cap.set.call_args_list))
                if platform == 'win32':
                    cap.set.assert_any_call(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    def test_stale_camera_does_not_report_fresh_video_fps(self):
        camera = Camera.__new__(Camera)
        camera._capture_fps = 30.0
        camera.last_frame_at = time.monotonic() - 5
        self.assertEqual(camera.capture_fps, 0.0)
        camera.last_frame_at = time.monotonic()
        self.assertEqual(camera.capture_fps, 30.0)

    def test_fast_profile_and_explicit_backend(self):
        camera = Camera.__new__(Camera)
        camera.index, camera.width, camera.height, camera.requested_fps = 0, 640, 480, 30
        camera.backend, camera.exposure = "msmf", None
        cap = MagicMock()
        cap.isOpened.return_value = True
        cap.get.return_value = 30.0
        with patch.object(sys, 'platform', 'win32'), patch('console.cv2.VideoCapture', return_value=cap) as capture:
            self.assertTrue(camera._open())
        capture.assert_called_once_with(0, cv2.CAP_MSMF)
        cap.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        calls = cap.set.call_args_list
        last_codec = max(i for i, c in enumerate(calls) if c.args[0] == cv2.CAP_PROP_FOURCC)
        last_size = max(i for i, c in enumerate(calls) if c.args[0] in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))
        self.assertGreater(last_codec, last_size)

    def test_render_limit_does_not_limit_command_scheduling(self):
        from test_console import make_console
        from pad_bridge import PadState
        camera_clock = [0.0]
        con = make_console()
        neutral = PadState(0., 0., 0., False, [], [])

        def key(_):
            camera_clock[0] += 0.005
            return ord('q') if camera_clock[0] >= 0.5 else 255

        with patch('console.time.monotonic', side_effect=lambda: camera_clock[0]), \
             patch.object(con.pad, 'pump'), patch.object(con.pad, 'read', return_value=neutral), \
             patch.object(con, 'render') as render, patch.object(con, '_save_evidence'), \
             patch('console.cv2.namedWindow'), patch('console.cv2.resizeWindow'), \
             patch('console.cv2.setMouseCallback'), patch('console.cv2.destroyAllWindows'), \
             patch('console.cv2.imshow'), patch('console.cv2.waitKey', side_effect=key):
            self.assertEqual(con.run(), 0)
        self.assertGreater(len(con.board.sent), render.call_count)
        self.assertLessEqual(render.call_count, 16)
        self.assertGreaterEqual(len(con.board.sent), 20)
