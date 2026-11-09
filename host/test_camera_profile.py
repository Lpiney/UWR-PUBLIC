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
                cap = MagicMock()
                cap.isOpened.return_value = True
                cap.get.return_value = 30.0
                with patch.object(sys, 'platform', platform), patch('console.cv2.VideoCapture', return_value=cap):
                    self.assertTrue(camera._open())
                for prop, value in [(cv2.CAP_PROP_FRAME_WIDTH, 1280),
                                    (cv2.CAP_PROP_FRAME_HEIGHT, 720),
                                    (cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_BUFFERSIZE, 1)]:
                    cap.set.assert_any_call(prop, value)
                if platform == 'win32':
                    cap.set.assert_any_call(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    def test_stale_camera_does_not_report_fresh_video_fps(self):
        camera = Camera.__new__(Camera)
        camera._capture_fps = 30.0
        camera.last_frame_at = time.monotonic() - 5
        self.assertEqual(camera.capture_fps, 0.0)
        camera.last_frame_at = time.monotonic()
        self.assertEqual(camera.capture_fps, 30.0)
