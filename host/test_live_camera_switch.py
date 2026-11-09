"""Live camera changes must never recreate the serial link or controller."""
import threading
import time
from unittest import TestCase
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

from console import Camera, TaskColor, CAMERA_BAR_HEIGHT
from test_console import make_console, telemetry


def camera():
    cam = Camera.__new__(Camera)
    cam.index, cam.width, cam.height, cam.requested_fps = 2, 1280, 720, 30
    cam._profile_lock = threading.Lock()
    cam._pending_profile = None
    cam.switching, cam.status, cam.lost = False, 'Actual 1280x720', False
    cam._cap = MagicMock()
    cam._frame = np.zeros((720, 1280, 3), np.uint8)
    cam.last_frame_at = time.monotonic()
    cam._capture_fps, cam._capture_count = 30., 0
    cam._capture_since, cam._misses, cam._next_try = time.monotonic(), 0, 0.
    cam._stop, cam._thread = threading.Event(), None
    return cam


class LiveCameraSwitchTests(TestCase):
    def test_request_only_queues_work_and_hides_stale_frame(self):
        cam = camera()
        old = cam._cap
        with patch.object(cam, '_open') as opened:
            self.assertTrue(cam.request_resolution(640, 480))
            old.release.assert_not_called()
            opened.assert_not_called()
            self.assertIsNone(cam.read())
        self.assertTrue(cam.switching)

    def test_worker_reopens_same_device_and_publishes_new_size(self):
        cam = camera()
        old, new = cam._cap, MagicMock()
        entered, unblock = threading.Event(), threading.Event()
        owners = []

        def old_read():
            entered.set()
            unblock.wait(2)
            return True, np.zeros((720, 1280, 3), np.uint8)

        def opened():
            owners.append(threading.get_ident())
            self.assertEqual((cam.index, cam.width, cam.height), (2, 640, 480))
            cam._cap = new
            return True

        def new_read():
            cam._stop.set()
            return True, np.zeros((480, 640, 3), np.uint8)

        old.read.side_effect = old_read
        new.read.side_effect = new_read
        with patch.object(cam, '_open', side_effect=opened):
            worker = threading.Thread(target=cam._run, daemon=True)
            worker.start()
            try:
                self.assertTrue(entered.wait(2))
                cam.request_resolution(640, 480)
                old.release.assert_not_called()
            finally:
                unblock.set()
                worker.join(3)
                cam._stop.set()
        self.assertFalse(worker.is_alive())
        self.assertNotEqual(owners, [threading.get_ident()])
        old.release.assert_called_once()
        self.assertFalse(cam.switching)
        self.assertEqual(cam.read().shape, (480, 640, 3))
        self.assertIn('Actual 640x480', cam.status)

    def test_open_failure_restores_previous_profile_for_retry(self):
        cam = camera()
        cam.request_resolution(640, 480)
        with patch.object(cam, '_open', return_value=False):
            cam._apply_resolution_request()
        self.assertEqual((cam.width, cam.height), (1280, 720))
        self.assertTrue(cam.switching)
        self.assertIn('restoring', cam.status)

    def test_toolbar_keeps_board_and_pad_and_resets_pixel_target(self):
        con = make_console(camera=camera())
        board, pad = con.board, con.pad
        con._switch(TaskColor)
        con.render(telemetry(), False)
        con.mouse.x, con.mouse.y = 100, 100
        con.task.pending_color = 'B'
        con.board.close = MagicMock()
        con.pad.open = MagicMock()
        con._mouse_event(cv2.EVENT_LBUTTONDOWN, 110, 20, 0, None)
        self.assertIs(con.board, board)
        self.assertIs(con.pad, pad)
        con.board.close.assert_not_called()
        con.pad.open.assert_not_called()
        self.assertTrue(con.camera.switching)
        self.assertFalse(con.mouse.known)
        self.assertIsNone(con.task.pending_color)
        self.assertIsNone(con.last_frame)
        # Image clicks while switching must not create a target on a blank frame.
        con._mouse_event(cv2.EVENT_LBUTTONDOWN, 100, 100+CAMERA_BAR_HEIGHT, 0, None)
        self.assertFalse(con.mouse.known)
        con.render(telemetry(), False)
        self.assertIsNone(con.last_frame)

    def test_header_coordinates_map_back_to_camera_pixels(self):
        con = make_console(camera=camera())
        con.render(telemetry(), False)
        con._mouse_event(cv2.EVENT_MOUSEMOVE, 123, 234+CAMERA_BAR_HEIGHT, 0, None)
        self.assertEqual(con.mouse.position(), (123, 234))

    def test_switching_does_not_stop_command_loop(self):
        from pad_bridge import PadState
        con = make_console(camera=camera())
        con.camera.request_resolution(640, 480)
        clock = [0.]

        def key(_):
            clock[0] += .005
            return ord('q') if clock[0] >= .5 else 255

        with patch('console.time.monotonic', side_effect=lambda: clock[0]), \
             patch.object(con.pad, 'pump'), \
             patch.object(con.pad, 'read', return_value=PadState(0., 0., 0., False, [], [])), \
             patch('console.cv2.namedWindow'), patch('console.cv2.resizeWindow'), \
             patch('console.cv2.setMouseCallback'), patch('console.cv2.destroyAllWindows'), \
             patch('console.cv2.imshow'), patch('console.cv2.waitKey', side_effect=key):
            self.assertEqual(con.run(), 0)
        self.assertGreaterEqual(len(con.board.sent), 20)
