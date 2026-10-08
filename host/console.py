#!/usr/bin/env python3
"""console - the surface-side operator console.

One process owns everything on the laptop so that the four functions can share
it without fighting:

    Xbox controller ─┐
    camera ──────────┼──> console (this program) ──serial──> ESP32 ──> ESCs
    board telemetry ─┘

The window is a DJI-style OSD: camera in the middle, four corner panels.

    +------------------------------------------+
    | LINK  UNLOCK          L 1718  ####-      |   top-left     link + lock
    |                       R 1282  ##---      |   top-right    motor output
    |                                          |
    |              +------------+              |
    |              |  CAMERA    |              |   centre       video
    |              +------------+              |
    |                                          |
    | MODE  AprilTag        A  AprilTag        |   bottom-left  task result
    | ID 07  dist 1.24 m    B  Colour          |   bottom-right key hints
    +------------------------------------------+

Why one process rather than one script per function: a camera can only be
opened by one process at a time, importing OpenCV and warming the camera up
costs a second or two that a competition button press cannot afford, and two
processes reading the same controller is a mess. Pressing a key switches the
active task inside this loop while the 50 Hz drive loop keeps running, so the
motors never stutter when the mode changes.

The safety key is a toggle and always active, whatever task is selected. The
task only decides what is displayed and what the camera is used for; the stick
always drives.

Usage
    python3 console.py                  # full console
    python3 console.py --no-serial      # no board: controller + camera only
    python3 console.py --synthetic      # no camera: OSD on a test pattern
    python3 console.py --render-preview # write layout PNGs and exit
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

import numpy as np

import cv2

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from pad_bridge import (                       # noqa: E402
    DEFAULT_DEADMAN_BUTTON, Latch, Board, Pad, btn_name,
    find_ports, missing_deps, pick_port,
)
from vision.apriltag import AprilTagScanner    # noqa: E402
from vision.color import (                     # noqa: E402
    COLORS as POLE_COLORS, ColorDetector, calibrate,
    annotate as annotate_color,
)

# ===========================================================================
#  Configuration
# ===========================================================================

WINDOW = "UWR ROV Console"
MASKS_WINDOW = "Colour masks"

CAM_INDEX = 0
CAM_WIDTH = 1280
CAM_HEIGHT = 720

CAM_GAIN = 100.0        # OpenCV camera brightness, 0-200 (100 = untouched)
CAM_EXPOSURE = 0.0      # 0 = auto. Set negative for manual on some webcams.

# SDL's standard mapping puts A/B/X/Y at 0/1/2/3. Confirm on the real
# controller with  python3 pad_bridge.py --show-input  before trusting these.
BTN_APRILTAG = 0        # A
BTN_COLOR    = 1        # B
BTN_WIFI     = 2        # X
# Y (3) is the safety key - see DEFAULT_DEADMAN_BUTTON in pad_bridge.
BTN_CAPTURE  = 5        # RB: recognize once, in whichever task is active

# AprilTag capture window. Long enough to survive a frame blurred by a ripple,
# short enough that pressing RB still feels immediate.
TAG_CAPTURE_SECONDS = 0.3
TAG_CAPTURE_HITS = 2
# How many tags the mission asks for. Only used for the n/3 progress readout;
# the answer is recomputed after every capture rather than waiting for three.
TAG_CAPTURES_EXPECTED = 3

FONT = cv2.FONT_HERSHEY_SIMPLEX
THICKNESS = 1

WHITE = (255, 255, 255)
GREY = (150, 150, 150)
DARK = (60, 60, 60)
GREEN = (90, 220, 90)
AMBER = (60, 200, 240)
RED = (80, 80, 230)

PULSE_MIN = 1000
PULSE_MAX = 2000


# ===========================================================================
#  Camera
# ===========================================================================

def camera_backends():
    """Capture backend preference for this platform.

    Windows opens cameras far faster through DirectShow than through the
    default MSMF backend, which can take several seconds - long enough to look
    like the program has hung. macOS and Linux are left on the default.
    """
    if sys.platform.startswith("win"):
        return (cv2.CAP_DSHOW, cv2.CAP_ANY)
    return (cv2.CAP_ANY,)


class Camera:
    """Camera, with a synthetic fallback for testing without one.

    The fallback is not decoration: it lets the whole console be exercised on
    a desk with no camera attached, which is exactly when the OSD layout gets
    adjusted.
    """

    def __init__(self, index=CAM_INDEX, width=CAM_WIDTH, height=CAM_HEIGHT,
                 synthetic=False):
        self.synthetic = synthetic
        self.cap = None
        self.frame_no = 0

        if synthetic:
            return

        for backend in camera_backends():
            cap = cv2.VideoCapture(index, backend)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_GAIN, CAM_GAIN)
                if CAM_EXPOSURE:
                    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
                    cap.set(cv2.CAP_PROP_EXPOSURE, CAM_EXPOSURE)
                self.cap = cap
                return
            cap.release()

    @property
    def ok(self) -> bool:
        return self.synthetic or self.cap is not None

    def read(self):
        if self.synthetic:
            return self._synthetic()
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        return frame if ok else None

    def _synthetic(self):
        """A moving pattern, so the OSD is judged against a non-uniform image."""
        self.frame_no += 1
        w, h = CAM_WIDTH, CAM_HEIGHT
        frame = np.zeros((h, w, 3), np.uint8)
        x = np.linspace(0, 255, w, dtype=np.uint8)
        frame[:, :, 0] = x[None, :]
        frame[:, :, 1] = np.linspace(0, 255, h, dtype=np.uint8)[:, None]
        cv2.putText(frame, "SYNTHETIC TEST PATTERN - no camera",
                    (40, h // 2), FONT, 1.0, WHITE, 2)
        t = self.frame_no * 4
        cv2.circle(frame, (w // 2 + int(300 * np.cos(t / 40.0)),
                           h // 2 + int(200 * np.sin(t / 40.0))), 40, WHITE, 2)
        for i in range(0, w, 80):
            cv2.line(frame, (i, 0), (i, h), DARK, 1)
        for j in range(0, h, 80):
            cv2.line(frame, (0, j), (w, j), DARK, 1)
        return frame

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


# ===========================================================================
#  OSD drawing
# ===========================================================================

def _text_wh(text: str, fs: float):
    (w, h), base = cv2.getTextSize(text, FONT, fs, THICKNESS)
    return w, h + base


def _panel_box(img, x, y, w, h, anchor):
    """Place and shade a translucent panel. Returns its top-left corner."""
    if "r" in anchor:
        x -= w
    if "b" in anchor:
        y -= h
    # Clamp so a panel can never run off the frame, whatever the resolution.
    x = max(0, min(x, img.shape[1] - w))
    y = max(0, min(y, img.shape[0] - h))
    roi = img[y:y + h, x:x + w]
    if roi.size:
        cv2.addWeighted(np.zeros_like(roi), 0.42, roi, 0.58, 0, roi)
        cv2.rectangle(img, (x, y), (x + w, y + h), (90, 90, 90), 1)
    return x, y


def draw_panel(img, x, y, lines, anchor="tl", fs=0.5, pad=9, gap=6):
    """Draw a translucent panel of text lines anchored to a corner.

    anchor says which corner (x, y) refers to: "tl", "tr", "bl" or "br".
    Each entry in lines is either a string or a (string, colour) pair.
    """
    items = [(ln, WHITE) if isinstance(ln, str) else ln for ln in lines]
    if not items:
        return

    text_w = max((_text_wh(t, fs)[0] for t, _ in items), default=0)
    line_h = max((_text_wh(t, fs)[1] for t, _ in items), default=0) + gap
    w = text_w + pad * 2
    h = line_h * len(items) + pad * 2

    x, y = _panel_box(img, x, y, w, h, anchor)

    ty = y + pad
    for text, color in items:
        _, th = _text_wh(text, fs)
        cv2.putText(img, text, (x + pad, ty + th - gap), FONT, fs, color,
                    THICKNESS, cv2.LINE_AA)
        ty += line_h
    return x, y, w, h


def pulse_frac(us: int) -> float:
    return max(0.0, min(1.0, (us - PULSE_MIN) / float(PULSE_MAX - PULSE_MIN)))


# ===========================================================================
#  Tasks
# ===========================================================================

def close_window(name):
    """Close a window if it is open. Some OpenCV backends raise when asked
    about a window that was never created."""
    try:
        if cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) >= 1:
            cv2.destroyWindow(name)
    except cv2.error:
        pass


def select_box(frame, title):
    """Let the operator drag a box, returned in original-image pixels.

    Blocks the drive loop for as long as the selection window is open, which
    means the board's link timeout stops the motors. That is the safe
    direction - the operator is deliberately not driving while framing a box.
    """
    scale = min(1, 960 / frame.shape[1], 720 / frame.shape[0])
    preview = cv2.resize(frame, None, fx=scale, fy=scale) if scale < 1 else frame
    box = cv2.selectROI(title, preview, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow(title)
    if not box[2] or not box[3]:
        return None
    x, y, w, h = [round(v / scale) for v in box]
    return x, y, min(w, frame.shape[1] - x), min(h, frame.shape[0] - y)


class Task:
    """A function that a controller button switches to.

    Kept deliberately tiny so adding a mission is one small class plus one
    entry in TASKS. The stick keeps driving the ROV regardless of which task
    is active; a task only decides what is on screen and what the camera is
    used for.
    """

    name = "Manual"
    button = None
    hint = ""

    def enter(self, ctx):
        """Called when this task becomes active."""

    def leave(self, ctx):
        """Called when another task takes over."""

    def update(self, ctx, frame):
        """Called once per new frame while this task is active."""

    def view(self, ctx, frame):
        """The image to show in the centre. Default is the live frame."""
        return frame

    def overlay(self, ctx, frame):
        """Draw on the centre image, in place."""

    def info(self, ctx) -> list:
        """Lines for the bottom-left panel."""
        return [("MODE  Manual", AMBER),
                ("no task selected", GREY)]

    def on_key(self, key, ctx) -> bool:
        """Handle a printable key. Return True if it was consumed."""
        return False

    def keys(self, ctx) -> list:
        """Keyboard shortcuts for this task, as (key, what it does) pairs.

        S is handled globally by the console, so it is not listed here.
        """
        return []

    def capture(self, ctx) -> bool:
        """Recognize once, right now. False if this task has nothing to capture."""
        return False


class TaskAprilTag(Task):
    """Mission 2.1: identify the tags, then report the largest or smallest ID.

    Which of the two the judges want is announced on the day, so the operator
    picks it first with L or M. RB then takes one capture at a time; every ID
    confirmed in a capture is merged into a set, and the answer is recomputed
    after each one - readable before all three are in, in case two captures
    already cover the field.

    Nothing freezes. A frozen frame is the wrong tool when the operator is
    steering while scanning: they need to see where the ROV is going. Boxes
    are drawn while a capture window is open, and the collected IDs live in
    the panel.

    A capture that sees no tag does not count against the three, so a frame
    lost to a ripple costs nothing but the press.
    """

    name = "AprilTag"
    button = BTN_APRILTAG
    hint = "AprilTag"

    def __init__(self):
        self.mode = None            # None until the operator picks L or M
        self.ids = set()            # every ID confirmed so far
        self.captures = 0
        self.capturing = False
        self.started = 0.0

    def enter(self, ctx):
        self.reset(ctx)

    def leave(self, ctx):
        ctx.apriltag.reset()
        self.capturing = False

    def reset(self, ctx):
        """Start over. Keeps the selection logic - that is set by the judges,
        not by how the scan went."""
        self.ids = set()
        self.captures = 0
        self.capturing = False
        ctx.apriltag.reset()

    def choose(self, ctx, mode):
        self.mode = mode
        ctx.apriltag.mode = mode
        print(f"selection logic: {mode}")

    def capture(self, ctx):
        if self.mode is None:
            print("pick the selection logic first: L = largest, M = smallest")
            return False
        ctx.apriltag.reset()
        ctx.apriltag.min_hits = TAG_CAPTURE_HITS
        self.capturing = True
        self.started = ctx.now
        return True

    def update(self, ctx, frame):
        if not self.capturing:
            return
        ctx.apriltag.detect(frame)
        if ctx.now - self.started < TAG_CAPTURE_SECONDS:
            return

        self.capturing = False
        found = set(ctx.apriltag.confirmed_ids)
        if found:
            self.ids |= found
            self.captures += 1
        else:
            print("no tag confirmed in that capture - it did not count")

    @property
    def result(self):
        """The ID to report, or None while nothing has been identified."""
        if self.mode is None or not self.ids:
            return None
        return max(self.ids) if self.mode == "largest" else min(self.ids)

    def overlay(self, ctx, frame):
        # Boxes only while a capture is running. Drawing them afterwards would
        # leave boxes on a live picture pointing at where the tag used to be.
        if self.capturing:
            ctx.apriltag.annotate(frame)

    def info(self, ctx):
        lines = [("MODE  AprilTag", AMBER)]

        if self.mode is None:
            lines.append(("SELECTION LOGIC NOT SET", AMBER))
            lines.append(("press L = largest, M = smallest", WHITE))
            return lines

        lines.append((f"logic    {'largest' if self.mode == 'largest' else 'smallest'}"
                      f"  ({self.mode[0].upper()})", WHITE))

        if self.capturing:
            left = max(0.0, TAG_CAPTURE_SECONDS - (ctx.now - self.started))
            lines.append((f"CAPTURING {left:.1f}s", AMBER))
        else:
            lines.append((f"captures {self.captures}/{TAG_CAPTURES_EXPECTED}", WHITE))

        lines.append((f"ids      {sorted(self.ids) if self.ids else '-'}", WHITE))

        result = self.result
        if result is None:
            lines.append(("RESULT   --  (press RB)", GREY))
        else:
            done = self.captures >= TAG_CAPTURES_EXPECTED
            lines.append((f"RESULT   {result}" + ("  DONE" if done else ""), GREEN))
        return lines

    def on_key(self, key, ctx):
        if key == "c":
            return self.capture(ctx)
        if key == "l":
            self.choose(ctx, "largest")
            return True
        if key == "m":
            self.choose(ctx, "smallest")
            return True
        if key == "r":
            self.reset(ctx)
            return True
        return False

    def keys(self, ctx):
        return [("L", "logic: largest"),
                ("M", "logic: smallest"),
                ("C", "capture"),
                ("R", "reset scan")]


class TaskColor(Task):
    """Mission 5: find coloured poles.

    As with AprilTag, entering the task only previews. RB runs one burst, and
    the closing frame is frozen with the boxes drawn, because the result is a
    timestamped observation rather than live tracking and must not drift with
    the camera.
    """

    name = "Colour"
    button = BTN_COLOR
    hint = "Colour"

    def __init__(self):
        self.frozen = None

    def enter(self, ctx):
        ctx.color.clear()
        self.frozen = None

    def leave(self, ctx):
        ctx.color.clear()
        self.frozen = None
        close_window(MASKS_WINDOW)

    def capture(self, ctx):
        ctx.color.clear()
        self.frozen = None
        ctx.color.start(ctx.now)
        return True

    def update(self, ctx, frame):
        if not ctx.color.burst.active:
            return
        result = ctx.color.update(frame, ctx.now)
        if result is not None:
            snapshot = ctx.color.burst.snapshot
            self.frozen = annotate_color(snapshot.copy(), result, ctx.color.required)

    def view(self, ctx, frame):
        return self.frozen if self.frozen is not None else frame

    def info(self, ctx):
        c = ctx.color
        result = c.burst.result
        lines = [("MODE  Colour", AMBER),
                 (f"order    {'-'.join(c.order)}   next {c.required or 'done'}", WHITE)]
        if c.burst.active:
            left = max(0.0, c.settings.seconds - (ctx.now - c.burst.started))
            lines.append((f"CAPTURING {left:.1f}s", AMBER))
        elif result is None:
            lines.append(("READY - press RB to capture", GREY))
        else:
            lines.append((f"status   {result['status']}", WHITE))
            if result["targets"]:
                for item in result["targets"][:3]:
                    lines.append((f"{item['color']}  score {item['shape_score']:.2f}"
                                  f"  hits {item['hits']}/{result['frames']}",
                                  POLE_COLORS.get(item["color"], WHITE)))
            else:
                lines.append(("no pole found - aim and retry", GREY))
        if c.roi:
            lines.append(("ROI active  (F to clear)", GREY))
        return lines

    def keys(self, ctx):
        return [("C", "capture"),
                ("N", "next colour"),
                ("O", "select roi"),
                ("F", "clear roi"),
                ("1 2 3", "calibrate R/Y/B"),
                ("D", "colour masks"),
                ("R", "clear result")]

    def on_key(self, key, ctx):
        c = ctx.color
        if key == "c":
            return self.capture(ctx)
        if key == "r":
            c.clear()
            self.frozen = None
        elif key == "n":
            c.advance()
            self.frozen = None
        elif key == "f":
            c.set_roi(None)
            self.frozen = None
        elif key == "o":
            box = select_box(ctx.last_frame, "Select recognition ROI")
            if box:
                c.set_roi(box)
                self.frozen = None
        elif key in ("1", "2", "3"):
            color = {"1": "R", "2": "Y", "3": "B"}[key]
            box = select_box(ctx.last_frame, f"Sample the {color} pole only")
            if box:
                bands = calibrate(ctx.last_frame, box)
                if bands:
                    try:
                        c.apply_calibration(color, bands)
                        c.clear()
                        self.frozen = None
                        print(f"{color} calibrated and saved to {c.config_path}")
                    except (ValueError, OSError) as exc:
                        print(f"calibration failed: {exc}")
        elif key == "d":
            c.show_masks = not c.show_masks
            if not c.show_masks:
                close_window(MASKS_WINDOW)
        else:
            return False
        return True


class TaskWifi(Task):
    """Mission 1: the board does the work, this only shows the result.

    Entering the task asks the board for a fresh fetch, so the referee sees
    current data rather than whatever the last poll happened to return.
    """

    name = "WiFi"
    button = BTN_WIFI
    hint = "WiFi"

    def enter(self, ctx):
        if ctx.board is not None:
            try:
                ctx.board.send_raw("F\n")
            except Exception:
                pass

    def info(self, ctx):
        m = ctx.mission
        lines = [("MODE  Mission 1", AMBER),
                 (f"wifi  {m['wifi']}   rssi {m['rssi']}", WHITE),
                 (f"ip    {m['ip']}", WHITE),
                 (f"http  {m['http']}", WHITE)]
        if m["data"]:
            body = m["data"]
            lines.append((f"DATA  {body[:44]}", GREEN))
            if len(body) > 44:
                lines.append((f"      {body[44:88]}", GREEN))
        else:
            lines.append(("DATA  (nothing received yet)", GREY))
        return lines


TASKS = [TaskAprilTag, TaskColor, TaskWifi]


# ===========================================================================
#  Console
# ===========================================================================

class Console:

    def __init__(self, pad, board, camera, use_deadman, show_help=True):
        self.pad = pad
        self.board = board
        self.camera = camera
        self.use_deadman = use_deadman
        self.latch = Latch(locked=True)
        self.task = Task()                 # Manual
        self.show_help = show_help
        self.frames = 0
        self.fps = 0.0
        self._t_fps = time.monotonic()
        self.now = time.monotonic()
        self.last_frame = None
        self.mission = {"wifi": "-", "rssi": "-", "ip": "-", "http": "-",
                        "data": ""}
        self.apriltag = AprilTagScanner(mode="largest", min_hits=3, expected=3)
        self.color = ColorDetector()
        if board is not None:
            board.on_line = self._on_board_line

    # ---- board events -------------------------------------------------
    def _on_board_line(self, line: str):
        """Consume the non-JOY lines: ST carries Mission 1 status, DATA the
        payload shown to the referee."""
        parts = line.split()
        if not parts:
            return
        tag = parts[0].upper()
        if tag == "DATA":
            self.mission["data"] = line[len(parts[0]):].strip()
        elif tag == "ST":
            for token in parts[1:]:
                if "=" not in token:
                    continue
                k, v = token.split("=", 1)
                k = k.lower()
                if k in ("wifi", "rssi", "ip", "http"):
                    self.mission[k] = v

    # ---- task switching -----------------------------------------------
    def _switch(self, task_cls):
        """Select a task, or go back to Manual if it is already active."""
        if type(self.task) is task_cls:
            new = Task()
        else:
            new = task_cls()
        self.task.leave(self)
        self.task = new
        self.task.enter(self)

    @staticmethod
    def _edge(pressed: list, prev: list, button: int) -> bool:
        """True on the frame a button goes down, not while it is held."""
        if button >= len(pressed) or not pressed[button]:
            return False
        return button >= len(prev) or not prev[button]

    def _handle_buttons(self, pressed: list, prev: list):
        for task_cls in TASKS:
            if task_cls.button is not None and self._edge(pressed, prev, task_cls.button):
                self._switch(task_cls)
        if self._edge(pressed, prev, BTN_CAPTURE):
            # A task that can capture explains its own refusals; only Manual
            # and WiFi have nothing to say here.
            if type(self.task).capture is Task.capture:
                print("nothing to capture - press A or B first")
            else:
                self.task.capture(self)

    # ---- OSD ----------------------------------------------------------
    def _draw_status(self, img, telem, link, en):
        if self.pad.connected:
            dot, link_txt = GREEN, "LINK"
        elif self.board is None:
            dot, link_txt = GREY, "NO BOARD"
        else:
            dot, link_txt = RED, "NO PAD"

        if not self.use_deadman:
            lock_txt, lock_col = "NO SAFETY KEY", AMBER
        elif not self.pad.connected:
            lock_txt, lock_col = "LOCKED (pad lost)", RED
        elif en:
            lock_txt, lock_col = "UNLOCK", GREEN
        else:
            lock_txt, lock_col = "LOCK", RED

        x, y = 16, 16
        cv2.circle(img, (x + 7, y + 12), 7, dot, -1)
        cv2.putText(img, link_txt, (x + 22, y + 18), FONT, 0.5, WHITE,
                    THICKNESS, cv2.LINE_AA)
        _, py, _, ph = draw_panel(img, 16, 40, [(lock_txt, lock_col)],
                                  anchor="tl", fs=0.5)
        if self.show_help:
            draw_panel(img, 16, py + ph + 4, [(f"{self.fps:4.1f} fps", GREY)],
                       anchor="tl", fs=0.42, pad=6)

    def _draw_motors(self, img, telem):
        """Motor panel: one row per side, each with its own pulse-width gauge.

        Hand-rolled rather than using draw_panel, because draw_panel places
        text and gauges in separate bands and this needs a gauge beside each
        row.
        """
        if telem is None:
            draw_panel(img, img.shape[1] - 16, 16, [("waiting for board", GREY)],
                       anchor="tr", fs=0.5)
            return

        fs, pad, gap = 0.5, 9, 6
        rows = [("L", telem.l_us), ("R", telem.r_us)]
        label = f"L {PULSE_MAX:4d}"
        label_w, label_h = _text_wh(label, fs)
        line_h = label_h + gap
        bar_w, bar_h = 180, label_h - 6

        w = pad * 2 + label_w + 12 + bar_w
        h = pad * 2 + line_h * len(rows)
        x, y = _panel_box(img, img.shape[1] - 16, 16, w, h, "tr")

        ty = y + pad
        for name, us in rows:
            cv2.putText(img, f"{name} {us:4d}", (x + pad, ty + label_h - gap),
                        FONT, fs, WHITE, THICKNESS, cv2.LINE_AA)
            bx = x + pad + label_w + 12
            by = ty + 3
            cv2.rectangle(img, (bx, by), (bx + bar_w, by + bar_h), DARK, -1)
            cv2.rectangle(img, (bx, by),
                          (bx + int(bar_w * pulse_frac(us)), by + bar_h),
                          AMBER, -1)
            ty += line_h

    def _draw_hints(self, img):
        """Two panels in the bottom-right corner: the controller map, and just
        above it the keyboard shortcuts for whatever task is active."""
        lines = []
        for task_cls in TASKS:
            active = type(self.task) is task_cls
            lines.append((f"{btn_name(task_cls.button):<4} {task_cls.hint:<9}"
                          f"[{'ON' if active else '  '}]",
                          AMBER if active else GREY))

        capturable = type(self.task).capture is not Task.capture
        lines.append((f"{btn_name(BTN_CAPTURE):<4} {'Capture':<9}"
                      f"[{'ON' if capturable else '  '}]",
                      GREEN if capturable else GREY))

        btn = self.pad.deadman_button
        if not self.use_deadman:
            lines.append((f"{'':<4} no safety key", GREY))
        else:
            locked = self.latch.locked
            lines.append((f"{btn_name(btn):<4} Lock     "
                          f"[{'LOCKED' if locked else 'UNLOCK'}]",
                          GREY if locked else GREEN))
        _, py, _, _ = draw_panel(img, img.shape[1] - 16, img.shape[0] - 16,
                                 lines, anchor="br", fs=0.5)

        keys = [("Q Esc", "quit"), ("S", "save png")] + list(self.task.keys(self))
        draw_panel(img, img.shape[1] - 16, py - 6, self._key_lines(keys),
                   anchor="br", fs=0.44, pad=7)

    @staticmethod
    def _key_lines(keys):
        """Render (key, action) pairs as aligned rows."""
        width = max(len(k) for k, _ in keys)
        return [(f"{k:<{width}}   {action}", GREY) for k, action in keys]

    def draw_osd(self, img, telem, link, en):
        self._draw_status(img, telem, link, en)
        self._draw_motors(img, telem)
        draw_panel(img, 16, img.shape[0] - 16, self.task.info(self),
                   anchor="bl", fs=0.5)
        self._draw_hints(img)

    def snapshot(self, frame):
        folder = pathlib.Path(__file__).resolve().parent / "snapshots"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / time.strftime("rov-%Y%m%d-%H%M%S.png")
        if cv2.imwrite(str(path), frame):
            print(f"saved {path}")
        else:
            print(f"could not write {path}")

    def render(self, telem, en, frame=None):
        """Produce one finished OSD frame. Used by the live loop and by
        --render-preview, so the preview can never drift from the real thing."""
        if frame is None:
            frame = self.camera.read()
        if frame is None:
            frame = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), np.uint8)
        self.now = time.monotonic()
        self.last_frame = frame

        # The task runs on the live frame, then draws on whatever it wants
        # shown. Copying before the overlay keeps a task from painting onto
        # the buffer the camera will hand back next tick.
        self.task.update(self, frame)
        view = self.task.view(self, frame)
        if view is frame:
            view = frame.copy()
        self.task.overlay(self, view)

        self.draw_osd(view, telem, telem is not None, en)
        return view

    # ---- main loop ----------------------------------------------------
    def run(self) -> int:
        print("Console running. Q or Esc in the window quits.")
        print("Drag the window edge to resize; the panels follow.")
        prev_buttons = []
        sent = 0
        t_last = 0.0
        period = 1.0 / 50.0
        warned = False
        t_start = time.monotonic()

        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1280, 720)

        try:
            while True:
                now = time.monotonic()
                self.pad.pump()

                if self.pad.connected:
                    nx, ny, key_down, _, pressed = self.pad.read()
                    self._handle_buttons(pressed, prev_buttons)
                    prev_buttons = pressed
                else:
                    nx, ny, key_down = 0.0, 0.0, False
                    prev_buttons = []

                if not self.use_deadman:
                    en = True
                elif not self.pad.connected:
                    self.latch.force_lock()
                    en = False
                else:
                    en = self.latch.update(key_down)

                if now - t_last >= period:
                    t_last = now
                    if self.board is not None:
                        try:
                            self.board.send(nx, ny, en)
                            sent += 1
                        except Exception as exc:
                            print(f"serial write failed: {exc}")
                            return 2

                telem = None
                if self.board is not None:
                    self.board.poll()
                    telem = self.board.telem
                    if telem is None and not warned and now - t_start > 4.0:
                        warned = True
                        print("No telemetry from the board yet. Check it is "
                              "powered, flashed with the ROV firmware, and that "
                              f"{self.board.port} is the right port.")

                img = self.render(telem, en)

                self.frames += 1
                if now - self._t_fps >= 0.5:
                    self.fps = self.frames / (now - self._t_fps)
                    self.frames = 0
                    self._t_fps = now

                cv2.imshow(WINDOW, img)
                if self.color.show_masks and self.color.burst.masks:
                    cv2.imshow(MASKS_WINDOW,
                               np.hstack(list(self.color.burst.masks.values())))

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if 32 <= key < 127:
                    ch = chr(key).lower()
                    if ch == "s":
                        # Saving works the same in every task, so it does not
                        # belong to any one of them.
                        self.snapshot(self.task.view(self, self.last_frame))
                    else:
                        self.task.on_key(ch, self)
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()
            self.camera.release()

        print("\nExiting (the board returns to neutral on link timeout).")
        return 0


# ===========================================================================
#  Headless layout preview
# ===========================================================================

def render_preview(outdir: pathlib.Path) -> int:
    """Write one PNG per screen state, without a camera or a board.

    Layout bugs are easy to introduce and hard to see, so this renders the
    real draw path rather than a copy of it.
    """
    from pad_bridge import Telemetry

    outdir.mkdir(parents=True, exist_ok=True)

    cam = Camera(synthetic=True)
    pad = Pad(0, 1, DEFAULT_DEADMAN_BUTTON, False, False)
    pad.js = object()      # stands in for a connected controller in preview only
    console = Console(pad=pad, board=None, camera=cam, use_deadman=True)
    console.fps = 30.0

    telem = Telemetry()
    telem.l_us, telem.r_us = 1718, 1282
    telem.tl_us, telem.tr_us = 1718, 1282
    telem.en, telem.link, telem.ready = True, True, True

    states = [
        ("manual_locked", None, False),
        ("manual_unlocked", telem, True),
        ("nolink", None, False),
        ("apriltag", telem, True),
        ("color", telem, True),
        ("wifi", telem, True),
    ]

    for name, t, en in states:
        if name == "aprilag":
            console.task = TaskAprilTag()
        elif name == "color":
            console.task = TaskColor()
        elif name == "wifi":
            console.task = TaskWifi()
            console.mission = {"wifi": "CONNECTED", "rssi": "-58", "ip":
                               "192.168.4.7", "http": "200",
                               "data": "MISSION1 DATA OK - underwater robotics"}
        else:
            console.task = Task()

        # "nolink" shows what the operator sees if the controller drops out.
        console.pad.js = None if name == "nolink" else object()
        console.latch.locked = not en
        console.latch._was_down = en        # let the toggle model match the state

        img = console.render(t, en, frame=cam.read())
        path = outdir / f"osd_{name}.png"
        cv2.imwrite(str(path), img)
        print(f"wrote {path}")

    cam.release()
    return 0


# ===========================================================================
#  Entry point
# ===========================================================================

def main() -> int:
    p = argparse.ArgumentParser(
        description="UWR ROV operator console: controller, camera and a DJI-style OSD")
    p.add_argument("--port", help="serial device (auto-detected if omitted)")
    p.add_argument("--no-serial", action="store_true", help="no board attached")
    p.add_argument("--no-deadman", action="store_true",
                   help="no safety key required (first debugging only, risky)")
    p.add_argument("--deadman", type=int, default=DEFAULT_DEADMAN_BUTTON,
                   help="safety key button number "
                        f"(default {DEFAULT_DEADMAN_BUTTON}, which is Y)")
    p.add_argument("--camera", type=int, default=CAM_INDEX, help="camera index")
    p.add_argument("--synthetic", action="store_true",
                   help="no camera: draw the OSD over a test pattern")
    p.add_argument("--no-help", action="store_true", help="hide the fps readout")
    p.add_argument("--render-preview", metavar="DIR",
                   help="write layout PNGs to DIR and exit (no hardware needed)")
    p.add_argument("--axis-x", type=int, default=0, help="stick X axis number")
    p.add_argument("--axis-y", type=int, default=1, help="stick Y axis number")
    p.add_argument("--invert-x", action="store_true", help="negate X")
    p.add_argument("--invert-y", action="store_true", help="negate Y")
    args = p.parse_args()

    if args.render_preview:
        return render_preview(pathlib.Path(args.render_preview))

    try:
        import pygame  # noqa: F401
    except ImportError:
        missing_deps()
        return 2

    pad = Pad(args.axis_x, args.axis_y, args.deadman, args.invert_x, args.invert_y)
    if not pad.open():
        print("No controller found. Plug in USB or pair Bluetooth, then confirm "
              "with  python3 pad_bridge.py --show-input")
        return 2
    print(f"controller: {pad.name}")

    if args.no_deadman:
        print("--no-deadman: the motors are live without pressing anything.")
    else:
        pad.deadman_button = (DEFAULT_DEADMAN_BUTTON if args.deadman is None
                              else args.deadman)
        print(f"safety key = {btn_name(pad.deadman_button)} "
              f"(button {pad.deadman_button}). Starts locked.")

    board = None
    if not args.no_serial:
        try:
            import serial  # noqa: F401
        except ImportError:
            missing_deps()
            return 2
        port = args.port
        if not port:
            chosen, cands = pick_port(find_ports())
            if chosen:
                port = chosen
            elif not cands:
                print("No serial ports found. Is the board plugged in? "
                      "Use --no-serial to run without it.")
                return 2
            else:
                print("Several ports could be the board; pick one with --port:")
                for x in cands:
                    print("   ", x)
                return 2
        board = Board(port)
        try:
            board.open()
        except Exception as exc:
            print(f"cannot open {port}: {exc}")
            return 2
        print(f"serial: {port} @ 115200")

    camera = Camera(args.camera, synthetic=args.synthetic)
    if not camera.ok:
        print(f"Camera {args.camera} did not open. Use --synthetic to run the "
              "console without one, or --camera N to pick another index.")
        return 2
    print("camera: synthetic test pattern" if args.synthetic else "camera: ok")

    console = Console(pad, board, camera, not args.no_deadman,
                      show_help=not args.no_help)
    try:
        return console.run()
    finally:
        if board is not None:
            board.close()


if __name__ == "__main__":
    raise SystemExit(main())
