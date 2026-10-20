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

Pressing T opens a command line in the status stack: type a board command,
press Enter, and the board's reply (OK / ERR / EV) stays on screen. It exists
because the alternative is a serial monitor, whose telemetry arrives 20 times
a second and scrolls the one line you sent out of sight before you can read it.

Usage
    python3 console.py
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
import threading
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
    CALIBRATE_RADIUS, COLORS as POLE_COLORS, NAMES, ColorDetector,
    box_around, calibrate, masks, sample_at,
)

# ===========================================================================
#  Configuration
# ===========================================================================

WINDOW = "UWR ROV Console"
MASKS_WINDOW = "Colour masks"

CAM_INDEX = 0
CAM_WIDTH = 1280
CAM_HEIGHT = 720
# How far to look for a camera. Six is more than a laptop and a USB hub are
# likely to present, and probing an index that is empty can be slow.
CAMERA_SCAN_LIMIT = 6

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

# Longest command the console will accept, in characters. The firmware
# discards any line longer than LINK_LINE_MAX (256) whole, so there is nothing
# to gain past this; a limit that bites is better than one that silently
# throws the line away.
COMMAND_MAX = 80

# Networks shown in the WiFi panel. The firmware reports up to M1_SCAN_MAX;
# more than this and the bottom-left panel grows up into the status stack.
NETS_SHOWN = 5

# "OK url = http://...", and "OK P url = http://..." after a P command. The
# value is taken as the rest of the line: an SSID may contain spaces, so
# splitting on whitespace would cut "HUAWEI JACK" to "HUAWEI".
REPLY_RE = re.compile(r"^OK\s+(?:P\s+)?(\w+)\s*=\s*(.*)$")

FONT = cv2.FONT_HERSHEY_SIMPLEX
THICKNESS = 1

WHITE = (255, 255, 255)
POINTER_COLOR = (255, 255, 0)    # cyan: distinct from every panel and pole
GREY = (205, 205, 205)      # de-emphasised, but still readable on a panel
DARK = (60, 60, 60)
GREEN = (90, 220, 90)
AMBER = (60, 200, 240)
RED = (80, 80, 230)

PULSE_MAX = 2000
PULSE_NEUTRAL = 1500

# Commanded vz below this reads as no command, so the UP / HOLD / DOWN label
# does not flicker while the stick settles.
VERTICAL_DEADBAND = 0.05


def vertical_label(z: float) -> str:
    """UP / HOLD / DOWN for the vertical thruster.

    Read from the commanded direction, not from the pulse width. The pulse is
    whatever number the ESC is given, and on this vehicle the vertical thruster
    is mounted the other way round, so 1000 drives it up: labelling from the
    pulse would put the word opposite to what the ROV does. The firmware owns
    that inversion; Z= in the telemetry stays "positive is up".

    One row rather than two: the thruster has a single channel, so UP and DOWN
    are the two ends of the same value. A deadband keeps the label from
    flickering while the command settles.
    """
    if z > VERTICAL_DEADBAND:
        return "UP"
    if z < -VERTICAL_DEADBAND:
        return "DOWN"
    return "HOLD"


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


def probe_camera(index):
    """Open one camera index and report (width, height, delivers_frames), or
    None if it will not open. The device is closed again immediately.

    Used by --list-cameras, so picking the right index is a look rather than
    a guess. Opening a camera that is not there can be slow, which is why the
    scan is a separate command rather than something the console does at
    startup.
    """
    for backend in camera_backends():
        cap = cv2.VideoCapture(index, backend)
        if not cap.isOpened():
            cap.release()
            continue
        size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        delivers = False
        for _ in range(6):
            ok, frame = cap.read()
            if ok and frame is not None:
                size = (frame.shape[1], frame.shape[0])
                delivers = True
                break
            time.sleep(0.05)
        cap.release()
        return size[0], size[1], delivers
    return None


def list_cameras(limit=CAMERA_SCAN_LIMIT) -> int:
    print(f"probing camera indices 0-{limit - 1}, each opened and closed again")
    found = []
    for index in range(limit):
        result = probe_camera(index)
        if result is None:
            print(f"  {index}: -")
            continue
        width, height, delivers = result
        note = "" if delivers else "   (opened, but no frames)"
        print(f"  {index}: {width}x{height}{note}")
        if delivers:
            found.append(index)
    if not found:
        print("\nNo usable camera. On macOS, check System Settings -> Privacy "
              "& Security -> Camera; a denied permission looks exactly like "
              "this.")
        return 2
    print(f"\nusable: {', '.join(str(i) for i in found)}")
    print(f"pick one with:  --camera {found[0]}")
    return 0


def upright(frame):
    """Turn a frame the right way up.

    The camera is mounted upside down on the ROV, so every frame arrives
    rotated 180 degrees.

    Doing it here, on the raw frame as it comes off the device, is the only
    place that stays correct. Everything downstream is then working in the
    space the operator sees: the OSD is drawn on the corrected picture, and
    the pointer position reported by the window refers to the same pixels the
    colour task samples. Correcting anywhere later would put the crosshair on
    one side of the pool and the sample it reads on the other.
    """
    return cv2.rotate(frame, cv2.ROTATE_180)


class Camera:
    """Camera, read on a background thread.

    Reading on the main loop would stall the whole console whenever the
    camera hiccups, and a stalled console stops sending drive commands - the
    board would drop the motors to neutral mid-manoeuvre. A worker keeps the
    newest frame available without the main loop ever waiting on the device.

    It is also what makes reconnect possible. Reopening a camera that has been
    unplugged takes seconds; seconds the operator would otherwise spend
    unable to steer while the picture was gone anyway.

    On reconnect only the index that last worked is retried. Scanning for a
    replacement risks silently picking a different camera - the laptop's
    built-in one, say - and showing the operator the ceiling instead of the
    pool. If the camera comes back elsewhere, --list-cameras will say where.
    """

    # Consecutive failed reads before the camera is declared gone.
    LOST_AFTER = 15
    RESCAN_SECONDS = 1.0        # how often to retry while it is gone

    def __init__(self, index=CAM_INDEX, width=CAM_WIDTH, height=CAM_HEIGHT):
        self.index = index
        self.width = width
        self.height = height
        self.lost = False

        self._cap = None
        self._frame = None
        self._misses = 0
        self._next_try = 0.0
        self._stop = threading.Event()
        self._thread = None

        # At startup, look for a camera rather than giving up on the first
        # index: an index that is wrong is a guess, not a statement.
        self._open(scan=True)
        if self._cap is None:
            self.lost = True
            return

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        # Wait for the first frame so a camera that opens but never delivers
        # is reported at startup rather than as a window that stays black.
        deadline = time.monotonic() + 3.0
        while self._frame is None and time.monotonic() < deadline:
            if self.lost:
                break
            time.sleep(0.02)

    def _open(self, scan=False) -> bool:
        """Try to open the camera. scan=True looks past the requested index."""
        order = [self.index]
        if scan:
            order += [i for i in range(CAMERA_SCAN_LIMIT) if i != self.index]

        for index in order:
            for backend in camera_backends():
                cap = cv2.VideoCapture(index, backend)
                if not cap.isOpened():
                    cap.release()
                    continue
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_GAIN, CAM_GAIN)
                if CAM_EXPOSURE:
                    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
                    cap.set(cv2.CAP_PROP_EXPOSURE, CAM_EXPOSURE)
                self._cap = cap
                if index != self.index:
                    print(f"camera {self.index} was not there; using {index}")
                self.index = index
                return True
        self._cap = None
        return False

    def _run(self):
        while not self._stop.is_set():
            if self._cap is None:
                if time.monotonic() < self._next_try:
                    time.sleep(0.1)
                    continue
                self._next_try = time.monotonic() + self.RESCAN_SECONDS
                if self._open():
                    self.lost = False
                    print(f"camera {self.index} reconnected")
                continue

            ok, frame = self._cap.read()
            if ok and frame is not None:
                # A fresh array each read, so publishing it needs no lock:
                # the worker never writes into a frame it has handed over.
                # upright() returns a new array too, so that still holds.
                self._frame = upright(frame)
                self._misses = 0
                self.lost = False
            else:
                self._misses += 1
                if self._misses >= self.LOST_AFTER:
                    self._cap.release()
                    self._cap = None
                    self.lost = True
                    self._next_try = 0.0     # try again at once
                    print("camera lost - unplugged, or taken by another program")

    def read(self):
        """The newest frame, or None. Never blocks on the device."""
        return self._frame

    @property
    def ok(self) -> bool:
        """Frames are actually arriving."""
        return self._frame is not None

    def release(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None


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
        # Shade hard rather than lightly. The panels sit over whatever the
        # camera is pointed at, and the pool surface is bright: at 0.58 the
        # backdrop of a white tile came out at 148, which is the same value as
        # GREY - every grey row of text disappeared into it. At 0.32 the
        # backdrop is around 82 whatever is behind it, so light text reads
        # from the surface to the bottom of the pool.
        cv2.addWeighted(np.zeros_like(roi), 0.68, roi, 0.32, 0, roi)
        cv2.rectangle(img, (x, y), (x + w, y + h), (110, 110, 110), 1)
    return x, y


def fit_text(text, img, fs, keep_head=True, margin=48):
    """Shorten text until it fits across the frame.

    draw_panel clamps its corner to the frame but a line wider than the frame
    still runs off the right edge and is cut without saying so. Trimming here
    at least leaves an ellipsis behind, so a shortened answer does not look
    like a complete one.
    """
    avail = img.shape[1] - margin
    if _text_wh(text, fs)[0] <= avail:
        return text
    while text:
        shown = text + "…" if keep_head else "…" + text
        if _text_wh(shown, fs)[0] <= avail:
            return shown
        text = text[:-1] if keep_head else text[1:]
    return "…"


def draw_panel(img, x, y, lines, anchor="tl", fs=0.5, pad=9, gap=6, title=None):
    """Draw a translucent panel of text lines anchored to a corner.

    anchor says which corner (x, y) refers to: "tl", "tr", "bl" or "br".
    Each entry in lines is either a string or a (string, colour) pair.
    title draws a heading on its own rule above the lines. The two hint panels
    need it: one lists controller buttons and one lists keyboard keys, and
    letters like C and R mean different things depending on which is which.
    """
    items = [(ln, WHITE) if isinstance(ln, str) else ln for ln in lines]
    if not items:
        return

    text_w = max((_text_wh(t, fs)[0] for t, _ in items), default=0)
    line_h = max((_text_wh(t, fs)[1] for t, _ in items), default=0) + gap

    title_fs = fs * 0.85
    title_w, title_h = 0, 0
    if title:
        title_w, th = _text_wh(title, title_fs)
        title_h = th + gap + 4          # the heading plus its rule

    w = max(text_w, title_w) + pad * 2
    h = line_h * len(items) + title_h + pad * 2

    x, y = _panel_box(img, x, y, w, h, anchor)

    ty = y + pad
    if title:
        _, th = _text_wh(title, title_fs)
        cv2.putText(img, title, (x + pad, ty + th - gap), FONT, title_fs,
                    WHITE, THICKNESS, cv2.LINE_AA)
        ry = ty + th + 2
        cv2.line(img, (x + pad, ry), (x + w - pad, ry), GREY, 1)
        ty += title_h

    for text, color in items:
        _, th = _text_wh(text, fs)
        cv2.putText(img, text, (x + pad, ty + th - gap), FONT, fs, color,
                    THICKNESS, cv2.LINE_AA)
        ty += line_h
    return x, y, w, h


def throttle_frac(us: int) -> float:
    """Signed throttle, -1..+1, from a pulse width. 0 is neutral.

    The panel shows throttle rather than microseconds: 1718 is a number the
    ESC understands, not one the operator can act on at a glance.
    """
    half = (PULSE_MAX - PULSE_NEUTRAL)
    return max(-1.0, min(1.0, (us - PULSE_NEUTRAL) / float(half)))


def motor_rows(telem) -> list:
    """The three throttle rows, as (label, signed value, how to write it).

    The right thruster is negated. The two are mounted anti-parallel (see the
    mixer in main.cpp), so its "forward" is the low end of its pulse range:
    L 2000 / R 1000 drives the ROV straight ahead. Showing that raw reads as
    one thruster full ahead and one full astern, when the vehicle is doing
    neither. Negated, "+" on every row means "this thruster drives the ROV
    forward", and a spin shows as the two rows pointing opposite ways.

    The vertical row takes the commanded vz rather than its pulse, for the
    same reason: that thruster is inverted as well, and vz already means
    "positive is up".
    """
    return [("L", throttle_frac(telem.l_us), "signed"),
            ("R", -throttle_frac(telem.r_us), "signed"),
            (vertical_label(telem.z), telem.z, "magnitude")]


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


class Mouse:
    """Where the cursor is over the console window.

    OpenCV reports coordinates in image space, so a resized window needs no
    correction. It only reports while the cursor is over the window, so the
    last known point is kept: aiming, then moving off to the controller to
    press RB, is the intended way to use this.
    """

    def __init__(self):
        self.x = None
        self.y = None

    @property
    def known(self) -> bool:
        return self.x is not None and self.y is not None

    def position(self):
        return (self.x, self.y) if self.known else (None, None)

    def on_event(self, event, x, y, flags, param):
        self.x, self.y = x, y


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
        """Keyboard shortcuts for this task, as (key, what it does) pairs."""
        return []

    def commands(self, ctx) -> list:
        """Board commands worth showing while one is being typed, as
        (command, what it does) pairs."""
        return []

    def capture(self, ctx) -> bool:
        """Recognize once, right now. False if this task has nothing to capture."""
        return False

    def evidence_seq(self) -> int:
        """A number that goes up when a capture has just produced a result.

        The console saves the frame the moment it changes, so the picture kept
        as evidence carries the answer the operator was shown - not the empty
        preview from before they pressed the button.
        """
        return 0


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

    def evidence_seq(self):
        # One per capture that actually confirmed something. A press that saw
        # nothing produces no result, so there is nothing to keep as evidence.
        return self.captures

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

        # Name the key, not the first letter of the word. "smallest" starts
        # with S, but the key that selects it is M, and a bracketed letter in
        # this panel reads as the key to press.
        lines.append((f"logic    {self.mode}"
                      f"  ({'L' if self.mode == 'largest' else 'M'})", WHITE))

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

    Point at a pole and press RB. The patch under the pointer decides which
    colour it is, and the answer is written along the bottom of the picture
    in that colour. Nothing runs until asked, and nothing freezes: the
    operator is choosing where to look, which needs a live view.

    Calibration works the same way - point at bare pole and press 1, 2 or 3 -
    so the whole task is one gesture rather than two.
    """

    name = "Colour"
    button = BTN_COLOR
    hint = "Colour"

    def __init__(self):
        self.answer = None          # colour from the last capture, or None
        self.seq = 0                # bumps per answer, for the evidence shot

    def enter(self, ctx):
        self.answer = None

    def evidence_seq(self):
        return self.seq

    def leave(self, ctx):
        self.answer = None
        close_window(MASKS_WINDOW)

    def capture(self, ctx):
        x, y = ctx.mouse.position()
        if x is None:
            print("move the mouse over the picture first, then press RB")
            return False
        self.answer = sample_at(ctx.last_frame, x, y, ctx.color.ranges)
        if self.answer is None:
            print("nothing recognisable under the pointer - aim at bare pole")
        else:
            self.seq += 1
            print(f"color recognition = {NAMES[self.answer]}")
        return True

    def calibrate_from_pointer(self, ctx, color):
        x, y = ctx.mouse.position()
        if x is None:
            print(f"point at bare {color} pole first, then press the key")
            return
        box = box_around(x, y, CALIBRATE_RADIUS, ctx.last_frame.shape)
        bands = calibrate(ctx.last_frame, box)
        if not bands:
            return
        try:
            ctx.color.apply_calibration(color, bands)
            print(f"{color} calibrated from the pointer, saved to "
                  f"{ctx.color.config_path}")
        except (ValueError, OSError) as exc:
            print(f"calibration failed: {exc}")

    def overlay(self, ctx, frame):
        # Crosshair where the program thinks the pointer is. Without it a
        # capture that misses the pole looks like a broken detector.
        x, y = ctx.mouse.position()
        if x is not None:
            cv2.drawMarker(frame, (x, y), POINTER_COLOR, cv2.MARKER_CROSS,
                           22, 2, cv2.LINE_AA)

        if self.answer is not None:
            self._draw_answer(frame, self.answer)

    @staticmethod
    def _draw_answer(frame, color):
        """The answer, centred along the bottom of the picture, in its own
        colour. On a dark backing: yellow text on a sunlit pool floor would
        otherwise be invisible."""
        text = f"color recognition = {NAMES[color]}"
        fs, thickness = 0.9, 2
        (tw, th), base = cv2.getTextSize(text, FONT, fs, thickness)
        x = max(8, (frame.shape[1] - tw) // 2)
        y = frame.shape[0] - 24
        cv2.rectangle(frame, (x - 14, y - th - 10), (x + tw + 14, y + base + 6),
                      (0, 0, 0), -1)
        cv2.putText(frame, text, (x, y), FONT, fs, POLE_COLORS[color],
                    thickness, cv2.LINE_AA)

    def info(self, ctx):
        lines = [("MODE  Colour", AMBER)]

        if not ctx.mouse.known:
            lines.append(("move the mouse over the picture first", GREY))
        else:
            lines.append(("READY - point at a pole, press RB", GREY))

        if self.answer is None:
            lines.append(("answer   --", GREY))
        else:
            lines.append((f"answer   {NAMES[self.answer]}",
                          POLE_COLORS[self.answer]))
        return lines

    def keys(self, ctx):
        return [("C", "capture"),
                ("1 2 3", "calibrate R/Y/B"),
                ("R", "clear answer"),
                ("D", "colour masks")]

    def on_key(self, key, ctx):
        if key == "c":
            return self.capture(ctx)
        if key == "r":
            self.answer = None
            return True
        if key in ("1", "2", "3"):
            self.calibrate_from_pointer(ctx, {"1": "R", "2": "Y", "3": "B"}[key])
            return True
        if key == "d":
            ctx.color.show_masks = not ctx.color.show_masks
            if not ctx.color.show_masks:
                close_window(MASKS_WINDOW)
            return True
        return False


class TaskWifi(Task):
    """Mission 1: the board does the work, this only shows the result.

    Entering the task asks the board for a fresh fetch, so the referee sees
    current data rather than whatever the last poll happened to return.
    """

    name = "WiFi"
    button = BTN_WIFI
    hint = "WiFi"

    def enter(self, ctx):
        # A scan from a previous visit is stale and would keep the DATA line
        # hidden, so start clean.
        ctx.mission["nets"] = []
        ctx.mission["scan_new"] = []
        ctx.mission["scanning"] = False
        ctx.mission["scan_count"] = None
        if ctx.board is not None:
            try:
                # One fetch, so the referee sees current data, plus the two
                # settings worth having on screen while troubleshooting: which
                # network the board joined, and what address it is fetching.
                for line in ("F", "G ssid", "G url"):
                    ctx.board.send_raw(line + "\n")
            except Exception:
                pass

    def keys(self, ctx):
        # T is handled by the console itself, so it works in every task; it is
        # listed here because this is where the operator needs it.
        return [("T", "type a command")]

    def commands(self, ctx):
        # Shown while typing. The window has no clipboard, so anything that
        # has to be typed by hand has to be readable off the screen.
        return [("L", "list visible networks"),
                ("G url", "show the address"),
                ("P url <ip:port>", "set the address"),
                ("P ssid <name>", "change network"),
                ("P pass <password>", "and its password"),
                ("W", "reconnect now")]

    def info(self, ctx):
        m = ctx.mission
        lines = [("MODE  Mission 1", AMBER),
                 (f"ssid  {m['ssid']}", WHITE),
                 (f"url   {m['url']}", WHITE),
                 (f"wifi  {m['wifi']}   rssi {m['rssi']}", WHITE),
                 (f"ip    {m['ip']}", WHITE),
                 (f"http  {m['http']}", WHITE)]
        # A scan answers a question that DATA cannot, so while one is on screen
        # it takes the space: you scan when nothing is coming back, and then
        # the list of what the board can see is the thing worth looking at.
        if m["scanning"] and not m["nets"]:
            # First scan of this visit. A rescan keeps the previous list up
            # instead, so the panel never goes blank while the radio works.
            lines.append(("nets  scanning...", AMBER))
        elif m["nets"]:
            # Strongest first: a scan comes back in whatever order the radio
            # found them, and the one worth joining is the one with signal.
            # Capped, or a hall full of phones pushes the panel up into the
            # status stack.
            for i, (rssi, channel, ssid) in enumerate(
                    sorted(m["nets"], key=lambda n: -n[0])[:NETS_SHOWN]):
                label = "nets  " if i == 0 else "      "
                lines.append((f"{label}{rssi:>5} ch{channel:<3} {ssid}", WHITE))
        elif m["scan_count"] is not None and m["scan_count"] <= 0:
            # The scan finished and found nothing, or the radio refused it.
            # Saying so matters: falling back to DATA hides the answer to the
            # question that was asked.
            why = ("0 found - nothing in range" if m["scan_count"] == 0
                   else f"scan failed ({m['scan_count']})")
            lines.append((f"nets  {why}", RED))
        elif m["data"]:
            # The whole string is shown, wrapped rather than cut: this is the
            # line the referee reads, and a truncation would be silent. The
            # firmware already caps it at 180 characters.
            body = m["data"]
            for i in range(0, len(body), 44):
                prefix = "DATA  " if i == 0 else "      "
                lines.append((f"{prefix}{body[i:i + 44]}", GREEN))
        else:
            lines.append(("DATA  (nothing received yet)", GREY))
        return lines


TASKS = [TaskAprilTag, TaskColor, TaskWifi]


# ===========================================================================
#  Console
# ===========================================================================

class Console:

    def __init__(self, pad, board, camera):
        self.pad = pad
        self.board = board
        self.camera = camera
        self.latch = Latch(locked=True)
        self.task = Task()                 # Manual
        self.frames = 0
        self.fps = 0.0
        self._t_fps = time.monotonic()
        self.now = time.monotonic()
        self.last_frame = None
        self.mission = {"wifi": "-", "rssi": "-", "ip": "-", "http": "-",
                        "data": "", "ssid": "-", "url": "-",
                        "nets": [], "scan_new": [], "scanning": False,
                        "scan_count": None}
        self.apriltag = AprilTagScanner(mode="largest", min_hits=3, expected=3)
        self.color = ColorDetector()
        self.mouse = Mouse()
        self.typing = None          # the command being typed, or None
        self.reply = None           # (text, colour) from the board, or None
        self._evidence = 0          # last result sequence saved as a picture
        board.on_line = self._on_board_line

    # ---- board events -------------------------------------------------
    def _on_board_line(self, line: str):
        """Consume the non-JOY lines: ST carries Mission 1 status, DATA the
        payload shown to the referee, and OK/ERR/EV the answer to whatever
        was last sent."""
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
        elif tag == "NET":
            # A scan, sent as BEGIN / one line per network / END. The network
            # name is last because it may contain spaces.
            #
            # Results are collected into a staging list and swapped in at END,
            # never cleared at BEGIN. BEGIN arrives after the radio has already
            # finished scanning, and a scan that returns nothing - or whose
            # lines the board could not queue - would otherwise wipe the list
            # already on screen and put nothing back.
            rest = line[len(parts[0]):].strip()
            if rest == "BEGIN":
                self.mission["scanning"] = True
                self.mission["scan_count"] = None
                self.mission["scan_new"] = []
            elif rest.endswith(" END"):
                try:
                    self.mission["scan_count"] = int(rest.split()[0])
                except (ValueError, IndexError):
                    self.mission["scan_count"] = None
                self.mission["nets"] = self.mission["scan_new"]
                self.mission["scanning"] = False
            else:
                rssi, _, tail = rest.partition(" ")
                channel, _, ssid = tail.partition(" ")
                try:
                    self.mission["scan_new"].append((int(rssi), channel, ssid))
                except ValueError:
                    pass          # not a line we recognise; nothing to show
        elif tag in ("OK", "ERR", "EV"):
            # Held until the next command replaces it, not faded after a few
            # seconds: an error is something to keep reading.
            self.reply = (line, {"OK": GREEN, "ERR": RED}.get(tag, AMBER))
            # An answer to G or P also says what the board is now set to, and
            # the panel should follow it. Taken as the rest of the line rather
            # than as a token, because an SSID may contain spaces.
            m = REPLY_RE.match(line)
            if m and m.group(1) in self.mission:
                self.mission[m.group(1)] = m.group(2)

    # ---- typing a command ---------------------------------------------
    def _key(self, key) -> bool:
        """One keystroke from the window. False means quit."""
        if self.typing is not None:
            # While a command is being typed every key belongs to it,
            # including q and Esc - otherwise there is no way to type a Wi-Fi
            # password that contains one.
            self._typing_key(key)
            return True
        if key in (ord("q"), 27):
            return False
        if 32 <= key < 127:
            ch = chr(key).lower()
            if ch == "t":
                self.start_typing()
            else:
                self.task.on_key(ch, self)
        return True

    def start_typing(self):
        self.typing = ""

    def _typing_key(self, key):
        """One keystroke while a command is being typed.

        Case is preserved, unlike the task keys below: the value of P ssid is
        a Wi-Fi name, and those are case-sensitive.
        """
        if key in (13, 10):                      # Enter sends
            text, self.typing = self.typing, None
            if text.strip():
                self.send_command(text)
        elif key == 27:                          # Esc abandons
            self.typing = None
        elif key in (8, 127):                    # Backspace
            self.typing = self.typing[:-1]
        elif 32 <= key < 127 and len(self.typing) < COMMAND_MAX:
            self.typing += chr(key)

    def send_command(self, text: str):
        text = text.strip()
        if not text:
            return
        # R is reset-everything: neutral, the three trims and fsMs go with the
        # Wi-Fi settings. One keystroke away from the command line is too
        # close for that, and the trims were calibrated on a bench.
        if text.upper().startswith("R") and (len(text) == 1 or text[1] == " "):
            self.reply = ("R disabled - it wipes trims and neutral too", RED)
            print("R refused: it resets every parameter, not just the Wi-Fi")
            return
        try:
            self.board.send_raw(text + "\n")
        except Exception as exc:
            self.reply = (f"send failed: {exc}", RED)

    # ---- task switching -----------------------------------------------
    def _switch(self, task_cls):
        """Select a task, or go back to Manual if it is already active."""
        if type(self.task) is task_cls:
            new = Task()
        else:
            new = task_cls()
        self.task.leave(self)
        self.task = new
        # Each task counts its own results from zero, so start the saved-shot
        # sequence from zero with it.
        self._evidence = 0
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
        else:
            dot, link_txt = RED, "NO PAD"

        if not self.pad.connected:
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
        # A black picture with no explanation is the worst failure mode: the
        # operator cannot tell a dead camera from a dark pool.
        if self.camera.lost:
            _, py, _, ph = draw_panel(img, 16, py + ph + 4, [("NO CAMERA", RED)],
                                      anchor="tl", fs=0.5)
        _, py, _, ph = draw_panel(img, 16, py + ph + 4,
                                  [(f"{self.fps:4.1f} fps", GREY)],
                                  anchor="tl", fs=0.42, pad=6)

        # Whatever is being typed, or the board's answer to the last command.
        # It sits in this stack rather than over the picture: it is status, and
        # it must not cover the thing the operator is steering by.
        if self.typing is not None:
            # Keep the tail while typing: the cursor is where you are looking.
            shown = fit_text(self.typing, img, 0.5, keep_head=False)
            draw_panel(img, 16, py + ph + 4, [(f"> {shown}_", WHITE)],
                       anchor="tl", fs=0.5)
        elif self.reply is not None:
            # Keep the head: OK or ERR is the part that says whether it worked.
            draw_panel(img, 16, py + ph + 4,
                       [(fit_text(self.reply[0], img, 0.5), self.reply[1])],
                       anchor="tl", fs=0.5)


    def _draw_motors(self, img, telem):
        """Throttle panel: one row per thruster, each with a centred gauge.

        Throttle, not pulse width. 1718 is what the ESC is given; -36% is what
        the operator can act on. The gauge is centred on zero - neutral in the
        middle, full reverse at the left edge, full ahead at the right - so the
        direction reads off the shape without parsing the sign.

        Hand-rolled rather than using draw_panel, because draw_panel places
        text and gauges in separate bands and this needs a gauge beside each
        row.
        """
        if telem is None:
            draw_panel(img, img.shape[1] - 16, 16, [("waiting for board", GREY)],
                       anchor="tr", fs=0.5)
            return

        fs, pad, gap = 0.5, 9, 6
        rows = motor_rows(telem)

        label_w, label_h = _text_wh(f"{'DOWN':<4}{'+100%':>7}", fs)
        line_h = label_h + gap
        bar_w, bar_h = 170, label_h - 6

        w = pad * 2 + label_w + 14 + bar_w
        h = pad * 2 + line_h * len(rows)
        x, y = _panel_box(img, img.shape[1] - 16, 16, w, h, "tr")

        ty = y + pad
        for name, value, shown in rows:
            value = max(-1.0, min(1.0, value))
            pct = int(round(value * 100))
            # L and R carry their sign; the vertical row's label already says
            # UP or DOWN, and "DOWN -40%" reads badly. Neutral is written
            # without a sign - "+0%" reads like a mistake.
            if shown != "signed" or pct == 0:
                text = f"{abs(pct)}%"
            else:
                text = f"{pct:+d}%"
            cv2.putText(img, f"{name:<4}{text:>7}", (x + pad, ty + label_h - gap),
                        FONT, fs, WHITE, THICKNESS, cv2.LINE_AA)

            bx = x + pad + label_w + 14
            by = ty + 3
            mid = bx + bar_w // 2
            cv2.rectangle(img, (bx, by), (bx + bar_w, by + bar_h), DARK, -1)
            reach = int((bar_w / 2.0) * abs(value))
            if value >= 0.0:
                cv2.rectangle(img, (mid, by), (mid + reach, by + bar_h),
                              AMBER, -1)
            else:
                cv2.rectangle(img, (mid - reach, by), (mid, by + bar_h),
                              AMBER, -1)
            # The centre tick stays visible when the bar is empty, so neutral
            # is a place on the gauge and not just an absence of colour.
            cv2.line(img, (mid, by), (mid, by + bar_h), GREY, 1)
            ty += line_h

    def _draw_hints(self, img):
        """The bottom-right corner: the controller map, and just above it the
        keyboard shortcuts for whatever task is active.

        Both are titled, because the operator has to know which device each
        letter belongs to: C is a button on one and a key on the other, and R
        appears only on the keyboard.

        While a command is being typed they are replaced by the board
        commands. That is what the operator needs at that moment, every key
        belongs to the command line anyway so the key hints would be
        misleading, and it avoids the two panels growing into each other -
        the window has no clipboard, so the commands have to be on screen.
        """
        if self.typing is not None:
            rows = self.task.commands(self)
            if rows:
                width = max(len(c) for c, _ in rows)
                draw_panel(img, img.shape[1] - 16, img.shape[0] - 16,
                           [(f"{c:<{width}}  {d}", GREY) for c, d in rows],
                           anchor="br", fs=0.5, title="BOARD COMMANDS")
                return

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
        locked = self.latch.locked
        lines.append((f"{btn_name(btn):<4} Lock     "
                      f"[{'LOCKED' if locked else 'UNLOCK'}]",
                      GREY if locked else GREEN))
        _, py, _, _ = draw_panel(img, img.shape[1] - 16, img.shape[0] - 16,
                                 lines, anchor="br", fs=0.5, title="PAD")

        keys = [("Q Esc", "quit")] + list(self.task.keys(self))
        draw_panel(img, img.shape[1] - 16, py - 6, self._key_lines(keys),
                   anchor="br", fs=0.44, pad=7, title="KEYBOARD")

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

    def snapshot(self, frame, label="rov"):
        """Write one frame to host/snapshots/.

        Milliseconds are in the name as well as seconds: two captures half a
        second apart would otherwise overwrite each other, and the second one
        is the one that matters.
        """
        folder = pathlib.Path(__file__).resolve().parent / "snapshots"
        folder.mkdir(parents=True, exist_ok=True)
        now = time.time()
        stamp = (time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
                 + f"-{int(now * 1000) % 1000:03d}")
        path = folder / f"{label}-{stamp}.png"
        if cv2.imwrite(str(path), frame):
            print(f"saved {path}")
        else:
            print(f"could not write {path}")

    def _save_evidence(self, img):
        """Keep the picture the operator was shown, once per new result.

        The frame passed in is the finished OSD, not the camera view: the
        point of keeping it is that it carries the panel the referee reads.
        """
        seq = self.task.evidence_seq()
        if seq and seq != self._evidence:
            self._evidence = seq
            self.snapshot(img, self.task.name.lower())

    def render(self, telem, en, frame=None):
        """Produce one finished OSD frame. `frame` overrides the camera, which
        is how a caller can draw a specific picture without owning the device."""
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
        # Callbacks only fire while waitKey is running, which it is once per
        # loop, so the pointer stays current enough for a deliberate press.
        cv2.setMouseCallback(WINDOW, self.mouse.on_event)

        try:
            while True:
                now = time.monotonic()
                self.pad.pump()

                if self.pad.connected:
                    state = self.pad.read()
                    nx, ny, vz = state.nx, state.ny, state.vz
                    key_down, pressed = state.deadman, state.pressed
                    self._handle_buttons(pressed, prev_buttons)
                    prev_buttons = pressed
                else:
                    nx, ny, vz, key_down = 0.0, 0.0, 0.0, False
                    prev_buttons = []

                if not self.pad.connected:
                    self.latch.force_lock()
                    en = False
                else:
                    en = self.latch.update(key_down)

                if now - t_last >= period:
                    t_last = now
                    try:
                        self.board.send(nx, ny, vz, en)
                        sent += 1
                    except Exception as exc:
                        print(f"serial write failed: {exc}")
                        return 2

                self.board.poll()
                telem = self.board.telem
                if telem is None and not warned and now - t_start > 4.0:
                    warned = True
                    print("No telemetry from the board yet. Check it is "
                          "powered, flashed with the ROV firmware, and that "
                          f"{self.board.port} is the right port.")

                img = self.render(telem, en)
                self._save_evidence(img)

                self.frames += 1
                if now - self._t_fps >= 0.5:
                    self.fps = self.frames / (now - self._t_fps)
                    self.frames = 0
                    self._t_fps = now

                cv2.imshow(WINDOW, img)
                if self.color.show_masks and self.last_frame is not None:
                    cv2.imshow(MASKS_WINDOW, np.hstack(list(
                        masks(self.last_frame, self.color.ranges).values())))

                if not self._key(cv2.waitKey(1) & 0xFF):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()
            self.camera.release()

        print("\nExiting (the board returns to neutral on link timeout).")
        return 0


# ===========================================================================
#  Entry point
# ===========================================================================

def main() -> int:
    p = argparse.ArgumentParser(
        description="UWR ROV operator console: controller, camera and a DJI-style OSD")
    p.add_argument("--port", help="serial device (auto-detected if omitted)")
    p.add_argument("--deadman", type=int, default=DEFAULT_DEADMAN_BUTTON,
                   help="safety key button number "
                        f"(default {DEFAULT_DEADMAN_BUTTON}, which is Y)")
    p.add_argument("--camera", type=int, default=CAM_INDEX,
                   help="camera index to prefer (others are tried if it is not "
                        "there; --list-cameras says which are)")
    p.add_argument("--list-cameras", action="store_true",
                   help="probe camera indices and exit")
    p.add_argument("--axis-x", type=int, default=0, help="stick X axis number")
    p.add_argument("--axis-y", type=int, default=1, help="stick Y axis number")
    p.add_argument("--invert-x", action="store_true", help="negate X")
    p.add_argument("--invert-y", action="store_true", help="negate Y")
    args = p.parse_args()

    if args.list_cameras:
        return list_cameras()

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

    pad.deadman_button = args.deadman
    print(f"safety key = {btn_name(pad.deadman_button)} "
          f"(button {pad.deadman_button}). Starts locked.")

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
            print("No serial ports found. Is the board plugged in?")
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

    camera = Camera(args.camera)
    if not camera.ok:
        print(f"No camera delivered frames (tried from index {args.camera}). "
              "Run --list-cameras to see what is attached.")
        return 2
    print(f"camera: index {camera.index}")

    console = Console(pad, board, camera)
    try:
        return console.run()
    finally:
        board.close()


if __name__ == "__main__":
    raise SystemExit(main())
