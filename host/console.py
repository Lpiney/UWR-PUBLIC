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
    DEFAULT_DEADMAN_BUTTON, Latch, Board, Pad,
    find_ports, learn_deadman, missing_deps, pick_port,
)

# ===========================================================================
#  Configuration
# ===========================================================================

WINDOW = "UWR ROV Console"

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

# SDL's standard gamepad layout. Only used to label the on-screen key map; an
# unknown number is printed as-is rather than guessed at.
BTN_NAMES = {0: "A", 1: "B", 2: "X", 3: "Y", 4: "LB", 5: "RB",
             6: "Back", 7: "Start", 8: "LStick", 9: "RStick"}


def btn_name(n) -> str:
    return BTN_NAMES.get(n, f"btn{n}")

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

        if not synthetic:
            cap = cv2.VideoCapture(index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_GAIN, CAM_GAIN)
            if CAM_EXPOSURE:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
                cap.set(cv2.CAP_PROP_EXPOSURE, CAM_EXPOSURE)
            if cap.isOpened():
                self.cap = cap

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

    def info(self, ctx) -> list:
        """Lines for the bottom-left panel."""
        return [("MODE  Manual", AMBER),
                ("no task selected", GREY)]


class TaskAprilTag(Task):
    name = "AprilTag"
    button = BTN_APRILTAG
    hint = "AprilTag"

    def info(self, ctx):
        return [("MODE  AprilTag", AMBER),
                ("detector not wired in yet", GREY)]


class TaskColor(Task):
    name = "Color"
    button = BTN_COLOR
    hint = "Colour"

    def info(self, ctx):
        return [("MODE  Colour", AMBER),
                ("detector not wired in yet", GREY)]


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
        self.mission = {"wifi": "-", "rssi": "-", "ip": "-", "http": "-",
                        "data": ""}
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

    def _handle_buttons(self, pressed: list, prev: list):
        for task_cls in TASKS:
            b = task_cls.button
            if b is None or b >= len(pressed):
                continue
            if pressed[b] and (b >= len(prev) or not prev[b]):
                self._switch(task_cls)

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
        """Key map, showing which physical button does what and what is active."""
        lines = []
        for task_cls in TASKS:
            active = type(self.task) is task_cls
            lines.append((f"{btn_name(task_cls.button):<4} {task_cls.hint:<9}"
                          f"[{'ON' if active else '  '}]",
                          AMBER if active else GREY))

        btn = self.pad.deadman_button
        if not self.use_deadman:
            lines.append((f"{'':<4} no safety key", GREY))
        else:
            locked = self.latch.locked
            lines.append((f"{btn_name(btn):<4} Lock     "
                          f"[{'LOCKED' if locked else 'UNLOCK'}]",
                          GREY if locked else GREEN))
        draw_panel(img, img.shape[1] - 16, img.shape[0] - 16, lines,
                   anchor="br", fs=0.5)

    def draw_osd(self, img, telem, link, en):
        self._draw_status(img, telem, link, en)
        self._draw_motors(img, telem)
        draw_panel(img, 16, img.shape[0] - 16, self.task.info(self),
                   anchor="bl", fs=0.5)
        self._draw_hints(img)

    def render(self, telem, en, frame=None):
        """Produce one finished OSD frame. Used by the live loop and by
        --render-preview, so the preview can never drift from the real thing."""
        if frame is None:
            frame = self.camera.read()
        if frame is None:
            frame = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), np.uint8)
        img = frame.copy()
        link = telem is not None
        self.draw_osd(img, telem, link, en)
        return img

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
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
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
        ("aprilag", telem, True),
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
    p.add_argument("--deadman", type=int, default=None,
                   help="safety key button number; the script asks you to press it "
                        f"if omitted (falls back to {DEFAULT_DEADMAN_BUTTON})")
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
    elif args.deadman is None:
        print("Press the button you want as the safety key (RB is a good choice).")
        idx, why = learn_deadman(lambda: (pad.pump(), pad.pressed_buttons())[1])
        pad.deadman_button = DEFAULT_DEADMAN_BUTTON if idx is None else idx
        print(why + f" (safety key = button {pad.deadman_button})")
    else:
        pad.deadman_button = args.deadman

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
