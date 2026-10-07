#!/usr/bin/env python3
"""pad_bridge - Xbox controller -> laptop -> serial -> ESP32.

Data path:

    Xbox controller --USB/Bluetooth--> laptop (this script) --serial--> ESP32
                                         ^                                |
                                         +---- telemetry (L/R, state) ----+

Every 20 ms this script sends one line:

    CMD nx=+0.42 ny=-0.17 en=1
      nx/ny   left stick, -1..+1 (+x is right, +y is forward)
      en      1 = unlocked (motors may drive), 0 = locked -> board forces neutral

The safety key is a toggle (locked by default): press once to unlock, press
again to lock. Loss of the controller re-locks it automatically.

Both safety nets live in the firmware, so they still work if this script dies:
    1) no valid command for LINK_TIMEOUT_MS -> forced neutral
    2) en=0 -> forced neutral

Usage
    pip install pygame pyserial

    # Identify axes and find which button should be the safety key
    python3 pad_bridge.py --show-input

    # Normal run. The safety key is Y and starts locked; press it once to
    # unlock, again to lock. Loss of the controller re-locks automatically.
    python3 pad_bridge.py

    # First debugging session, no lock wanted
    python3 pad_bridge.py --no-deadman

    # No board attached, controller only
    python3 pad_bridge.py --no-serial
"""

from __future__ import annotations

import argparse
import sys
import time

# ---- Axis convention (must match the firmware) ----
# Target: stick right = nx > 0, stick up = ny > 0.
#
# These two coefficients were measured on the real hardware, not read from a
# specification. On an Xbox Series X controller under macOS, SDL reports axis 0
# positive to the right and axis 1 positive *downward*.
#
# Do not trust documentation for this, and do not trust another platform.
# The only reliable check is the "will send" column of --show-input: pushing up
# must show a positive Y, pushing right a positive X. Change these coefficients
# if it does not, or pass --invert-x / --invert-y for a temporary override.
#
# Never change the firmware signs instead: those belong to the stick input.
PAD_X_SIGN = +1.0
PAD_Y_SIGN = -1.0

DEFAULT_AXIS_X = 0
DEFAULT_AXIS_Y = 1
DEFAULT_DEADMAN_BUTTON = 3      # Y, under SDL's standard gamepad mapping
CMD_HZ = 50

PULSE_NEUTRAL = 1500

# SDL's standard gamepad layout. Used only to label buttons on screen and in
# messages; an unknown number is printed as-is rather than guessed at.
BTN_NAMES = {0: "A", 1: "B", 2: "X", 3: "Y", 4: "LB", 5: "RB",
             6: "Back", 7: "Start", 8: "LStick", 9: "RStick"}


def btn_name(n) -> str:
    return BTN_NAMES.get(n, f"btn{n}")


# ===========================================================================
#  Controller
# ===========================================================================

class Pad:
    """Reads the controller. Axis and button numbers are confirmed on site
    with --show-input."""

    def __init__(self, axis_x, axis_y, deadman_button, invert_x, invert_y):
        self.axis_x = axis_x
        self.axis_y = axis_y
        self.deadman_button = deadman_button
        self.invert_x = invert_x
        self.invert_y = invert_y
        self.js = None
        self.name = ""

    def open(self) -> bool:
        import pygame
        pygame.init()
        pygame.joystick.init()
        self._pygame = pygame
        if pygame.joystick.get_count() == 0:
            return False
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.name = self.js.get_name()
        return True

    def pump(self):
        """Handle hot-plug, so unplugging and replugging the controller does
        not need a restart.

        Two traps, both hit in practice:
        1. The device index attribute of JOYDEVICEADDED differs between pygame
           versions (2.6 has device_index, older ones device). Reading one
           name unconditionally raises AttributeError the moment a controller
           is plugged in, killing the script.
        2. This runs on competition day, so no single bad event may take the
           bridge down. Each event is handled inside its own try.
        """
        pygame = self._pygame
        try:
            events = pygame.event.get()
        except Exception:
            return
        for ev in events:
            try:
                if ev.type == pygame.JOYDEVICEREMOVED:
                    self.js = None
                elif ev.type == pygame.JOYDEVICEADDED:
                    idx = getattr(ev, "device_index", None)
                    if idx is None:
                        idx = getattr(ev, "device", 0)
                    self.js = pygame.joystick.Joystick(int(idx))
                    self.js.init()
            except Exception:
                continue        # one bad event must not affect the others

    @property
    def connected(self) -> bool:
        return self.js is not None

    def read(self):
        """Return (nx, ny, deadman, raw_axes, raw_button_state)."""
        if self.js is None:
            return 0.0, 0.0, False, [], False
        axes = [self.js.get_axis(i) for i in range(self.js.get_numaxes())]

        def axis(i):
            return axes[i] if 0 <= i < len(axes) else 0.0

        nx = axis(self.axis_x) * PAD_X_SIGN * (-1.0 if self.invert_x else 1.0)
        ny = axis(self.axis_y) * PAD_Y_SIGN * (-1.0 if self.invert_y else 1.0)
        nx = max(-1.0, min(1.0, nx))
        ny = max(-1.0, min(1.0, ny))

        nb = self.js.get_numbuttons()
        pressed = [self.js.get_button(i) for i in range(nb)]
        idx = self.deadman_button
        # idx may be None before the safety key has been learned.
        deadman = bool(pressed[idx]) if (idx is not None and 0 <= idx < nb) else False
        return nx, ny, deadman, axes, pressed

    def pressed_buttons(self) -> set:
        """Set of button numbers currently down."""
        if self.js is None:
            return set()
        return {i for i in range(self.js.get_numbuttons()) if self.js.get_button(i)}

    def snapshot(self):
        """Raw axes and buttons, unprocessed, for --show-input."""
        if self.js is None:
            return []
        axes = [self.js.get_axis(i) for i in range(self.js.get_numaxes())]
        buttons = [i for i in range(self.js.get_numbuttons()) if self.js.get_button(i)]
        return axes, buttons


# ===========================================================================
#  Safety key
# ===========================================================================

class Latch:
    """Toggle safety key: locked by default, press to unlock, press to lock.

    A hold-to-drive key was rejected because it ties up a hand for the whole
    run, which on site turns into "I thought I was holding it". A toggle needs
    one press, and the state is visible both here and on the board.

    The fallback is stricter than a hold key: losing the controller re-locks.
    """

    def __init__(self, locked: bool = True):
        self.locked = locked
        self._was_down = False
        self.changed = None        # True if this update() flipped the state

    def update(self, key_down: bool) -> bool:
        """Feed in "is the safety key down right now", get back "may drive".

        Only the rising edge flips the state, so holding the key down does not
        toggle repeatedly.
        """
        self.changed = None
        down = bool(key_down)
        if down and not self._was_down:
            self.locked = not self.locked
            self.changed = self.locked
        self._was_down = down
        return not self.locked

    def force_lock(self):
        """Lock unconditionally (controller lost, script exiting)."""
        if not self.locked:
            self.locked = True
            self.changed = True


# ===========================================================================
#  Serial link to the board
# ===========================================================================

# Ports that are never the board. On macOS these are system and Bluetooth
# entry points; on Windows a motherboard serial port is often present too and
# its description reads "Communications Port".
_NOT_A_BOARD = ("debug-console", "bluetooth", "communications port")
# USB-to-serial bridge chips: FTDI, Silicon Labs, WCH, Prolific.
#
# Matching looks at the device name AND the description, because the two
# platforms put the information in different places. On macOS the device name
# carries the chip ("cu.usbserial-A5069RR4"); on Windows the device is just
# "COM5" and the only clue is the description, which reads "USB Serial Port"
# (FTDI's driver) or "USB-SERIAL CH340" (WCH's).
_BRIDGE_LIKE = ("usbserial", "slab", "wch", "ftdi", "cp210", "ch34", "ch910",
                "ch341", "prolific", "ft232", "usb serial port")
# Other USB serial ports, including the ESP32's own USB-CDC interface, which
# Windows describes as "USB Serial Device".
_USB_LIKE = ("usbserial", "usbmodem", "slab", "wch", "usb serial device",
             "usb-serial", "usb")


class Telemetry:
    """One parsed JOY line from the board."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.mag = 0.0
        self.in_dead = True
        self.ready = False
        self.l_us = PULSE_NEUTRAL
        self.r_us = PULSE_NEUTRAL
        self.tl_us = PULSE_NEUTRAL
        self.tr_us = PULSE_NEUTRAL
        self.en = False        # safety key unlocked on the host side
        self.link = False      # board is receiving commands
        self.seen = False


def parse_joy(line: str, t: Telemetry) -> bool:
    """Parse one JOY telemetry line.

        JOY X=.. Y=.. MAG=.. DEAD|OUT READY=0|1 L=.. R=.. TL=.. TR=.. EN=0|1 LINK=0|1

    Deliberately lenient: unknown tokens are ignored, so new firmware fields
    cannot break this parser.
    """
    parts = line.split()
    if not parts or parts[0].upper() != "JOY":
        return False

    kv: dict[str, str] = {}
    bare: set[str] = set()
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            kv[k.upper()] = v
        else:
            bare.add(p.upper())

    def num(key, default):
        try:
            return int(kv[key])
        except (KeyError, ValueError):
            return default

    def flt(key, default):
        try:
            return float(kv[key])
        except (KeyError, ValueError):
            return default

    t.x = flt("X", t.x)
    t.y = flt("Y", t.y)
    t.mag = flt("MAG", t.mag)
    t.in_dead = "DEAD" in bare
    t.ready = num("READY", 1) == 1
    t.l_us = num("L", PULSE_NEUTRAL)
    t.r_us = num("R", PULSE_NEUTRAL)
    t.tl_us = num("TL", t.l_us)
    t.tr_us = num("TR", t.r_us)
    t.en = num("EN", 0) == 1
    t.link = num("LINK", 0) == 1
    t.seen = True
    return True


def find_ports() -> list:
    """List serial ports as (device, description) pairs.

    The description is carried because it is the only thing that identifies a
    port on Windows, where the device is just "COM5".
    """
    try:
        from serial.tools import list_ports as slp
        return [(p.device, p.description or "") for p in slp.comports()]
    except Exception:
        return []


def pick_port(ports: list):
    """Pick the board out of a list of ports, or return None.

    Accepts either (device, description) pairs from find_ports() or bare
    device-name strings, so it stays a pure function and the self test can
    feed it realistic lists directly.

    Auto-detection exists because every platform's port list is full of
    entries that are not the board - Bluetooth devices and system debug ports
    on macOS, motherboard serial ports and virtual COM ports on Windows.
    """
    norm = [(p, "") if isinstance(p, str) else (p[0], p[1] or "") for p in ports]

    cand, bridge, usb_like = [], [], []
    for dev, desc in norm:
        hay = f"{dev} {desc}".lower()
        if any(s in hay for s in _NOT_A_BOARD):
            continue
        cand.append(dev)
        if any(k in hay for k in _BRIDGE_LIKE):
            bridge.append(dev)
        elif any(k in hay for k in _USB_LIKE):
            usb_like.append(dev)

    # Prefer the bridge chip. The firmware answers on both the native USB port
    # and UART0, but the bridge is the more predictable of the two.
    if len(bridge) == 1:
        return bridge[0], cand
    if len(usb_like) == 1:
        return usb_like[0], cand
    if len(cand) == 1:
        return cand[0], cand
    return None, cand


def is_windows_com(name: str) -> bool:
    return name.upper().startswith("COM") and name[3:].isdigit()


def diagnose_no_telemetry(port: str) -> str:
    """Explain why a port is open but silent.

    The old behaviour was to sit on "waiting for telemetry" forever. Tools that
    know the likely cause and stay quiet about it have cost this project a lot
    of time.
    """
    p = (port or "").lower()
    looks_like_board = (is_windows_com(port or "")
                        or any(k in p for k in _BRIDGE_LIKE)
                        or "usbmodem" in p)
    if looks_like_board:
        return ("Port looks plausible but no telemetry arrived. Check: is the board "
                "powered? Is the ROV firmware flashed (it emits a JOY line every "
                "50 ms)? Is the port held open by a serial monitor? On Windows, is "
                "the USB-serial driver installed (FTDI VCP, or WCH CH34x)?")
    return ("No telemetry: confirm the board is powered and running the ROV "
            "firmware, which emits a JOY line continuously.")


def cmd_line(nx: float, ny: float, en: bool) -> str:
    """The line the laptop sends. The format must match parseCmd() in the
    firmware's comms.h."""
    return f"CMD nx={nx:+.3f} ny={ny:+.3f} en={1 if en else 0}\n"


class Board:
    """Serial link: send commands, receive telemetry."""

    def __init__(self, port):
        self.port = port
        self.ser = None
        self.buf = b""
        self.telem = None          # most recent successfully parsed telemetry
        self.last_rx = 0.0
        self.bad_lines = 0
        # Optional hook for lines that are not JOY telemetry (OK/ERR/EV/DATA/ST).
        # Set by the console; left as None here so the bridge stays standalone.
        self.on_line = None

    def open(self):
        import serial
        # Opening the port may reset the board once, via the DTR/RTS auto-reset
        # circuit. That is normal: it re-runs setup() and starts sending
        # telemetry a few seconds later.
        self.ser = serial.Serial(self.port, 115200, timeout=0)
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass

    def send(self, nx, ny, en):
        self.ser.write(cmd_line(nx, ny, en).encode("ascii"))

    def send_raw(self, text: str):
        """Send one protocol line by hand, for commands other than CMD.
        The caller supplies the newline."""
        self.ser.write(text.encode("ascii"))

    def poll(self):
        """Drain whatever is readable and update self.telem."""
        chunk = self.ser.read(4096)
        if not chunk:
            return
        self.buf += chunk
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            t = Telemetry()
            if parse_joy(line, t):
                self.telem = t
                self.last_rx = time.monotonic()
            else:
                self.bad_lines += 1
                if self.on_line is not None:
                    self.on_line(line)

    def close(self):
        if self.ser is not None:
            try:
                self.send(0.0, 0.0, False)      # leave the board neutral
                time.sleep(0.05)
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass


# ===========================================================================
#  Modes
# ===========================================================================

def show_input(pad: Pad) -> int:
    """Read the controller only: print both the raw axes and the values that
    would be sent, side by side, so a wrong sign is obvious at a glance.
    """
    print("Input diagnostic: controller only, no serial, nothing sent.")
    print("Do three things, holding each for 2 seconds:")
    print("  1) push the left stick fully up")
    print("  2) push it fully down")
    print("  3) push it fully right")
    print("")
    print("Watch the 'will send' column:")
    print("  up -> Y must be positive; down -> Y negative; right -> X positive.")
    print("  If any of those is wrong, flip the matching sign coefficient at the")
    print("  top of this file (PAD_X_SIGN / PAD_Y_SIGN), or pass --invert-x /")
    print("  --invert-y for a temporary override.")
    print("Press the buttons too; their numbers appear on the right.\n")
    while True:
        pad.pump()
        if not pad.connected:
            print("\rwaiting for a controller (plug in USB or pair Bluetooth)" + " " * 20,
                  end="", flush=True)
            time.sleep(0.3)
            continue
        nx, ny, _, axes, _ = pad.read()
        buttons = sorted(pad.pressed_buttons())
        moving = " ".join(f"axis{i}={v:+.2f}" for i, v in enumerate(axes) if abs(v) > 0.25)
        line = (f"raw {moving:44s} | will send X{nx:+.2f} Y{ny:+.2f} | "
                f"buttons {buttons if buttons else ''}")
        print(f"\r{line[:170]:170s}", end="", flush=True)
        time.sleep(0.05)


def draw_status(pad, board, nx, ny, en, sent, use_deadman, latched=True):
    pad_txt = "pad ok" if pad.connected else "pad LOST (neutral)"
    if not use_deadman:
        lock_txt = "disabled (always live)"
    elif not pad.connected:
        lock_txt = "LOCKED (pad lost)"
    else:
        lock_txt = "UNLOCKED: may drive" if en else "LOCKED: stick does nothing"

    if board is None:
        board_txt = "no board"
    elif board.telem is None:
        board_txt = "waiting for telemetry (the board resets once on open)"
    else:
        t = board.telem
        age = (time.monotonic() - board.last_rx) * 1000
        board_txt = (f"board L={t.l_us:4d} R={t.r_us:4d} "
                     f"{'LINK ok' if t.link else 'LINK LOST'} "
                     f"{'unlocked' if t.en else 'locked'} "
                     f"({age:.0f} ms ago)")

    line = (f"{pad_txt} | stick X{nx:+.2f} Y{ny:+.2f} | {lock_txt:22s} | "
            f"sent {sent:6d} | {board_txt}")
    print(f"\r{line[:170]:170s}", end="", flush=True)


def run_bridge(pad: Pad, board, use_deadman: bool, period: float) -> int:
    print("Bridge mode: controller -> serial -> board. Ctrl-C to quit.")
    latch = Latch(locked=True)
    if use_deadman:
        print(f"safety key = button {pad.deadman_button} (toggle): currently LOCKED, "
              f"press once to unlock, again to lock.")
        print("Losing the controller re-locks it automatically.")
    if board is None:
        print("--no-serial: no board, controller readings only.")
    time.sleep(0.4)

    sent = 0
    t_last = 0.0
    t_draw = 0.0
    t_start = time.monotonic()
    warned = False
    try:
        while True:
            now = time.monotonic()
            pad.pump()

            if pad.connected:
                nx, ny, key_down, _, _ = pad.read()
            else:
                nx, ny, key_down = 0.0, 0.0, False      # pad lost -> neutral now

            if not use_deadman:
                en = True
            elif not pad.connected:
                latch.force_lock()                      # safety first
                en = False
            else:
                en = latch.update(key_down)
                if latch.changed is not None:
                    print("\n" + ("**LOCKED** (press again to unlock)" if latch.changed
                                  else "**UNLOCKED** (stick is live now; press again to lock)"))

            # Fixed send cadence, independent of the controller's refresh rate,
            # so the board sees a steady 50 Hz.
            if now - t_last >= period:
                t_last = now
                if board is not None:
                    try:
                        board.send(nx, ny, en)
                        sent += 1
                    except Exception as exc:
                        print(f"\nserial write failed: {exc}")
                        return 2

            if board is not None:
                board.poll()
                # Four seconds with no telemetry at all: say why instead of
                # leaving the operator to wait.
                if board.telem is None and not warned and now - t_start > 4.0:
                    warned = True
                    print("\n\n" + diagnose_no_telemetry(board.port))
                    print("(the script keeps running; restart it after fixing the port.)")

            # Sending and drawing are separate concerns: 50 Hz out, 10 Hz on
            # screen is plenty and far easier to read.
            if now - t_draw >= 0.1:
                t_draw = now
                draw_status(pad, board, nx, ny, en, sent, use_deadman,
                            latched=latch.locked)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n\nExiting (the board returns to neutral on link timeout).")
        return 0


def missing_deps():
    """Say which Python to install into. A machine usually has several
    python3 installations, and installing into one while running another is
    the single most common "but I did install it"."""
    print("This Python has no pygame / pyserial.")
    print(f"  current interpreter: {sys.executable}")
    print("  install into that one (same python, note the quotes):")
    print(f'    "{sys.executable}" -m pip install pygame pyserial')
    print("  or run it with the interpreter you already installed into:")
    print("    python3 pad_bridge.py")


# ===========================================================================
#  Self test (no controller, no pygame, no serial)
# ===========================================================================

class _FakeEvent:
    def __init__(self, type_, **kw):
        self.type = type_
        self.__dict__.update(kw)


class _FakeJoystick:
    """Fake controller: axis 0 = stick X, axis 1 = stick Y, button 5 = safety."""

    def __init__(self, idx=0, axes=(0.0, 0.0), buttons=()):
        self.idx = idx
        self.axes = list(axes) + [0.0] * 6
        self.buttons = [1 if i in buttons else 0 for i in range(11)]

    def init(self):
        pass

    def get_name(self):
        return f"FakePad{self.idx}"

    def get_numaxes(self):
        return len(self.axes)

    def get_axis(self, i):
        return self.axes[i]

    def get_numbuttons(self):
        return len(self.buttons)

    def get_button(self, i):
        return self.buttons[i]


class _FakePygame:
    JOYDEVICEADDED = 1536
    JOYDEVICEREMOVED = 1537

    def __init__(self, events=(), joystick=None):
        self._events = list(events)
        self._js = joystick
        self.joystick = _FakePygame._JoystickModule(joystick)

    class _JoystickModule:
        def __init__(self, js):
            self._js = js

        def Joystick(self, idx):          # noqa: N802 (pygame's interface name)
            if self._js is None:
                raise RuntimeError("no controller")
            return self._js

    @property
    def event(self):
        return self

    def get(self):
        ev, self._events = self._events, []
        return ev


def selftest() -> int:
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"  {'ok  ' if cond else 'FAIL'} {name}"
              + (f"   {detail}" if not cond and detail else ""))
        ok = ok and cond

    def make_pad(axes=(0.0, 0.0), buttons=(), **kw):
        p = Pad(kw.get("axis_x", 0), kw.get("axis_y", 1),
                kw.get("deadman", DEFAULT_DEADMAN_BUTTON),
                kw.get("inv_x", False), kw.get("inv_y", False))
        p._pygame = _FakePygame()          # never initialise the real pygame here
        p.js = _FakeJoystick(axes=axes, buttons=buttons)
        p.name = "FakePad"
        return p

    print("pad_bridge self test (no controller / pygame / serial needed)")

    # ---- 1. Hot-plug events ----
    print("\n[1] hot-plug events (this used to crash)")
    probe = _FakeEvent(_FakePygame.JOYDEVICEADDED, device_index=0)
    check("(precondition) such an event really has no .device attribute",
          not hasattr(probe, "device"))
    for attr in ("device_index", "device"):
        p = make_pad()
        p.js = None
        p._pygame = _FakePygame(events=[_FakeEvent(_FakePygame.JOYDEVICEADDED, **{attr: 0})],
                                joystick=_FakeJoystick(0))
        try:
            p.pump()
            check(f"event carrying only {attr} still attaches", p.js is not None)
        except Exception as exc:
            check(f"event carrying only {attr} still attaches", False,
                  f"{type(exc).__name__}: {exc}")

    p = make_pad()
    p.js = None
    p._pygame = _FakePygame(events=[_FakeEvent(_FakePygame.JOYDEVICEADDED, device_index=None)],
                            joystick=_FakeJoystick(0))
    try:
        p.pump()
        check("event carrying neither attribute does not crash", True)
    except Exception as exc:
        check("event carrying neither attribute does not crash", False,
              f"{type(exc).__name__}: {exc}")

    p = make_pad()
    p._pygame = _FakePygame(events=[_FakeEvent(_FakePygame.JOYDEVICEREMOVED, device_index=0)],
                            joystick=_FakeJoystick(0))
    p.pump()
    check("unplugging marks it disconnected", p.js is None and not p.connected)

    p = make_pad()
    p.js = _FakeJoystick(0)
    p._pygame = _FakePygame(events=[_FakeEvent(999, whatever=1)])
    try:
        p.pump()
        check("unknown event types are ignored", True)
    except Exception as exc:
        check("unknown event types are ignored", False, f"{type(exc).__name__}: {exc}")

    # ---- 2. Axis convention: right = +nx, up = +ny ----
    print("\n[2] axis convention - up gives positive ny, right gives positive nx")
    p = make_pad(axes=(0.83, 0.0))
    nx = p.read()[0]
    check("axis0=+0.83 (right) -> nx=+0.83", abs(nx - 0.83) < 1e-6, f"nx={nx}")

    p = make_pad(axes=(0.0, -0.66))     # measured: up is negative on axis 1
    ny = p.read()[1]
    check("axis1=-0.66 (up) -> ny=+0.66", abs(ny - 0.66) < 1e-6, f"ny={ny}")

    p = make_pad(axes=(0.0, 0.66))      # down is positive on the raw axis
    ny = p.read()[1]
    check("axis1=+0.66 (down) -> ny=-0.66", abs(ny + 0.66) < 1e-6, f"ny={ny}")

    # Do not hardcode values here: the base signs above are expected to change
    # with hardware, and a hardcoded test goes red every time they do. Only
    # assert that the override inverts.
    base_nx, base_ny, _, _, _ = make_pad(axes=(0.5, 0.5)).read()
    inv_nx, inv_ny, _, _, _ = make_pad(axes=(0.5, 0.5), inv_x=True, inv_y=True).read()
    check("--invert-x / --invert-y negate the result (regardless of base sign)",
          abs(inv_nx + base_nx) < 1e-6 and abs(inv_ny + base_ny) < 1e-6
          and abs(base_nx) > 0.1,
          f"base nx={base_nx} ny={base_ny} / inverted nx={inv_nx} ny={inv_ny}")

    p = make_pad(axes=(1.5, -1.5))
    nx, ny, _, _, _ = p.read()
    check("readings beyond +-1 are clamped (some controllers overshoot)",
          nx <= 1.0 and ny <= 1.0, f"nx={nx} ny={ny}")

    # ---- 3. Safety key ----
    print("\n[3] safety key")
    p = make_pad(axes=(0.0, 0.0))
    check("not pressed -> en=False", p.read()[2] is False)
    p = make_pad(axes=(0.0, 0.0), buttons=(DEFAULT_DEADMAN_BUTTON,))
    check(f"button {DEFAULT_DEADMAN_BUTTON} down -> en=True", p.read()[2] is True)
    p = make_pad(axes=(0.0, 0.0), buttons=(3,), deadman=99)
    check("out-of-range key number does not crash, reads as not pressed",
          p.read()[2] is False)

    p = make_pad(axes=(0.5, -0.5))
    p.deadman_button = None          # the --show-input path
    try:
        nx, ny, en, _, _ = p.read()
        check("unset safety key (None) does not crash", abs(nx - 0.5) < 1e-6 and en is False,
              f"nx={nx} en={en}")
    except Exception as exc:
        check("unset safety key (None) does not crash", False, f"{type(exc).__name__}: {exc}")

    print("\n[3b] toggle safety key (starts locked / one press flips / pad loss re-locks)")
    la = Latch()
    check("starts locked -> no drive", la.update(False) is False)
    check("one press -> unlocked", la.update(True) is True)
    check("holding it down does not oscillate", la.update(True) is True and not la.changed)
    check("released -> still unlocked (toggle, not hold)", la.update(False) is True)
    check("press again -> locked again", la.update(True) is False)
    check("still locked after release", la.update(False) is False)

    la2 = Latch()
    la2.update(True)
    la2.force_lock()
    check("pad loss re-locks", la2.locked and la2.update(False) is False)
    check("force_lock when already locked reports no event", Latch().update(False) is False)

    # ---- 4. Command format ----
    print("\n[4] command format")
    check("neutral, locked",
          cmd_line(0.0, 0.0, False) == "CMD nx=+0.000 ny=+0.000 en=0\n",
          repr(cmd_line(0.0, 0.0, False)))
    check("forward-right, unlocked",
          cmd_line(0.42, -0.17, True) == "CMD nx=+0.420 ny=-0.170 en=1\n",
          repr(cmd_line(0.42, -0.17, True)))

    # ---- 5. Telemetry parsing ----
    print("\n[5] telemetry parsing")
    t = Telemetry()
    check("a well-formed JOY line parses",
          parse_joy("JOY X=+0.42 Y=-0.17 MAG=0.45 OUT READY=1 L=1718 R=1282 "
                    "TL=1718 TR=1282 EN=1 LINK=1", t) and t.seen)
    check("fields land where they should",
          abs(t.x - 0.42) < 1e-6 and t.l_us == 1718 and t.en and t.link and not t.in_dead)
    check("an unknown extra field does not break it",
          parse_joy("JOY X=0 Y=0 MAG=0 DEAD READY=1 L=1500 R=1500 EN=0 LINK=0 FUTURE=7",
                    Telemetry()))
    check("a non-JOY line is rejected", not parse_joy("ST state=READY wifi=IDLE", Telemetry()))

    # ---- 6. Port picking ----
    print("\n[6] port picking")
    mac_list = ["/dev/cu.debug-console", "/dev/cu.iFLYBUDSPro2",
                "/dev/cu.Bluetooth-Incoming-Port", "/dev/cu.usbserial-A5069RR4"]
    chosen, _ = pick_port(mac_list)
    check("picks the board out of a real macOS list",
          chosen == "/dev/cu.usbserial-A5069RR4", f"chose {chosen}")

    chosen, _ = pick_port(["/dev/cu.debug-console", "/dev/cu.Bluetooth-Incoming-Port"])
    check("with only system/Bluetooth ports it refuses to guess", chosen is None)

    chosen, _ = pick_port(["/dev/cu.usbserial-A", "/dev/cu.usbserial-B"])
    check("two identical-looking candidates -> no guess", chosen is None)

    chosen, _ = pick_port(["/dev/ttyS0"])
    check("a single candidate wins even if the name looks odd", chosen == "/dev/ttyS0")

    two_ports = ["/dev/cu.debug-console", "/dev/cu.usbmodem101", "/dev/cu.usbserial-A5069RR4"]
    chosen, _ = pick_port(two_ports)
    check("bridge chip preferred over the native USB port",
          chosen == "/dev/cu.usbserial-A5069RR4", f"chose {chosen}")

    # Windows: the device is just COMx, so the chip only shows up in the
    # description. These are the strings pyserial actually reports there.
    chosen, _ = pick_port([("COM1", "Communications Port"),
                           ("COM5", "USB Serial Port"),
                           ("COM6", "USB Serial Device")])
    check("Windows: FTDI bridge picked over the motherboard port and the "
          "ESP32's own USB port", chosen == "COM5", f"chose {chosen}")

    chosen, _ = pick_port([("COM3", "Bluetooth Serial Port")])
    check("Windows: a lone Bluetooth port is not the board", chosen is None,
          f"chose {chosen}")

    chosen, _ = pick_port([("COM7", "Silicon Labs CP210x USB to UART Bridge")])
    check("Windows: a CP210x bridge is recognised", chosen == "COM7",
          f"chose {chosen}")

    chosen, _ = pick_port([("COM4", "USB-SERIAL CH340 (COM4)")])
    check("Windows: a CH340 bridge is recognised", chosen == "COM4",
          f"chose {chosen}")

    chosen, _ = pick_port([("COM2", "USB Serial Device")])
    check("Windows: the ESP32's own USB port alone is still usable",
          chosen == "COM2", f"chose {chosen}")

    check("silent port produces an actionable message",
          len(diagnose_no_telemetry("/dev/cu.usbserial-A5069RR4")) > 20)

    # ---- 7. Button labels ----
    print("\n[7] button labels")
    check("SDL numbers map to the expected names",
          btn_name(0) == "A" and btn_name(1) == "B" and btn_name(2) == "X"
          and btn_name(3) == "Y" and btn_name(5) == "RB")
    check("an unknown number is shown as-is, not guessed",
          btn_name(42) == "btn42")
    check("the safety key defaults to Y", DEFAULT_DEADMAN_BUTTON == 3)

    print(f"\n{'self test passed' if ok else 'self test FAILED'}")
    return 0 if ok else 1


# ===========================================================================
#  Entry point
# ===========================================================================

def main() -> int:
    p = argparse.ArgumentParser(
        description="Xbox controller -> laptop -> serial -> ESP32 "
                    "(sends one CMD line every 20 ms)")
    p.add_argument("--port", help="serial device (auto-detected if omitted)")
    p.add_argument("--show-input", action="store_true",
                   help="read the controller only, print axis/button numbers")
    p.add_argument("--no-deadman", action="store_true",
                   help="no safety key required (first debugging only, risky)")
    p.add_argument("--no-serial", action="store_true",
                   help="no board, controller readings only")
    p.add_argument("--deadman", type=int, default=DEFAULT_DEADMAN_BUTTON,
                   help="safety key button number "
                        f"(default {DEFAULT_DEADMAN_BUTTON}, which is Y)")
    p.add_argument("--axis-x", type=int, default=DEFAULT_AXIS_X, help="stick X axis number")
    p.add_argument("--axis-y", type=int, default=DEFAULT_AXIS_Y, help="stick Y axis number")
    p.add_argument("--invert-x", action="store_true", help="negate X (if right reads negative)")
    p.add_argument("--invert-y", action="store_true", help="negate Y (if up reads negative)")
    p.add_argument("--selftest", action="store_true",
                   help="no controller, no serial: test this script's logic only")
    args = p.parse_args()

    if args.selftest:
        return selftest()

    try:
        import pygame  # noqa: F401
    except ImportError:
        missing_deps()
        return 2

    pad = Pad(args.axis_x, args.axis_y, args.deadman, args.invert_x, args.invert_y)
    if not pad.open():
        print("No controller found. Plug in USB or pair Bluetooth, then confirm "
              "with --show-input.")
        return 2
    print(f"controller: {pad.name}")

    if args.show_input:
        return show_input(pad)

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
                print(f"serial: auto-selected {port}"
                      + (f" (from {len(cands)} candidates)" if len(cands) > 1 else ""))
            elif not cands:
                print("No serial ports found. Is the board plugged in? "
                      "Use --no-serial to test the controller alone.")
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
            print("(held by another program? close the serial monitor first)")
            return 2
        print(f"serial: {port} @ 115200")

    try:
        return run_bridge(pad, board, not args.no_deadman, 1.0 / CMD_HZ)
    finally:
        if board is not None:
            board.close()


if __name__ == "__main__":
    raise SystemExit(main())
