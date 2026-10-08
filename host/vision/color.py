"""Colour recognition (Mission 5).

The operator points at a pole and presses a button; the patch under the
pointer decides which colour it is. Nothing is scanned on its own and nothing
is tracked over time, because pointing is already a statement about where to
look - the work left is to decide what is there.

What that leaves out is deliberate. An earlier version carried a
shape-filtered detector, a temporal confirmation window and a white-ball
score, all of which existed to guess where poles might be in an unguided
frame. Once the operator says where to look, guessing is unnecessary, and all
three were removed rather than kept as switches nobody should turn on. See
the git history if one of them is ever wanted back.
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

COLORS = {"R": (0, 0, 255), "Y": (0, 220, 255), "B": (255, 120, 0)}
NAMES = {"R": "RED", "Y": "YELLOW", "B": "BLUE"}

# OpenCV hue is 0-179, so red wraps around both ends and needs two bands.
DEFAULT_RANGES = {
    "R": [[0, 12, 85, 255, 45, 255], [168, 179, 85, 255, 45, 255]],
    "Y": [[17, 38, 80, 255, 50, 255]],
    "B": [[90, 135, 75, 255, 40, 255]],
}

CONFIG_PATH = Path(__file__).resolve().parent.parent / "local-config.json"

# Sampling around the pointer. A square patch rather than a single pixel: one
# pixel can land on a highlight or a shadow and read as anything, and a pole
# is far wider than a pixel anyway.
SAMPLE_RADIUS = 7           # half-width, so 15x15 pixels
CALIBRATE_RADIUS = 15       # more samples, for stable thresholds

# A patch has to be mostly coloured, and mostly one colour, to be an answer.
# Without the first test a grey pool floor or a white highlight would come
# back as whichever band its noise happened to fall in. The second is set
# well above half so that pointing at the seam between two poles reports
# nothing rather than whichever one happens to cover a few more pixels.
MIN_COLOURED_SHARE = 0.4
MIN_MATCH_SHARE = 0.6


def validate_ranges(ranges):
    if set(ranges) != set(COLORS):
        raise ValueError("the configuration must define R, Y and B")
    for bands in ranges.values():
        if not bands:
            raise ValueError("a colour threshold list cannot be empty")
        for band in bands:
            if len(band) != 6 or any(type(v) is not int for v in band):
                raise ValueError("each band needs six ints: Hlo Hhi Slo Shi Vlo Vhi")
            for low, high, limit in zip(band[::2], band[1::2], (179, 255, 255)):
                if not 0 <= low <= high <= limit:
                    raise ValueError("HSV threshold out of range or inverted")


def load_config(path=CONFIG_PATH):
    """Read the calibrated bands, falling back to the defaults.

    The file used to be {"settings": ..., "ranges": ...}; only the ranges
    still matter, and a file in the old shape is read without complaint.
    """
    path = Path(path)
    if not path.exists():
        return json.loads(json.dumps(DEFAULT_RANGES))
    data = json.loads(path.read_text(encoding="utf-8"))
    ranges = data.get("ranges", data)
    validate_ranges(ranges)
    return ranges


def save_config(ranges, path=CONFIG_PATH):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"ranges": ranges}, indent=2) + "\n",
                    encoding="utf-8")


def color_mask(hsv, bands):
    """Pixels matching any of one colour's bands, with speckle removed."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for h0, h1, s0, s1, v0, v1 in bands:
        mask |= cv2.inRange(hsv, (h0, s0, v0), (h1, s1, v1))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def box_around(x, y, radius, shape):
    """A square box centred on (x, y), clipped to the frame."""
    height, width = shape[:2]
    x0, y0 = max(0, int(x) - radius), max(0, int(y) - radius)
    x1 = min(width, int(x) + radius + 1)
    y1 = min(height, int(y) + radius + 1)
    return x0, y0, max(0, x1 - x0), max(0, y1 - y0)


def sample_at(frame, x, y, ranges, radius=SAMPLE_RADIUS):
    """Which of R/Y/B is under the pointer, or None if none of them is."""
    x0, y0, w, h = box_around(x, y, radius, frame.shape)
    if w <= 0 or h <= 0:
        return None
    hsv = cv2.cvtColor(frame[y0:y0 + h, x0:x0 + w], cv2.COLOR_BGR2HSV)
    total = w * h

    coloured = cv2.countNonZero(cv2.inRange(hsv, (0, 60, 35), (179, 255, 255)))
    if coloured < total * MIN_COLOURED_SHARE:
        return None

    best, best_share = None, 0.0
    for color, bands in ranges.items():
        share = cv2.countNonZero(color_mask(hsv, bands)) / total
        if share > best_share:
            best, best_share = color, share
    return best if best_share >= MIN_MATCH_SHARE else None


def masks(frame, ranges):
    """The per-colour masks for the whole frame, for the debug window."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return {color: color_mask(hsv, bands) for color, bands in ranges.items()}


def calibrate(frame, box):
    """Derive HSV thresholds from a sample of bare pole.

    Hue is circular, so the mean is taken as a unit vector rather than as an
    arithmetic average - otherwise a red sample straddling 0/179 averages to
    cyan. Returns a band list, split in two when red wraps around.
    """
    x, y, w, h = box
    hsv = cv2.cvtColor(frame[y:y + h, x:x + w], cv2.COLOR_BGR2HSV).reshape(-1, 3)
    pixels = hsv[(hsv[:, 1] >= 60) & (hsv[:, 2] >= 35)]
    if len(pixels) < 20:
        print("too few coloured pixels - aim at bare pole, not the background")
        return None

    angles = pixels[:, 0].astype(float) * 2 * np.pi / 180
    vector = np.mean(np.exp(1j * angles))
    if abs(vector) < 0.8:
        print("sample is too colour-mixed - aim squarely at one pole")
        return None
    center = np.angle(vector) * 180 / (2 * np.pi) % 180

    offsets = (pixels[:, 0].astype(float) - center + 90) % 180 - 90
    lo, hi = np.percentile(offsets, [5, 95])
    if hi - lo > 35:
        print("hue spread too wide - the sample covers more than one colour")
        return None

    low, high = int(np.floor(center + lo - 5)), int(np.ceil(center + hi + 5))
    s0 = max(40, int(np.percentile(pixels[:, 1], 5)) - 35)
    v0 = max(25, int(np.percentile(pixels[:, 2], 5)) - 45)

    if low < 0:
        hue_bands = [(0, high), (low + 180, 179)]
    elif high > 179:
        hue_bands = [(low, 179), (0, high - 180)]
    else:
        hue_bands = [(low, high)]
    return [[a, b, s0, 255, v0, 255] for a, b in hue_bands]


class ColorDetector:
    """What the console drives: the calibrated bands, plus the colour order
    the mission is judged in."""

    def __init__(self, config=CONFIG_PATH):
        self.ranges = load_config(config)
        self.config_path = Path(config)
        # Mission 5 asks for the colours in a fixed order; the operator
        # advances with N after visually confirming a ball dropped.
        self.order = ["R", "B", "Y"]
        self.step = 0
        self.show_masks = False

    @property
    def required(self):
        return self.order[self.step] if self.step < len(self.order) else None

    def advance(self):
        self.step = min(len(self.order), self.step + 1)

    def apply_calibration(self, color, bands):
        self.ranges[color] = bands
        validate_ranges(self.ranges)
        save_config(self.ranges, self.config_path)
