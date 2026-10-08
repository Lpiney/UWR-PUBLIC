"""Colour recognition (Mission 5).

Finds coloured poles in the frame. It does not drive the ROV and does not
decide whether a ball has dropped - that is the operator's call.

Recognition runs in a burst rather than continuously: a short window of
frames is collected and a candidate has to appear in most of them before it
counts. That keeps ripple and glare from producing a one-frame false
positive, and it means the CPU is idle the rest of the time.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
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
# A patch has to be mostly coloured, and mostly one colour, to be reported.
# Without the first test a grey pool floor or a white highlight would come
# back as whichever band its noise happened to fall in.
MIN_COLOURED_SHARE = 0.4
MIN_MATCH_SHARE = 0.4


@dataclass
class Settings:
    seconds: float = 1.0
    min_hits: int = 3
    hit_ratio: float = 0.6
    min_area: float = 0.0003
    max_area: float = 0.18
    min_aspect: float = 2.0

    def validate(self):
        if not 0.1 <= self.seconds <= 10 or not 1 <= self.min_hits <= 1000:
            raise ValueError("seconds must be 0.1-10 and min_hits 1-1000")
        if not 0 < self.hit_ratio <= 1:
            raise ValueError("hit_ratio must be in (0, 1]")
        if not 0 < self.min_area < self.max_area < 1 or self.min_aspect < 1.5:
            raise ValueError("need 0 < min_area < max_area < 1 and min_aspect >= 1.5")


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
    path = Path(path)
    if path.exists():
        data = json.loads(path.read_text(encoding="utf-8"))
        settings = Settings(**data["settings"])
        ranges = data["ranges"]
    else:
        settings = Settings()
        ranges = json.loads(json.dumps(DEFAULT_RANGES))
    settings.validate()
    validate_ranges(ranges)
    return settings, ranges


def save_config(settings, ranges, path=CONFIG_PATH):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"settings": asdict(settings), "ranges": ranges},
                               indent=2) + "\n", encoding="utf-8")


def color_mask(hsv, bands):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for h0, h1, s0, s1, v0, v1 in bands:
        mask |= cv2.inRange(hsv, (h0, s0, v0), (h1, s1, v1))
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def ball_evidence(hsv, bbox):
    """A white circle above the pole only adds confidence; it is never required.

    Scoring rewards the white ball, but a pole must still be recognised when
    the ball is out of frame or has already been knocked off.
    """
    x, y, w, h = bbox
    margin = max(8, min(w, h) * 2)
    x0, x1 = max(0, x - margin), min(hsv.shape[1], x + w + margin)
    y0, y1 = max(0, y - margin), min(hsv.shape[0], y + max(8, h // 4))
    patch = hsv[y0:y1, x0:x1]
    white = cv2.inRange(patch, (0, 0, 150), (179, 65, 255))
    contours, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        bx, by, bw, bh = cv2.boundingRect(contour)
        # Large white areas (pool walls, text) and tiny specks are not the ball.
        if area < 12 or perimeter == 0 or area > w * h * 0.5:
            continue
        if 0.65 < bw / max(bh, 1) < 1.5 and 4 * np.pi * area / perimeter**2 > 0.65:
            center_x, center_y = x0 + bx + bw / 2, y0 + by + bh / 2
            if x - w * 0.5 <= center_x <= x + w * 1.5 and center_y <= y + h * 0.12:
                return True
    return False


def detect(frame, ranges, settings, roi=None):
    """Return candidate poles plus the debug masks.

    Only called while a burst is running. ROI coordinates are in original
    image pixels.
    """
    height, width = frame.shape[:2]
    x0, y0, rw, rh = roi or (0, 0, width, height)
    crop = frame[y0:y0 + rh, x0:x0 + rw]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    masks = {}
    candidates = []
    # Areas are fractions of the whole image, so shrinking the ROI must not
    # turn an ordinary pole into "too large to be a pole".
    image_area = width * height

    for color, bands in ranges.items():
        mask = color_mask(hsv, bands)
        masks[color] = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if not settings.min_area <= area / image_area <= settings.max_area:
                continue
            rect = cv2.minAreaRect(contour)
            short, long = sorted(rect[1])
            if short < 3 or long < 18 or long / short < settings.min_aspect:
                continue
            fill = area / max(short * long, 1)
            if fill < 0.28:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            # A blob touching both opposite edges is pool-floor striping or
            # background, not a pole. Clipped on one side only is still fine.
            if (x <= 1 and x + w >= rw - 1) or (y <= 1 and y + h >= rh - 1):
                continue
            # Horizontal stripes are also elongated; reject near-horizontal
            # targets. If the camera is rolled, level the image - loosening
            # the colour thresholds will not help.
            if h < w * 0.8:
                continue
            ball = ball_evidence(hsv, (x, y, w, h))
            score = min(long / short / 8, 1) * 0.4 + min(fill, 1) * 0.35
            score += 0.15 * min(h / max(w, 1) / 3, 1) + 0.1 * ball
            candidates.append({"color": color, "bbox": [x + x0, y + y0, w, h],
                               "shape_score": round(float(score), 3), "ball_hint": ball})

    # Calibrated ranges can overlap, so one object may match two colours.
    # Refuse to guess rather than report both.
    ambiguous = set()
    for i, a in enumerate(candidates):
        ax, ay, aw, ah = a["bbox"]
        for j in range(i + 1, len(candidates)):
            b = candidates[j]
            if a["color"] == b["color"]:
                continue
            bx, by, bw, bh = b["bbox"]
            overlap = (max(0, min(ax + aw, bx + bw) - max(ax, bx))
                       * max(0, min(ay + ah, by + bh) - max(ay, by)))
            if overlap / max(aw * ah + bw * bh - overlap, 1) > 0.5:
                ambiguous.update((i, j))

    return sorted((c for i, c in enumerate(candidates) if i not in ambiguous),
                  key=lambda c: c["shape_score"], reverse=True), masks


class Burst:
    """One recognition window: collect frames, then decide.

    Nothing is computed before start() or after the window closes.
    """

    def __init__(self, settings):
        self.settings = settings
        self.active = False
        self.result = None
        self.snapshot = None
        self.frames = 0
        self.tracks = []
        self.started = 0.0
        self.last_frame = None
        self.masks = None

    def reset(self):
        self.active = False
        self.result = None
        self.snapshot = None

    def start(self, now):
        self.active = True
        self.started = now
        self.frames = 0
        self.tracks = []
        self.result = None
        self.snapshot = None
        self.last_frame = None
        self.masks = None

    def update(self, frame, ranges, roi, now):
        if not self.active:
            return None
        candidates, self.masks = detect(frame, ranges, self.settings, roi)
        self.frames += 1
        self.last_frame = frame.copy()

        used = set()
        for candidate in candidates:
            x, y, w, h = candidate["bbox"]
            center = np.array([x + w / 2, y + h / 2])
            choices = []
            for i, track in enumerate(self.tracks):
                if i in used or track["candidate"]["color"] != candidate["color"]:
                    continue
                tx, ty, tw, th = track["candidate"]["bbox"]
                distance = np.linalg.norm(center - [tx + tw / 2, ty + th / 2])
                if distance <= max(20, min(h, th) * 0.3) and 0.5 <= h / th <= 2:
                    choices.append((distance, i))
            if choices:
                _, i = min(choices)
                self.tracks[i]["hits"] += 1
                self.tracks[i]["candidate"] = candidate
                self.tracks[i]["last_seen"] = self.frames
            else:
                i = len(self.tracks)
                self.tracks.append({"candidate": candidate, "hits": 1,
                                    "last_seen": self.frames})
            used.add(i)

        if now - self.started >= self.settings.seconds:
            return self.finish()
        return None

    def finish(self):
        self.active = False
        confirmed = []
        for track in self.tracks:
            # The result is drawn on the closing frame, so a target must still
            # be visible in it - otherwise a box would linger for a pole that
            # has already left the frame.
            if (track["hits"] >= self.settings.min_hits
                    and track["hits"] / max(self.frames, 1) >= self.settings.hit_ratio
                    and track["last_seen"] == self.frames):
                confirmed.append({**track["candidate"], "hits": track["hits"],
                                  "hit_ratio": round(track["hits"] / self.frames, 3)})
        self.result = {"status": "confirmed" if confirmed else "unconfirmed",
                       "frames": self.frames, "temporal_check": True,
                       "targets": confirmed}
        self.snapshot = self.last_frame
        return self.result


def box_around(x, y, radius, shape):
    """A square box centred on (x, y), clipped to the frame."""
    height, width = shape[:2]
    x0, y0 = max(0, int(x) - radius), max(0, int(y) - radius)
    x1 = min(width, int(x) + radius + 1)
    y1 = min(height, int(y) + radius + 1)
    return x0, y0, max(0, x1 - x0), max(0, y1 - y0)


def sample_at(frame, x, y, ranges, radius=SAMPLE_RADIUS):
    """Which of R/Y/B is under the pointer, or None if none of them is.

    Pointing is deliberate, so there is no shape filtering here: the operator
    has already said where to look. What is checked is that the patch is
    genuinely coloured and that a single colour covers most of it - a patch
    straddling the edge of two poles is not an answer.
    """
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


def calibrate(frame, box):
    """Derive HSV thresholds from a box the operator drew around bare pole.

    Hue is circular, so the mean is taken as a unit vector rather than as an
    arithmetic average - otherwise a red sample straddling 0/179 averages to
    cyan. Returns a band list, split in two when red wraps around.
    """
    x, y, w, h = box
    hsv = cv2.cvtColor(frame[y:y + h, x:x + w], cv2.COLOR_BGR2HSV).reshape(-1, 3)
    pixels = hsv[(hsv[:, 1] >= 60) & (hsv[:, 2] >= 35)]
    if len(pixels) < 20:
        print("too few coloured pixels - draw the box tightly around the pole")
        return None

    angles = pixels[:, 0].astype(float) * 2 * np.pi / 180
    vector = np.mean(np.exp(1j * angles))
    if abs(vector) < 0.8:
        print("sample is too colour-mixed - narrow the box to bare pole")
        return None
    center = np.angle(vector) * 180 / (2 * np.pi) % 180

    offsets = (pixels[:, 0].astype(float) - center + 90) % 180 - 90
    lo, hi = np.percentile(offsets, [5, 95])
    if hi - lo > 35:
        print("hue spread too wide - draw the box again")
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


def annotate(frame, result, required=None):
    """Draw the confirmed targets onto a frame, in place."""
    for item in result["targets"]:
        x, y, w, h = item["bbox"]
        color = item["color"]
        cv2.rectangle(frame, (x, y), (x + w, y + h), COLORS[color], 3)
        cx = x + w / 2
        if cx < frame.shape[1] * 0.4:
            direction = "LEFT"
        elif cx > frame.shape[1] * 0.6:
            direction = "RIGHT"
        else:
            direction = "CENTER"
        label = f"{NAMES[color]} {direction}"
        if color == required:
            label += "  TARGET"
        cv2.putText(frame, label, (x, max(45, y - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, COLORS[color], 2)
    return frame


class ColorDetector:
    """Facade the console drives: a burst, its result and the debug masks."""

    def __init__(self, config=CONFIG_PATH, source="0"):
        self.settings, self.ranges = load_config(config)
        self.config_path = Path(config)
        self.burst = Burst(self.settings)
        # Mission 5 asks for the colours in a fixed order; the operator
        # advances with N after visually confirming a ball dropped.
        self.order = ["R", "B", "Y"]
        self.step = 0
        self.roi = None
        self.show_masks = False

    @property
    def required(self):
        return self.order[self.step] if self.step < len(self.order) else None

    def start(self, now):
        self.burst.start(now)

    def update(self, frame, now):
        return self.burst.update(frame, self.ranges, self.roi, now)

    def advance(self):
        self.step = min(len(self.order), self.step + 1)
        self.burst.reset()

    def clear(self):
        self.burst.reset()

    def set_roi(self, box):
        self.roi = box
        self.burst.reset()

    def apply_calibration(self, color, bands):
        self.ranges[color] = bands
        validate_ranges(self.ranges)
        save_config(self.settings, self.ranges, self.config_path)
