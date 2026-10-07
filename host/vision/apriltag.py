"""AprilTag detection (Mission 2.1).

One family only: the competition notice specifies 36h11. Enabling several
dictionaries at once cannot improve the detection rate and it can produce a
wrong ID - and per the rules, reading the wrong tag scores zero. Every
detected square gets tried against every enabled dictionary, so adding
dictionaries only increases the chance of a misread.

No image preprocessing. Earlier versions carried denoise, gamma and CLAHE
options that had measured worse than doing nothing on degraded test images,
so they were removed rather than left in as switches nobody should turn on.
If detection turns out to be unstable in the pool, min_hits is the knob.

Frames come from the console; this module never opens a camera or a window.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

FAMILIES = {
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

# BGR, not RGB.
COLOR_TARGET = (0, 165, 255)     # orange: the chosen target tag
COLOR_TAG = (0, 255, 0)          # green: confirmed, but not the target
COLOR_PENDING = (170, 170, 170)  # grey: seen, not yet confirmed


@dataclass
class Sighting:
    """Accumulated observations of one tag ID.

    A single frame is not enough to trust: as the ROV moves, a tag is
    intermittently blurred by ripples or occluded, so frame-by-frame results
    jump around. Accumulating makes the answer stable.
    """

    tag_id: int
    corners: np.ndarray
    hits: int = 1
    area: float = 0.0    # area of the best view so far


class AprilTagScanner:
    """Detects tags, accumulates sightings, and decides the target ID."""

    def __init__(self, families=("36h11",), mode="largest", min_hits=3,
                 expected=3):
        """
        families: dictionaries to enable. Keep it to 36h11.
        mode:     "largest" or "smallest" - which ID to report as target.
        min_hits: frames a tag must be seen in before it counts.
        expected: how many tags are on the field, for the n/3 progress.
        """
        self.detectors = []
        for name in families:
            if name not in FAMILIES:
                raise ValueError(f"unknown family {name!r}; choose from {', '.join(FAMILIES)}")
            dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[name])
            self.detectors.append((name, cv2.aruco.ArucoDetector(
                dictionary, cv2.aruco.DetectorParameters())))

        self.mode = mode
        self.min_hits = min_hits
        self.expected = expected
        self.sightings: dict[int, Sighting] = {}

    def reset(self):
        self.sightings.clear()

    def to_gray(self, frame):
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def detect(self, frame):
        """Detect every tag in one frame and merge the result into the totals.

        Returns {tag_id: corners} for this frame alone; the answer to "which
        tag do we report" is target_id, which uses the accumulated sightings.
        """
        gray = self.to_gray(frame)
        found: dict[int, np.ndarray] = {}

        for _, detector in self.detectors:
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is None:
                continue
            for marker_corners, tag_id in zip(corners, ids.flatten()):
                # setdefault keeps the first hit, so a tag seen by more than
                # one dictionary is never counted twice.
                found.setdefault(int(tag_id), marker_corners)

        for tag_id, corners in found.items():
            # A larger area means the tag is closer and more square-on, so
            # keep the corners from the best view for drawing.
            area = float(cv2.contourArea(corners.reshape(-1, 1, 2)))
            sighting = self.sightings.get(tag_id)
            if sighting is None:
                self.sightings[tag_id] = Sighting(tag_id, corners, 1, area)
            else:
                sighting.hits += 1
                if area > sighting.area:
                    sighting.area = area
                    sighting.corners = corners
        return found

    @property
    def confirmed_ids(self) -> list[int]:
        return sorted(i for i, s in self.sightings.items() if s.hits >= self.min_hits)

    @property
    def target_id(self):
        """The ID to report: the largest or smallest confirmed one.

        This is the "apply the selection logic" part of Mission 2.1.
        """
        ids = self.confirmed_ids
        if not ids:
            return None
        return max(ids) if self.mode == "largest" else min(ids)

    def result(self) -> dict:
        return {
            "mode": self.mode,
            "target": self.target_id,
            "detected": self.confirmed_ids,
            "scanned": len(self.confirmed_ids),
            "expected": self.expected,
            "complete": len(self.confirmed_ids) >= self.expected,
        }

    def annotate(self, frame):
        """Draw the boxes and labels onto the frame, in place."""
        target = self.target_id
        for tag_id, sighting in sorted(self.sightings.items()):
            is_confirmed = sighting.hits >= self.min_hits
            is_target = tag_id == target

            if is_target:
                color, thickness = COLOR_TARGET, max(3, frame.shape[1] // 320)
            elif is_confirmed:
                color, thickness = COLOR_TAG, 2
            else:
                color, thickness = COLOR_PENDING, 1

            pts = sighting.corners.reshape(-1, 2).astype(np.int32)
            cv2.polylines(frame, [pts], True, color, thickness, cv2.LINE_AA)

            label = f"ID {tag_id}"
            if is_target:
                label += "  TARGET"
            elif not is_confirmed:
                label += f"  {sighting.hits}/{self.min_hits}"
            top = pts[pts[:, 1].argmin()]
            draw_label(frame, label, (int(top[0]), int(top[1]) - 12), color)

            if is_target:
                # A crosshair on the target helps the pilot line the ROV up.
                center = pts.mean(axis=0).astype(int)
                cv2.drawMarker(frame, tuple(center), color, cv2.MARKER_CROSS,
                               30, 2, cv2.LINE_AA)
        return frame


def draw_label(frame, text, org, color, scale=0.8, thickness=2):
    """Text with a black backing.

    Underwater frames are visually busy - pool floor, ripples, reflections -
    and plain text disappears into them. Filling a black rectangle first makes
    it readable regardless of what is behind it.
    """
    (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    x, y = org
    cv2.rectangle(frame, (x, y - h - 6), (x + w + 6, y + 4), (0, 0, 0), -1)
    cv2.putText(frame, text, (x + 3, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thickness, cv2.LINE_AA)
