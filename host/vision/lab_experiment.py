"""Opt-in, camera-calibrated CIE76 colour experiment; independent of HSV config."""
import json
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import cv2
import numpy as np

from .color import box_around
from .image_io import write_png

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / "lab-config.json"
LOGS = ROOT / "snapshots" / "colour-diagnostics"


def lab_sample(frame, x, y, radius):
    x0, y0, w, h = box_around(x, y, radius, frame.shape)
    patch = frame[y0:y0+h, x0:x0+w]
    if not patch.size:
        raise ValueError("empty sample")
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    valid = (hsv[:, :, 1] >= 30) & (hsv[:, :, 2] >= 25)
    if np.mean(valid) < 0.4:
        raise ValueError("too few coloured pixels; aim at the pole centre")
    # Float input gives real CIELAB units (L=0..100, a/b signed).
    lab = cv2.cvtColor(patch.astype(np.float32) / 255, cv2.COLOR_BGR2LAB)
    pixels = lab[valid]
    center = np.median(pixels, axis=0)
    if np.percentile(np.linalg.norm(pixels - center, axis=1), 80) > 20:
        raise ValueError("mixed sample; use a smaller box or avoid the pole edge")
    return center.tolist()


class LabExperiment:
    def __init__(self, path=CONFIG):
        self.path = Path(path)
        self.samples = {c: [] for c in "RYB"}
        self.load_error = None
        try:
            if self.path.exists():
                data = json.loads(self.path.read_text(encoding="utf-8"))
                for c in self.samples:
                    values = np.asarray(data[c], dtype=float)
                    if values.size and (values.ndim != 2 or values.shape[1] != 3
                                        or not np.isfinite(values).all()):
                        raise ValueError("invalid Lab samples")
                    self.samples[c] = values.reshape(-1, 3).tolist()[-1:]
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.samples = {c: [] for c in "RYB"}
            self.load_error = str(exc)

    def add(self, color, frame, x, y, radius):
        sample = lab_sample(frame, x, y, radius)
        updated = {c: list(v) for c, v in self.samples.items()}
        updated[color] = [sample]
        self.path.write_text(json.dumps(updated, indent=2) + "\n", encoding="utf-8")
        self.samples = updated
        return sample

    def classify(self, frame, x, y, radius):
        report = {"answer": None, "reason": "calibrate all three colours first",
                  "threshold": 25.0, "margin_required": 5.0}
        if not all(self.samples.values()):
            return report
        try:
            sample = lab_sample(frame, x, y, radius)
        except ValueError as exc:
            report["reason"] = str(exc)
            return report
        # Each colour has one reference: the latest successful calibration.
        distances = {c: float(np.min(np.linalg.norm(np.asarray(v) - sample, axis=1)))
                     for c, v in self.samples.items()}
        ranked = sorted(distances, key=distances.get)
        best, second = ranked[:2]
        report.update(lab=sample, distances=distances)
        if distances[best] > report["threshold"]:
            report["reason"] = "too far from calibrated colours"
        elif distances[second] - distances[best] < report["margin_required"]:
            report["reason"] = "ambiguous between two colours"
        else:
            report.update(answer=best, reason="accepted")
        return report


def record(frame, report, folder=LOGS):
    """Save unannotated input and metadata for successful AND rejected attempts."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    now = datetime.now().astimezone()
    name = now.strftime("%Y%m%d-%H%M%S-%f") + "-" + uuid4().hex[:6]
    path = folder / (name + ".png")
    write_png(path, frame)
    row = dict(report, time=now.isoformat(), image=path.name)
    with (folder / "events.jsonl").open("a", encoding="utf-8") as output:
        output.write(json.dumps(row) + "\n")
    return path
