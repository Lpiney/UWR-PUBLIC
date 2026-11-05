"""PNG output through Python paths, including Windows Unicode user folders."""
from pathlib import Path

import cv2


def write_png(path, frame):
    ok, encoded = cv2.imencode('.png', frame)
    if not ok:
        raise OSError(f'could not encode PNG: {path}')
    Path(path).write_bytes(encoded.tobytes())
