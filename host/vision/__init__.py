"""Vision detectors used by the operator console.

Each module exposes a plain class that takes frames and returns results, with
no window, no camera and no key handling of its own - the console owns all
three so the detectors can share them.
"""
