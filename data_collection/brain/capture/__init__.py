"""Frame sources: real camera, and a synthetic one for off-hardware work."""

from .source import CaptureSource, Frame, SyntheticThrowSource

__all__ = ["CaptureSource", "Frame", "SyntheticThrowSource"]

# PiCamera2Source is imported lazily -- picamera2 only exists on the Pi.
