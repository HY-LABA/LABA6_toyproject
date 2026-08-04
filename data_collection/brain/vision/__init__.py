"""Detection, tracking, and dataset handling.

Detector and tracker are imported lazily. They need OpenCV; dataset.py does
not, and requiring cv2 just to read a manifest would tie the training host to
the capture host for no reason.
"""

from .dataset import Sample, load_manifests, split_by_object, summarize

__all__ = [
    "Detection",
    "MotionDetector",
    "Sample",
    "TrackManager",
    "load_manifests",
    "split_by_object",
    "summarize",
]

_LAZY = {
    "Detection": ".detector",
    "MotionDetector": ".detector",
    "TrackManager": ".tracker",
}


def __getattr__(name):
    if name in _LAZY:
        import importlib
        module = importlib.import_module(_LAZY[name], __package__)
        return getattr(module, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
