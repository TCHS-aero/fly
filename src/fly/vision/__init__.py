"""
Vision package for the Fly project.
Handles object detection and computer vision tasks.
"""

__all__ = ["NanoDetector"] # ignore unused type checker error

def __getattr__(name):
    # PEP 562 lazy attribute: `fly.vision.gcsPipeline` and `detect.py` already defer importing NanoDetector
    # until a worker process actually runs detection.
    # setup_env.sh must be run before using this
    if name == "NanoDetector":
        from .rf_detr_nano import NanoDetector

        return NanoDetector
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    # since module not using default path, need to preserve
    # normal missing attr behavior
