# Localization test using the new AprilTag
# Format: tag36h11
# ID's : 0-11
# Tag size: 143.7mm x 143.7mm
# Locations: On walls in apartment (Tried to have at least 2 tags visable regardless of location in the room)

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)


import rclpy
