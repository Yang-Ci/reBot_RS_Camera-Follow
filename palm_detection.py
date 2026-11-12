"""Compatibility alias for the packaged hand-detection implementation."""
import sys
from robot_follow.perception import hand as _implementation

# Preserve assignment to legacy module-level tuning constants as well as imports.
sys.modules[__name__] = _implementation
