"""Pytest configuration and pythonpath resolution."""

import sys
from pathlib import Path

# Add src/ directory to sys.path so robot_conga can be imported without installation
src_dir = Path(__file__).resolve().parent.parent
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))
