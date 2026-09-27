"""Geometry utilities for Robot Conga."""

import math
from typing import Any, Tuple, Union


def wrap_to_pi(angle: float) -> float:
    """Normalize an angle in radians to the interval [-pi, pi).

    Args:
        angle: Angle in radians.

    Returns:
        Equivalent angle in [-pi, pi).
    """
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp a numerical value between a minimum and maximum bound.

    Args:
        value: The value to clamp.
        minimum: Lower bound.
        maximum: Upper bound.

    Returns:
        Clamped value.

    Raises:
        ValueError: If minimum > maximum.
    """
    if minimum > maximum:
        raise ValueError(f"minimum ({minimum}) cannot exceed maximum ({maximum})")
    return max(minimum, min(value, maximum))


def distance(
    p1: Union[Tuple[float, float], Any],
    p2: Union[Tuple[float, float], Any],
) -> float:
    """Calculate 2D Euclidean distance between two points or objects with .x and .y attributes.

    Args:
        p1: First point as a tuple (x, y) or object with x, y attributes.
        p2: Second point as a tuple (x, y) or object with x, y attributes.

    Returns:
        Euclidean distance.
    """
    x1 = getattr(p1, "x", p1[0] if isinstance(p1, (tuple, list)) else None)
    y1 = getattr(p1, "y", p1[1] if isinstance(p1, (tuple, list)) else None)
    x2 = getattr(p2, "x", p2[0] if isinstance(p2, (tuple, list)) else None)
    y2 = getattr(p2, "y", p2[1] if isinstance(p2, (tuple, list)) else None)

    if x1 is None or y1 is None or x2 is None or y2 is None:
        raise ValueError(f"Points {p1} and {p2} must provide x and y coordinates")

    return math.hypot(float(x2) - float(x1), float(y2) - float(y1))


def angle_difference(target: float, source: float) -> float:
    """Compute the shortest signed angular difference (target - source) in [-pi, pi).

    Args:
        target: Target heading in radians.
        source: Source heading in radians.

    Returns:
        Signed angular difference in radians.
    """
    return wrap_to_pi(target - source)
