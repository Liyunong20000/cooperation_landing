"""Pure planar control helpers. Distances use metres and angles use radians.

These functions have no ROS dependency and do not publish commands. Callers
provide finite inputs, 0 <= v_min <= v_max and a smoothing alpha in [0, 1].
"""

import math
from typing import Optional, Tuple


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp x to [lo, hi]."""
    return max(lo, min(hi, x))


def p_with_deadzone(err: float, k: float, v_min: float, v_max: float, tol: float) -> float:
    """P control with tolerance dead-zone and velocity saturation."""
    if abs(err) <= max(0.0, tol):
        return 0.0
    v = k * err
    if v == 0.0:
        return 0.0
    if abs(v) < v_min:
        v = v_min if v > 0 else -v_min
    return clamp(v, -v_max, v_max)


def smooth_with_min_velocity(
    previous: float, target: float, alpha: float, v_min: float, v_max: float
) -> float:
    """Smooth a command while preserving its nonzero minimum magnitude."""
    if target == 0.0:
        return 0.0

    smoothed = (1.0 - alpha) * previous + alpha * target

    # Follow the current target's sign when the error changes direction.
    if target > 0.0:
        smoothed = max(smoothed, v_min)
    else:
        smoothed = min(smoothed, -v_min)

    return clamp(smoothed, -v_max, v_max)


def wrap_angle(rad: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(rad), math.cos(rad))


def world_xy_to_body_xy(vx_w: float, vy_w: float, yaw: float) -> Tuple[float, float]:
    """Convert world-frame XY velocity into the robot body frame.

    Body frame follows the standard robot convention: x forward, y left, z up.
    """
    vx_b = math.cos(yaw) * vx_w + math.sin(yaw) * vy_w
    vy_b = -math.sin(yaw) * vx_w + math.cos(yaw) * vy_w
    return vx_b, vy_b


def adaptive_vy_gain(
    x_err_abs: Optional[float],
    base_k: float,
    small_err: float,
    large_err: float,
    small_err_scale: float = 0.8,
    large_err_scale: float = 1.5,
) -> float:
    """Scale lateral gain by lateral error magnitude: larger error => stronger response."""
    if x_err_abs is None or not math.isfinite(x_err_abs):
        return base_k
    if large_err <= small_err:
        return base_k

    e = clamp(x_err_abs, small_err, large_err)
    t = (e - small_err) / (large_err - small_err)
    scale = small_err_scale + t * (large_err_scale - small_err_scale)
    return base_k * scale
