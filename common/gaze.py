"""Who-is-looking-at-whom geometry, shared by the simulator (highlight) and,
later, the teleop server / agent backends (attention)."""

import numpy as np


def in_view(viewer_xy, viewer_yaw, target_xy, max_distance, half_angle_rad):
    """True if `target_xy` is within `max_distance` of the viewer and inside the
    horizontal cone of `half_angle_rad` around the viewer's heading."""
    offset = np.asarray(target_xy, dtype=np.float64) - np.asarray(viewer_xy, dtype=np.float64)
    distance = float(np.hypot(*offset))
    if distance > max_distance:
        return False
    if distance < 1e-6:
        return True
    heading = np.array((np.cos(viewer_yaw), np.sin(viewer_yaw)))
    return float(offset @ heading) / distance >= np.cos(half_angle_rad)
