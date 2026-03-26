"""
Path utility functions for FarNav MPC controller.
No ROS2 dependency — pure NumPy for testability and reuse.
"""
import numpy as np


def project_onto_path(path_x, path_y, arc_lengths, robot_x, robot_y, theta_hint,
                      search_window=50):
    """
    Find the arc-length theta of the closest point on the path to the robot.

    Uses a local search around theta_hint for speed, falls back to global
    search if hint is invalid.

    Args:
        path_x, path_y: arrays of path coordinates (N points)
        arc_lengths: array of arc-length values for each path point (N,)
        robot_x, robot_y: robot position
        theta_hint: previous theta estimate for warm-starting
        search_window: number of indices around hint to search

    Returns:
        theta: arc-length parameter of closest point
    """
    n = len(path_x)

    # Find index closest to theta_hint
    hint_idx = np.searchsorted(arc_lengths, theta_hint, side='left')
    hint_idx = np.clip(hint_idx, 0, n - 1)

    # Local search window around hint
    lo = max(0, hint_idx - search_window)
    hi = min(n, hint_idx + search_window)

    dx = path_x[lo:hi] - robot_x
    dy = path_y[lo:hi] - robot_y
    dist_sq = dx * dx + dy * dy

    best_local = lo + np.argmin(dist_sq)

    # Interpolate between the best index and its neighbors for sub-index accuracy
    theta = arc_lengths[best_local]

    # Refine: check neighbors and do linear interpolation
    if 0 < best_local < n - 1:
        # Project robot position onto the segment [best-1, best+1]
        for seg_start in [best_local - 1, best_local]:
            seg_end = seg_start + 1
            ax, ay = path_x[seg_start], path_y[seg_start]
            bx, by = path_x[seg_end], path_y[seg_end]
            abx, aby = bx - ax, by - ay
            apx, apy = robot_x - ax, robot_y - ay
            ab_len_sq = abx * abx + aby * aby
            if ab_len_sq < 1e-12:
                continue
            t = (apx * abx + apy * aby) / ab_len_sq
            t = np.clip(t, 0.0, 1.0)
            proj_x = ax + t * abx
            proj_y = ay + t * aby
            d = (proj_x - robot_x) ** 2 + (proj_y - robot_y) ** 2
            theta_candidate = arc_lengths[seg_start] + t * (arc_lengths[seg_end] - arc_lengths[seg_start])
            # Pick the closer projection
            best_d = (path_x[best_local] - robot_x) ** 2 + (path_y[best_local] - robot_y) ** 2
            if d < best_d:
                theta = theta_candidate

    return float(np.clip(theta, arc_lengths[0], arc_lengths[-1]))


def get_path_state(theta, path_x, path_y, arc_lengths):
    """
    Get path position, heading, and curvature at arc-length theta.

    Args:
        theta: arc-length parameter
        path_x, path_y: path coordinate arrays
        arc_lengths: arc-length array

    Returns:
        (x, y, psi, kappa): position, heading (rad), curvature (1/m)
    """
    n = len(path_x)
    theta = float(np.clip(theta, arc_lengths[0], arc_lengths[-1]))

    # Find bracketing indices
    idx = np.searchsorted(arc_lengths, theta, side='right') - 1
    idx = np.clip(idx, 0, n - 2)

    # Linear interpolation factor
    seg_len = arc_lengths[idx + 1] - arc_lengths[idx]
    if seg_len < 1e-12:
        t = 0.0
    else:
        t = (theta - arc_lengths[idx]) / seg_len

    # Interpolate position
    x = path_x[idx] + t * (path_x[idx + 1] - path_x[idx])
    y = path_y[idx] + t * (path_y[idx + 1] - path_y[idx])

    # Heading from finite differences
    if 0 < idx < n - 2:
        dx = path_x[idx + 1] - path_x[idx - 1]
        dy = path_y[idx + 1] - path_y[idx - 1]
    else:
        dx = path_x[min(idx + 1, n - 1)] - path_x[idx]
        dy = path_y[min(idx + 1, n - 1)] - path_y[idx]

    psi = float(np.arctan2(dy, dx))

    # Curvature from discrete second derivative
    kappa = 0.0
    if 1 <= idx <= n - 3:
        dx1 = path_x[idx + 1] - path_x[idx]
        dy1 = path_y[idx + 1] - path_y[idx]
        dx0 = path_x[idx] - path_x[idx - 1]
        dy0 = path_y[idx] - path_y[idx - 1]
        ds = 0.5 * (arc_lengths[idx + 1] - arc_lengths[idx - 1])
        if ds > 1e-6:
            dpsi = np.arctan2(dy1, dx1) - np.arctan2(dy0, dx0)
            # Wrap to [-pi, pi]
            dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi
            kappa = float(dpsi / ds)

    return x, y, psi, kappa


def compute_errors(robot_x, robot_y, robot_psi, theta, path_x, path_y, arc_lengths):
    """
    Compute cross-track error and heading error.

    Cross-track error is signed: positive = robot is left of path direction.

    Args:
        robot_x, robot_y: robot position
        robot_psi: robot heading (rad)
        theta: current arc-length projection
        path_x, path_y, arc_lengths: path data

    Returns:
        (e_c, e_psi): cross-track error (m), heading error (rad)
    """
    px, py, path_psi, _ = get_path_state(theta, path_x, path_y, arc_lengths)

    # Cross-track error (signed, positive = left of path)
    dx = robot_x - px
    dy = robot_y - py
    e_c = -np.sin(path_psi) * dx + np.cos(path_psi) * dy

    # Heading error wrapped to [-pi, pi]
    e_psi = robot_psi - path_psi
    e_psi = (e_psi + np.pi) % (2 * np.pi) - np.pi

    return float(e_c), float(e_psi)
