# controller_mpc Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build an MPC controller node that follows crop-row spline paths on the Farm-ng Amiga skid-steer robot.

**Architecture:** CasADi/IPOPT MPC with unicycle dynamics, subscribing to odometry + global path, publishing velocity commands. Shared `path_utils.py` library handles path projection and error computation. All parameters in YAML.

**Tech Stack:** ROS2 (rclpy), CasADi, IPOPT, NumPy, SciPy

---

### Task 1: path_utils.py — Path Utility Library

**Files:**
- Create: `src/farnav/farnav/path_utils.py`
- Test: `src/farnav/test/test_path_utils.py`

**Step 1: Write failing tests for all three functions**

```python
# src/farnav/test/test_path_utils.py
import numpy as np
import pytest
from farnav.path_utils import project_onto_path, get_path_state, compute_errors


class TestProjectOntoPath:
    """Test closest-point projection onto a path."""

    def setup_method(self):
        """Straight path along X axis from (0,0) to (10,0)."""
        self.path_x = np.linspace(0.0, 10.0, 100)
        self.path_y = np.zeros(100)
        self.arc_lengths = np.linspace(0.0, 10.0, 100)

    def test_robot_on_path_midpoint(self):
        theta = project_onto_path(
            self.path_x, self.path_y, self.arc_lengths,
            robot_x=5.0, robot_y=0.0, theta_hint=4.0
        )
        assert abs(theta - 5.0) < 0.15

    def test_robot_offset_laterally(self):
        theta = project_onto_path(
            self.path_x, self.path_y, self.arc_lengths,
            robot_x=5.0, robot_y=2.0, theta_hint=4.0
        )
        assert abs(theta - 5.0) < 0.15

    def test_robot_at_start(self):
        theta = project_onto_path(
            self.path_x, self.path_y, self.arc_lengths,
            robot_x=0.0, robot_y=0.0, theta_hint=0.0
        )
        assert abs(theta - 0.0) < 0.15

    def test_robot_at_end(self):
        theta = project_onto_path(
            self.path_x, self.path_y, self.arc_lengths,
            robot_x=10.0, robot_y=0.0, theta_hint=9.0
        )
        assert abs(theta - 10.0) < 0.15


class TestGetPathState:
    """Test path state interpolation."""

    def setup_method(self):
        """Straight path along X axis."""
        self.path_x = np.linspace(0.0, 10.0, 100)
        self.path_y = np.zeros(100)
        self.arc_lengths = np.linspace(0.0, 10.0, 100)

    def test_midpoint_position(self):
        x, y, psi, kappa = get_path_state(
            5.0, self.path_x, self.path_y, self.arc_lengths
        )
        assert abs(x - 5.0) < 0.15
        assert abs(y - 0.0) < 0.01

    def test_heading_straight_path(self):
        _, _, psi, _ = get_path_state(
            5.0, self.path_x, self.path_y, self.arc_lengths
        )
        assert abs(psi - 0.0) < 0.05  # heading ~0 for path along X

    def test_curvature_straight_path(self):
        _, _, _, kappa = get_path_state(
            5.0, self.path_x, self.path_y, self.arc_lengths
        )
        assert abs(kappa) < 0.01  # ~zero curvature for straight line

    def test_clamp_beyond_end(self):
        x, y, _, _ = get_path_state(
            15.0, self.path_x, self.path_y, self.arc_lengths
        )
        assert abs(x - 10.0) < 0.15  # clamps to end


class TestComputeErrors:
    """Test cross-track and heading error computation."""

    def setup_method(self):
        """Straight path along X axis."""
        self.path_x = np.linspace(0.0, 10.0, 100)
        self.path_y = np.zeros(100)
        self.arc_lengths = np.linspace(0.0, 10.0, 100)

    def test_on_path_zero_errors(self):
        e_c, e_psi = compute_errors(
            robot_x=5.0, robot_y=0.0, robot_psi=0.0,
            theta=5.0,
            path_x=self.path_x, path_y=self.path_y,
            arc_lengths=self.arc_lengths
        )
        assert abs(e_c) < 0.01
        assert abs(e_psi) < 0.05

    def test_lateral_offset_positive(self):
        e_c, _ = compute_errors(
            robot_x=5.0, robot_y=1.0, robot_psi=0.0,
            theta=5.0,
            path_x=self.path_x, path_y=self.path_y,
            arc_lengths=self.arc_lengths
        )
        assert abs(e_c - 1.0) < 0.15  # 1m left of path

    def test_heading_error(self):
        _, e_psi = compute_errors(
            robot_x=5.0, robot_y=0.0, robot_psi=0.5,
            theta=5.0,
            path_x=self.path_x, path_y=self.path_y,
            arc_lengths=self.arc_lengths
        )
        assert abs(e_psi - 0.5) < 0.05
```

**Step 2: Run tests to verify they fail**

Run: `cd /home/het/lab/amiga_project/src/farnav && python -m pytest test/test_path_utils.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'farnav.path_utils'`

**Step 3: Implement path_utils.py**

```python
# src/farnav/farnav/path_utils.py
"""
Path utility functions for FarNav MPC controller.
No ROS2 dependency — pure NumPy for testability and reuse.
"""
import numpy as np


def project_onto_path(path_x, path_y, arc_lengths, robot_x, robot_y, theta_hint,
                      search_window=50):
    """
    Find the arc-length θ of the closest point on the path to the robot.

    Uses a local search around theta_hint for speed, falls back to global
    search if hint is invalid.

    Args:
        path_x, path_y: arrays of path coordinates (N points)
        arc_lengths: array of arc-length values for each path point (N,)
        robot_x, robot_y: robot position
        theta_hint: previous θ estimate for warm-starting
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
            if seg_start == best_local - 1 or seg_start == best_local:
                # Pick the closer projection
                best_d = (path_x[best_local] - robot_x) ** 2 + (path_y[best_local] - robot_y) ** 2
                if d < best_d:
                    theta = theta_candidate

    return float(np.clip(theta, arc_lengths[0], arc_lengths[-1]))


def get_path_state(theta, path_x, path_y, arc_lengths):
    """
    Get path position, heading, and curvature at arc-length θ.

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
    # Use central difference where possible
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
```

**Step 4: Run tests to verify they pass**

Run: `cd /home/het/lab/amiga_project/src/farnav && python -m pytest test/test_path_utils.py -v`
Expected: All 11 tests PASS

**Step 5: Commit**

```bash
git add src/farnav/farnav/path_utils.py src/farnav/test/test_path_utils.py
git commit -m "feat(farnav): add path_utils.py with project, get_path_state, compute_errors"
```

---

### Task 2: controller_mpc.yaml — Configuration File

**Files:**
- Create: `src/farnav/config/controller_mpc.yaml`

**Step 1: Create the YAML config**

```yaml
# src/farnav/config/controller_mpc.yaml
# MPC Controller parameters for FarNav crop-row following
controller_mpc:
  ros__parameters:
    # --- Horizon ---
    N: 20                       # prediction steps
    dt: 0.1                     # prediction time step (s) → 2s lookahead

    # --- Cost weights ---
    w_cte: 10.0                 # cross-track error
    w_head: 5.0                 # heading error
    w_v: 1.0                    # velocity tracking (v - v_ref)
    w_dv: 2.0                   # control smoothness: delta v_cmd
    w_dw: 5.0                   # control smoothness: delta omega_cmd

    # --- Constraints ---
    v_wheel_max: 1.5            # max individual wheel speed (m/s)
    omega_max: 1.0              # max angular velocity (rad/s)
    delta_omega_max: 0.5        # max angular velocity change per step (rad/s)
    row_half_width: 0.45        # lateral constraint (m)

    # --- Vehicle parameters ---
    track_half_width: 0.435     # half track width b (m), Amiga ~0.87m track
    tau_v: 0.3                  # velocity first-order lag time constant (s)

    # --- Reference ---
    v_ref: 0.5                  # desired cruise speed (m/s)

    # --- Node settings ---
    control_rate_hz: 20.0       # control loop frequency (Hz)
    path_frame_id: "utm"        # coordinate frame for path
```

**Step 2: Commit**

```bash
git add src/farnav/config/controller_mpc.yaml
git commit -m "feat(farnav): add controller_mpc.yaml with all MPC params"
```

---

### Task 3: controller_mpc.py — MPC Node (CasADi Setup + Solver Build)

**Files:**
- Create: `src/farnav/farnav/controller_mpc.py`
- Test: `src/farnav/test/test_controller_mpc.py`

**Step 1: Write failing test for MPC solver construction and a straight-line solve**

```python
# src/farnav/test/test_controller_mpc.py
import numpy as np
import pytest

from farnav.controller_mpc import build_mpc_solver, solve_mpc


class TestBuildSolver:
    """Test that the CasADi solver builds without error."""

    def test_solver_builds(self):
        params = {
            'N': 20, 'dt': 0.1,
            'w_cte': 10.0, 'w_head': 5.0, 'w_v': 1.0, 'w_dv': 2.0, 'w_dw': 5.0,
            'v_wheel_max': 1.5, 'omega_max': 1.0, 'delta_omega_max': 0.5,
            'row_half_width': 0.45, 'track_half_width': 0.435, 'tau_v': 0.3,
            'v_ref': 0.5,
        }
        solver, nlp_dims = build_mpc_solver(params)
        assert solver is not None
        assert nlp_dims['n_x'] == 4
        assert nlp_dims['n_u'] == 2
        assert nlp_dims['N'] == 20


class TestSolveMPC:
    """Test MPC on a straight path — robot offset laterally."""

    def setup_method(self):
        self.params = {
            'N': 20, 'dt': 0.1,
            'w_cte': 10.0, 'w_head': 5.0, 'w_v': 1.0, 'w_dv': 2.0, 'w_dw': 5.0,
            'v_wheel_max': 1.5, 'omega_max': 1.0, 'delta_omega_max': 0.5,
            'row_half_width': 0.45, 'track_half_width': 0.435, 'tau_v': 0.3,
            'v_ref': 0.5,
        }
        self.solver, self.nlp_dims = build_mpc_solver(self.params)

        # Straight path along X axis
        self.path_x = np.linspace(0.0, 50.0, 500)
        self.path_y = np.zeros(500)
        self.arc_lengths = np.linspace(0.0, 50.0, 500)

    def test_straight_path_offset_steers_back(self):
        """Robot is 0.3m left of path, heading along path. Should steer right (omega < 0)."""
        x0 = np.array([5.0, 0.3, 0.0, 0.0])  # [X, Y, psi, v]
        theta_current = 5.0
        prev_u = np.array([0.0, 0.0])

        v_cmd, omega_cmd, predicted_states = solve_mpc(
            self.solver, self.nlp_dims, self.params,
            x0, theta_current, prev_u,
            self.path_x, self.path_y, self.arc_lengths
        )
        # Should command forward velocity near v_ref
        assert v_cmd > 0.1
        # Should steer right (negative omega) to correct positive CTE
        assert omega_cmd < 0.0

    def test_on_path_drives_straight(self):
        """Robot exactly on path, heading correct. Should drive straight."""
        x0 = np.array([5.0, 0.0, 0.0, 0.0])
        theta_current = 5.0
        prev_u = np.array([0.0, 0.0])

        v_cmd, omega_cmd, predicted_states = solve_mpc(
            self.solver, self.nlp_dims, self.params,
            x0, theta_current, prev_u,
            self.path_x, self.path_y, self.arc_lengths
        )
        assert v_cmd > 0.1
        assert abs(omega_cmd) < 0.1  # ~straight

    def test_heading_error_corrects(self):
        """Robot on path but heading 30 deg off. Should steer to correct."""
        x0 = np.array([5.0, 0.0, 0.5, 0.3])  # ~30 deg heading error
        theta_current = 5.0
        prev_u = np.array([0.3, 0.0])

        v_cmd, omega_cmd, predicted_states = solve_mpc(
            self.solver, self.nlp_dims, self.params,
            x0, theta_current, prev_u,
            self.path_x, self.path_y, self.arc_lengths
        )
        # Should steer negative to correct positive heading error
        assert omega_cmd < -0.05

    def test_predicted_states_shape(self):
        """Predicted trajectory should have N+1 states."""
        x0 = np.array([5.0, 0.0, 0.0, 0.0])
        theta_current = 5.0
        prev_u = np.array([0.0, 0.0])

        _, _, predicted_states = solve_mpc(
            self.solver, self.nlp_dims, self.params,
            x0, theta_current, prev_u,
            self.path_x, self.path_y, self.arc_lengths
        )
        assert predicted_states.shape == (21, 4)  # N+1 x n_x
```

**Step 2: Run tests to verify they fail**

Run: `cd /home/het/lab/amiga_project/src/farnav && python -m pytest test/test_controller_mpc.py -v`
Expected: FAIL — `ModuleNotFoundError`

**Step 3: Implement controller_mpc.py with `build_mpc_solver` and `solve_mpc` functions + ROS2 node**

```python
# src/farnav/farnav/controller_mpc.py
"""
MPC controller for FarNav crop-row following on Farm-ng Amiga.
Unicycle dynamics, CasADi/IPOPT solver.
"""
import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool

from farnav.path_utils import project_onto_path, get_path_state, compute_errors

# ── Euler from quaternion (only yaw) ─────────────────────────────────────
def _yaw_from_quat(q):
    """Extract yaw from geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


# ── CasADi MPC builder (no ROS dependency) ──────────────────────────────
def build_mpc_solver(params):
    """
    Build a CasADi NLP solver for the MPC problem.

    Args:
        params: dict with keys N, dt, w_cte, w_head, w_v, w_dv, w_dw,
                v_wheel_max, omega_max, delta_omega_max, row_half_width,
                track_half_width, tau_v, v_ref

    Returns:
        (solver, nlp_dims): CasADi solver object and dimension info dict
    """
    N = params['N']
    dt = params['dt']
    n_x = 4   # [X, Y, psi, v]
    n_u = 2   # [v_cmd, omega_cmd]

    tau_v = params['tau_v']
    b = params['track_half_width']
    v_ref = params['v_ref']
    w_cte = params['w_cte']
    w_head = params['w_head']
    w_v = params['w_v']
    w_dv = params['w_dv']
    w_dw = params['w_dw']
    v_wheel_max = params['v_wheel_max']
    omega_max = params['omega_max']
    delta_omega_max = params['delta_omega_max']
    row_half_width = params['row_half_width']

    # ── Decision variables ──
    # States: X_0..X_N  (N+1 states, each n_x)
    # Controls: U_0..U_{N-1} (N controls, each n_u)
    # Reference path points: P (parameter) — ref_x, ref_y, ref_psi for each step
    # Previous control: u_prev (parameter)
    X = ca.MX.sym('X', n_x, N + 1)
    U = ca.MX.sym('U', n_u, N)

    # Parameters: initial state (4) + prev_u (2) + reference per step (3 * N)
    n_p = n_x + n_u + 3 * N
    P = ca.MX.sym('P', n_p)

    x0_param = P[0:n_x]
    u_prev_param = P[n_x:n_x + n_u]

    # ── Build NLP ──
    cost = 0.0
    g = []        # constraints
    lbg = []
    ubg = []

    # Initial state constraint
    g.append(X[:, 0] - x0_param)
    lbg += [0.0] * n_x
    ubg += [0.0] * n_x

    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]
        Xk, Yk, psi_k, v_k = xk[0], xk[1], xk[2], xk[3]
        v_cmd_k, omega_k = uk[0], uk[1]

        # Reference for step k
        ref_offset = n_x + n_u + 3 * k
        ref_x = P[ref_offset]
        ref_y = P[ref_offset + 1]
        ref_psi = P[ref_offset + 2]

        # Cross-track error (signed)
        e_c = -ca.sin(ref_psi) * (Xk - ref_x) + ca.cos(ref_psi) * (Yk - ref_y)

        # Heading error
        e_psi = psi_k - ref_psi
        # Wrap heading error to [-pi, pi] using atan2(sin, cos)
        e_psi = ca.atan2(ca.sin(e_psi), ca.cos(e_psi))

        # ── Stage cost ──
        cost += w_cte * e_c ** 2
        cost += w_head * e_psi ** 2
        cost += w_v * (v_k - v_ref) ** 2

        # Control smoothness: penalize change from previous control
        if k == 0:
            du_v = v_cmd_k - u_prev_param[0]
            du_w = omega_k - u_prev_param[1]
        else:
            du_v = v_cmd_k - U[0, k - 1]
            du_w = omega_k - U[1, k - 1]
        cost += w_dv * du_v ** 2
        cost += w_dw * du_w ** 2

        # ── Dynamics constraint (Euler integration) ──
        x_next = ca.vertcat(
            Xk + dt * v_k * ca.cos(psi_k),
            Yk + dt * v_k * ca.sin(psi_k),
            psi_k + dt * omega_k,
            v_k + dt * (v_cmd_k - v_k) / tau_v
        )
        g.append(X[:, k + 1] - x_next)
        lbg += [0.0] * n_x
        ubg += [0.0] * n_x

        # ── Cross-track constraint: |e_c| <= row_half_width ──
        g.append(e_c)
        lbg.append(-row_half_width)
        ubg.append(row_half_width)

        # ── Wheel speed constraints: |v ± omega*b| <= v_wheel_max ──
        # v_R = v_k + omega_k * b <= v_wheel_max
        # v_R = v_k + omega_k * b >= -v_wheel_max
        # v_L = v_k - omega_k * b <= v_wheel_max
        # v_L = v_k - omega_k * b >= -v_wheel_max
        g.append(v_k + omega_k * b)
        lbg.append(-v_wheel_max)
        ubg.append(v_wheel_max)
        g.append(v_k - omega_k * b)
        lbg.append(-v_wheel_max)
        ubg.append(v_wheel_max)

        # ── Delta omega constraint ──
        if k == 0:
            dw = omega_k - u_prev_param[1]
        else:
            dw = omega_k - U[1, k - 1]
        g.append(dw)
        lbg.append(-delta_omega_max)
        ubg.append(delta_omega_max)

    # ── Flatten decision variables ──
    opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
    g = ca.vertcat(*g)

    nlp = {
        'x': opt_vars,
        'f': cost,
        'g': g,
        'p': P,
    }

    opts = {
        'ipopt.print_level': 0,
        'ipopt.sb': 'yes',
        'print_time': 0,
        'ipopt.max_iter': 100,
        'ipopt.warm_start_init_point': 'yes',
    }
    solver = ca.nlpsol('mpc', 'ipopt', nlp, opts)

    nlp_dims = {
        'n_x': n_x,
        'n_u': n_u,
        'N': N,
        'n_vars': opt_vars.shape[0],
        'n_g': g.shape[0],
        'n_p': n_p,
        'lbg': lbg,
        'ubg': ubg,
    }
    return solver, nlp_dims


def solve_mpc(solver, nlp_dims, params, x0, theta_current, prev_u,
              path_x, path_y, arc_lengths, warm_start=None):
    """
    Solve the MPC problem for one control step.

    Args:
        solver: CasADi solver from build_mpc_solver
        nlp_dims: dict from build_mpc_solver
        params: parameter dict
        x0: current state [X, Y, psi, v] (4,)
        theta_current: current arc-length on path
        prev_u: previous control [v_cmd, omega_cmd] (2,)
        path_x, path_y, arc_lengths: path data arrays
        warm_start: optional previous solution for warm-starting

    Returns:
        (v_cmd, omega_cmd, predicted_states): first control + full predicted trajectory (N+1, 4)
    """
    N = nlp_dims['N']
    n_x = nlp_dims['n_x']
    n_u = nlp_dims['n_u']
    dt = params['dt']
    v_ref = params['v_ref']
    omega_max = params['omega_max']
    v_wheel_max = params['v_wheel_max']

    # ── Build parameter vector: x0 + prev_u + references ──
    p_val = np.zeros(nlp_dims['n_p'])
    p_val[0:n_x] = x0
    p_val[n_x:n_x + n_u] = prev_u

    # Generate reference points along path for each prediction step
    theta_refs = np.zeros(N)
    for k in range(N):
        theta_refs[k] = theta_current + v_ref * dt * (k + 1)
        theta_refs[k] = min(theta_refs[k], arc_lengths[-1])

    for k in range(N):
        ref_x, ref_y, ref_psi, _ = get_path_state(
            theta_refs[k], path_x, path_y, arc_lengths
        )
        offset = n_x + n_u + 3 * k
        p_val[offset] = ref_x
        p_val[offset + 1] = ref_y
        p_val[offset + 2] = ref_psi

    # ── Variable bounds ──
    n_vars = nlp_dims['n_vars']
    lbx = -np.inf * np.ones(n_vars)
    ubx = np.inf * np.ones(n_vars)

    n_state_vars = n_x * (N + 1)

    # Control bounds: v_cmd in [-v_wheel_max, v_wheel_max], omega in [-omega_max, omega_max]
    for k in range(N):
        u_offset = n_state_vars + k * n_u
        lbx[u_offset] = -v_wheel_max   # v_cmd lower
        ubx[u_offset] = v_wheel_max     # v_cmd upper
        lbx[u_offset + 1] = -omega_max  # omega lower
        ubx[u_offset + 1] = omega_max   # omega upper

    # ── Initial guess ──
    x0_guess = np.zeros(n_vars)
    if warm_start is not None:
        x0_guess = warm_start
    else:
        # Simple initial guess: propagate x0 with v_ref, 0 omega
        for k in range(N + 1):
            idx = k * n_x
            x0_guess[idx] = x0[0] + v_ref * dt * k * np.cos(x0[2])
            x0_guess[idx + 1] = x0[1] + v_ref * dt * k * np.sin(x0[2])
            x0_guess[idx + 2] = x0[2]
            x0_guess[idx + 3] = v_ref
        for k in range(N):
            u_offset = n_state_vars + k * n_u
            x0_guess[u_offset] = v_ref
            x0_guess[u_offset + 1] = 0.0

    # ── Solve ──
    sol = solver(
        x0=x0_guess,
        p=p_val,
        lbx=lbx,
        ubx=ubx,
        lbg=nlp_dims['lbg'],
        ubg=nlp_dims['ubg'],
    )

    opt = np.array(sol['x']).flatten()

    # Extract predicted states
    predicted_states = np.zeros((N + 1, n_x))
    for k in range(N + 1):
        predicted_states[k, :] = opt[k * n_x:(k + 1) * n_x]

    # Extract first control
    u0_offset = n_state_vars
    v_cmd = float(opt[u0_offset])
    omega_cmd = float(opt[u0_offset + 1])

    return v_cmd, omega_cmd, predicted_states


# ── ROS2 Node ────────────────────────────────────────────────────────────
class ControllerMPC(Node):

    def __init__(self):
        super().__init__('controller_mpc')

        # ── Declare all parameters ──
        self.declare_parameter('N', 20)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('w_cte', 10.0)
        self.declare_parameter('w_head', 5.0)
        self.declare_parameter('w_v', 1.0)
        self.declare_parameter('w_dv', 2.0)
        self.declare_parameter('w_dw', 5.0)
        self.declare_parameter('v_wheel_max', 1.5)
        self.declare_parameter('omega_max', 1.0)
        self.declare_parameter('delta_omega_max', 0.5)
        self.declare_parameter('row_half_width', 0.45)
        self.declare_parameter('track_half_width', 0.435)
        self.declare_parameter('tau_v', 0.3)
        self.declare_parameter('v_ref', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('path_frame_id', 'utm')

        self.params = self._load_params()

        # ── Build solver ──
        self.get_logger().info('Building CasADi MPC solver...')
        self.solver, self.nlp_dims = build_mpc_solver(self.params)
        self.get_logger().info('MPC solver ready.')

        # ── State ──
        self.path_x = None
        self.path_y = None
        self.arc_lengths = None
        self.odom_state = None       # [X, Y, psi, v]
        self.odom_stamp = None
        self.enabled = False
        self.theta_current = 0.0
        self.prev_u = np.array([0.0, 0.0])
        self.warm_start = None

        # ── Subscribers ──
        self.create_subscription(
            Odometry, '/amiga/odom', self._odom_cb, 10)
        self.create_subscription(
            Path, '/farnav/global_path', self._path_cb, 10)
        self.create_subscription(
            Bool, '/farnav/controller_enable', self._enable_cb, 10)

        # ── Publishers ──
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/amiga/cmd_vel', 10)
        self.pred_path_pub = self.create_publisher(
            Path, '/farnav/mpc_predicted_path', 10)

        # ── Control timer ──
        rate = self.params['control_rate_hz']
        self.timer = self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'ControllerMPC started — rate: {rate} Hz, N: {self.params["N"]}, '
            f'dt: {self.params["dt"]}, v_ref: {self.params["v_ref"]}'
        )

    def _load_params(self):
        return {
            'N': self.get_parameter('N').get_parameter_value().integer_value,
            'dt': self.get_parameter('dt').get_parameter_value().double_value,
            'w_cte': self.get_parameter('w_cte').get_parameter_value().double_value,
            'w_head': self.get_parameter('w_head').get_parameter_value().double_value,
            'w_v': self.get_parameter('w_v').get_parameter_value().double_value,
            'w_dv': self.get_parameter('w_dv').get_parameter_value().double_value,
            'w_dw': self.get_parameter('w_dw').get_parameter_value().double_value,
            'v_wheel_max': self.get_parameter('v_wheel_max').get_parameter_value().double_value,
            'omega_max': self.get_parameter('omega_max').get_parameter_value().double_value,
            'delta_omega_max': self.get_parameter('delta_omega_max').get_parameter_value().double_value,
            'row_half_width': self.get_parameter('row_half_width').get_parameter_value().double_value,
            'track_half_width': self.get_parameter('track_half_width').get_parameter_value().double_value,
            'tau_v': self.get_parameter('tau_v').get_parameter_value().double_value,
            'v_ref': self.get_parameter('v_ref').get_parameter_value().double_value,
            'control_rate_hz': self.get_parameter('control_rate_hz').get_parameter_value().double_value,
            'path_frame_id': self.get_parameter('path_frame_id').get_parameter_value().string_value,
        }

    # ── Callbacks ────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        self.odom_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            vx,
        ])
        self.odom_stamp = msg.header.stamp

    def _path_cb(self, msg: Path):
        if len(msg.poses) < 2:
            return
        n = len(msg.poses)
        self.path_x = np.array([msg.poses[i].pose.position.x for i in range(n)])
        self.path_y = np.array([msg.poses[i].pose.position.y for i in range(n)])
        self.arc_lengths = np.array([msg.poses[i].pose.position.z for i in range(n)])
        self.theta_current = 0.0
        self.warm_start = None
        self.get_logger().info(f'Received path with {n} points, arc_length={self.arc_lengths[-1]:.1f}m')

    def _enable_cb(self, msg: Bool):
        self.enabled = msg.data
        if not self.enabled:
            self._publish_zero_cmd()
            self.warm_start = None
        self.get_logger().info(f'Controller {"ENABLED" if self.enabled else "DISABLED"}')

    # ── Control loop ─────────────────────────────────────────────────────
    def _control_loop(self):
        # Guard: need enable + odom + path
        if not self.enabled or self.odom_state is None or self.path_x is None:
            if self.enabled:
                self._publish_zero_cmd()
            return

        x0 = self.odom_state.copy()

        # Project robot onto path
        self.theta_current = project_onto_path(
            self.path_x, self.path_y, self.arc_lengths,
            x0[0], x0[1], self.theta_current
        )

        # Check if reached end of path
        if self.theta_current >= self.arc_lengths[-1] - 0.1:
            self.get_logger().info('Reached end of path, stopping.')
            self._publish_zero_cmd()
            return

        # Solve MPC
        try:
            v_cmd, omega_cmd, predicted_states = solve_mpc(
                self.solver, self.nlp_dims, self.params,
                x0, self.theta_current, self.prev_u,
                self.path_x, self.path_y, self.arc_lengths,
                warm_start=self.warm_start,
            )
        except Exception as e:
            self.get_logger().warn(f'MPC solve failed: {e}')
            self._publish_zero_cmd()
            return

        # Store for warm-start (shift solution by one step)
        n_x = self.nlp_dims['n_x']
        n_u = self.nlp_dims['n_u']
        N = self.nlp_dims['N']
        n_state_vars = n_x * (N + 1)

        # Build shifted warm start
        ws = np.zeros(self.nlp_dims['n_vars'])
        # Shift states: drop first, duplicate last
        for k in range(N):
            ws[k * n_x:(k + 1) * n_x] = predicted_states[k + 1, :]
        ws[N * n_x:(N + 1) * n_x] = predicted_states[N, :]
        # Shift controls: drop first, duplicate last
        # (need to extract from current solution first)
        # For simplicity, just reuse predicted trajectory as state warm start
        # and set controls to v_ref, 0
        for k in range(N - 1):
            ws_u = n_state_vars + k * n_u
            ws[ws_u] = v_cmd       # approximate
            ws[ws_u + 1] = omega_cmd
        ws[n_state_vars + (N - 1) * n_u] = v_cmd
        ws[n_state_vars + (N - 1) * n_u + 1] = omega_cmd
        self.warm_start = ws

        self.prev_u = np.array([v_cmd, omega_cmd])

        # Publish command
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v_cmd
        cmd.twist.angular.z = omega_cmd
        self.cmd_pub.publish(cmd)

        # Publish predicted path
        self._publish_predicted_path(predicted_states)

    def _publish_zero_cmd(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self.cmd_pub.publish(cmd)
        self.prev_u = np.array([0.0, 0.0])

    def _publish_predicted_path(self, predicted_states):
        path_msg = Path()
        stamp = self.get_clock().now().to_msg()
        frame_id = self.params['path_frame_id']
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = frame_id

        for k in range(predicted_states.shape[0]):
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = frame_id
            pose.pose.position.x = predicted_states[k, 0]
            pose.pose.position.y = predicted_states[k, 1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.pred_path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 4: Run tests to verify they pass**

Run: `cd /home/het/lab/amiga_project/src/farnav && python -m pytest test/test_controller_mpc.py -v`
Expected: All 5 tests PASS (solver build takes ~2s on first run)

**Step 5: Commit**

```bash
git add src/farnav/farnav/controller_mpc.py src/farnav/test/test_controller_mpc.py
git commit -m "feat(farnav): add MPC controller with CasADi/IPOPT solver and ROS2 node"
```

---

### Task 4: Launch File + Package Wiring

**Files:**
- Create: `src/farnav/launch/controller_mpc.launch.py`
- Modify: `src/farnav/setup.py` (add entry point + launch data_files)
- Modify: `src/farnav/package.xml` (no new deps needed — casadi is pip-only)

**Step 1: Create launch file**

```python
# src/farnav/launch/controller_mpc.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('farnav'),
        'config',
        'controller_mpc.yaml',
    )

    return LaunchDescription([
        Node(
            package='farnav',
            executable='controller_mpc',
            name='controller_mpc',
            output='screen',
            parameters=[config],
        ),
    ])
```

**Step 2: Update setup.py — add entry point and launch data_files**

In `src/farnav/setup.py`, make two changes:

Add to `data_files` list (after the config line):
```python
('share/' + package_name + '/launch', glob('launch/*.py')),
```

Add to `console_scripts` list:
```python
'controller_mpc = farnav.controller_mpc:main',
```

**Step 3: Build and verify**

Run: `cd /home/het/lab/amiga_project && colcon build --packages-select farnav --symlink-install`
Expected: Build succeeds

Run: `source install/setup.bash && ros2 run farnav controller_mpc --ros-args --params-file src/farnav/config/controller_mpc.yaml`
Expected: Node starts, prints "Building CasADi MPC solver...", "MPC solver ready.", waits for topics

**Step 4: Commit**

```bash
git add src/farnav/launch/controller_mpc.launch.py src/farnav/setup.py
git commit -m "feat(farnav): add controller_mpc launch file and package wiring"
```

---

### Task 5: Integration Test — Full Pipeline Smoke Test

**Files:**
- Create: `src/farnav/test/test_mpc_integration.py`

**Step 1: Write integration test — simulate straight path + offset robot**

```python
# src/farnav/test/test_mpc_integration.py
"""
Integration test: simulate MPC following a straight path.
No ROS2 needed — tests the full solve loop with path_utils + controller_mpc.
"""
import numpy as np
import pytest
from farnav.path_utils import project_onto_path, compute_errors
from farnav.controller_mpc import build_mpc_solver, solve_mpc


MPC_PARAMS = {
    'N': 20, 'dt': 0.1,
    'w_cte': 10.0, 'w_head': 5.0, 'w_v': 1.0, 'w_dv': 2.0, 'w_dw': 5.0,
    'v_wheel_max': 1.5, 'omega_max': 1.0, 'delta_omega_max': 0.5,
    'row_half_width': 0.45, 'track_half_width': 0.435, 'tau_v': 0.3,
    'v_ref': 0.5,
}


def _simulate_step(state, v_cmd, omega, dt, tau_v):
    """Propagate unicycle + velocity lag one step."""
    X, Y, psi, v = state
    X_new = X + dt * v * np.cos(psi)
    Y_new = Y + dt * v * np.sin(psi)
    psi_new = psi + dt * omega
    v_new = v + dt * (v_cmd - v) / tau_v
    return np.array([X_new, Y_new, psi_new, v_new])


class TestMPCClosedLoop:

    def setup_method(self):
        self.solver, self.nlp_dims = build_mpc_solver(MPC_PARAMS)
        # Straight path along X
        self.path_x = np.linspace(0.0, 100.0, 1000)
        self.path_y = np.zeros(1000)
        self.arc_lengths = np.linspace(0.0, 100.0, 1000)

    def test_converges_to_path(self):
        """Robot starts 0.3m offset, should converge to path within 5 seconds."""
        state = np.array([0.0, 0.3, 0.0, 0.0])
        prev_u = np.array([0.0, 0.0])
        theta = 0.0
        dt_sim = 0.05   # simulate at 20Hz
        warm = None

        for step in range(100):  # 5 seconds
            theta = project_onto_path(
                self.path_x, self.path_y, self.arc_lengths,
                state[0], state[1], theta
            )
            v_cmd, omega_cmd, pred = solve_mpc(
                self.solver, self.nlp_dims, MPC_PARAMS,
                state, theta, prev_u,
                self.path_x, self.path_y, self.arc_lengths,
                warm_start=warm,
            )
            state = _simulate_step(state, v_cmd, omega_cmd, dt_sim, MPC_PARAMS['tau_v'])
            prev_u = np.array([v_cmd, omega_cmd])

        e_c, e_psi = compute_errors(
            state[0], state[1], state[2], theta,
            self.path_x, self.path_y, self.arc_lengths
        )
        assert abs(e_c) < 0.05, f"CTE did not converge: {e_c:.3f}m"
        assert abs(e_psi) < 0.1, f"Heading did not converge: {e_psi:.3f}rad"
        assert state[3] > 0.3, f"Speed too low: {state[3]:.3f}m/s"

    def test_stays_within_row(self):
        """Robot starts 0.4m offset (near limit). Must never exceed row_half_width."""
        state = np.array([0.0, 0.4, 0.1, 0.0])
        prev_u = np.array([0.0, 0.0])
        theta = 0.0
        dt_sim = 0.05
        max_cte = 0.0

        for step in range(60):  # 3 seconds
            theta = project_onto_path(
                self.path_x, self.path_y, self.arc_lengths,
                state[0], state[1], theta
            )
            v_cmd, omega_cmd, pred = solve_mpc(
                self.solver, self.nlp_dims, MPC_PARAMS,
                state, theta, prev_u,
                self.path_x, self.path_y, self.arc_lengths,
            )
            state = _simulate_step(state, v_cmd, omega_cmd, dt_sim, MPC_PARAMS['tau_v'])
            prev_u = np.array([v_cmd, omega_cmd])

            e_c, _ = compute_errors(
                state[0], state[1], state[2], theta,
                self.path_x, self.path_y, self.arc_lengths
            )
            max_cte = max(max_cte, abs(e_c))

        assert max_cte < 0.5, f"Exceeded row width: max CTE = {max_cte:.3f}m"
```

**Step 2: Run integration test**

Run: `cd /home/het/lab/amiga_project/src/farnav && python -m pytest test/test_mpc_integration.py -v -s`
Expected: Both tests PASS (may take ~10s due to repeated solves)

**Step 3: Run all tests**

Run: `cd /home/het/lab/amiga_project/src/farnav && python -m pytest test/ -v --ignore=test/test_copyright.py --ignore=test/test_flake8.py --ignore=test/test_pep257.py`
Expected: All 18 tests PASS

**Step 4: Commit**

```bash
git add src/farnav/test/test_mpc_integration.py
git commit -m "test(farnav): add MPC closed-loop integration tests"
```

---

Plan complete and saved to `docs/plans/2026-03-08-controller-mpc-plan.md`. Two execution options:

**1. Subagent-Driven (this session)** — I dispatch fresh subagent per task, review between tasks, fast iteration

**2. Parallel Session (separate)** — Open new session with executing-plans, batch execution with checkpoints

Which approach?