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
        e_psi = ca.atan2(ca.sin(e_psi), ca.cos(e_psi))

        # ── Stage cost ──
        cost += w_cte * e_c ** 2
        cost += w_head * e_psi ** 2
        cost += w_v * (v_k - v_ref) ** 2

        # Control smoothness
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

        # ── Cross-track constraint ──
        g.append(e_c)
        lbg.append(-row_half_width)
        ubg.append(row_half_width)

        # ── Wheel speed constraints ──
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

    # Control bounds — enforce minimum forward velocity to prevent
    # skid-steer spin-in-place (robot shakes violently at v≈0 + high omega)
    v_min = 0.1  # minimum forward speed (m/s)
    for k in range(N):
        u_offset = n_state_vars + k * n_u
        lbx[u_offset] = v_min            # v_cmd lower (no reverse, no near-zero)
        ubx[u_offset] = v_wheel_max      # v_cmd upper
        lbx[u_offset + 1] = -omega_max   # omega lower
        ubx[u_offset + 1] = omega_max    # omega upper

    # ── Initial guess ──
    x0_guess = np.zeros(n_vars)
    if warm_start is not None:
        x0_guess = warm_start
    else:
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
        self._enable_time = None      # timestamp when controller was enabled
        self._ramp_duration = 3.0     # seconds to ramp from v_min to v_ref

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
        new_x = np.array([msg.poses[i].pose.position.x for i in range(n)])
        new_y = np.array([msg.poses[i].pose.position.y for i in range(n)])
        new_arc = np.array([msg.poses[i].pose.position.z for i in range(n)])

        # Only reset theta if this is a genuinely new path (different endpoint or length)
        is_new_path = (self.path_x is None or
                       len(new_x) != len(self.path_x) or
                       abs(new_arc[-1] - self.arc_lengths[-1]) > 0.5)

        self.path_x = new_x
        self.path_y = new_y
        self.arc_lengths = new_arc

        if is_new_path:
            self.theta_current = 0.0
            self.warm_start = None
            self.get_logger().info(f'NEW path received: {n} points, arc_length={self.arc_lengths[-1]:.1f}m')

    def _enable_cb(self, msg: Bool):
        self.enabled = msg.data
        if not self.enabled:
            self._publish_zero_cmd()
            self.warm_start = None
        if self.enabled:
            self._enable_time = self.get_clock().now()
        else:
            self._enable_time = None
        self.get_logger().info(f'Controller {"ENABLED" if self.enabled else "DISABLED"}')

    # ── Control loop ─────────────────────────────────────────────────────
    def _control_loop(self):
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

        total_length = self.arc_lengths[-1]
        remaining = total_length - self.theta_current

        # Compute errors for logging
        e_c, e_psi = compute_errors(
            x0[0], x0[1], x0[2], self.theta_current,
            self.path_x, self.path_y, self.arc_lengths
        )

        # Distance from robot to path end point
        dist_to_end = np.sqrt(
            (x0[0] - self.path_x[-1])**2 + (x0[1] - self.path_y[-1])**2
        )

        # Get reference state at current theta for logging
        ref_x, ref_y, ref_psi, _ = get_path_state(
            self.theta_current, self.path_x, self.path_y, self.arc_lengths
        )

        # Log every 20th cycle (~1 Hz at 20 Hz control rate)
        self._loop_count = getattr(self, '_loop_count', 0) + 1
        if self._loop_count % 20 == 0:
            self.get_logger().info(
                f'[STATE] robot=({x0[0]:.2f},{x0[1]:.2f}) yaw={np.degrees(x0[2]):.1f}deg v={x0[3]:.2f}m/s'
            )
            self.get_logger().info(
                f'[REF]   ref=({ref_x:.2f},{ref_y:.2f}) ref_yaw={np.degrees(ref_psi):.1f}deg | '
                f'theta={self.theta_current:.2f}/{total_length:.2f}m'
            )
            self.get_logger().info(
                f'[ERROR] e_c={e_c:.3f}m e_psi={np.degrees(e_psi):.1f}deg | '
                f'dist_to_ref={np.sqrt((x0[0]-ref_x)**2+(x0[1]-ref_y)**2):.2f}m | '
                f'dist_to_B={dist_to_end:.2f}m rem={remaining:.2f}m'
            )
            self.get_logger().info(
                f'[PATH]  start=({self.path_x[0]:.2f},{self.path_y[0]:.2f}) '
                f'end=({self.path_x[-1]:.2f},{self.path_y[-1]:.2f}) | '
                f'path_dir={np.degrees(np.arctan2(self.path_y[-1]-self.path_y[0], self.path_x[-1]-self.path_x[0])):.1f}deg'
            )

        # Check if reached end of path — use both arc-length AND distance to end
        if remaining < 0.5 or dist_to_end < 1.0:
            self.get_logger().info(
                f'Reached end of path! remaining={remaining:.2f}m dist_to_B={dist_to_end:.2f}m. Stopping.'
            )
            self._publish_zero_cmd()
            self.enabled = False
            return

        # Velocity ramp-up: start slow so heading corrects gently (no skid-steer shaking)
        effective_params = dict(self.params)
        if self._enable_time is not None:
            elapsed = (self.get_clock().now() - self._enable_time).nanoseconds / 1e9
            if elapsed < self._ramp_duration:
                v_min = 0.1
                ramp_frac = elapsed / self._ramp_duration
                effective_params['v_ref'] = v_min + (self.params['v_ref'] - v_min) * ramp_frac

        # Ramp down speed near end of path for smooth deceleration
        decel_zone = 3.0
        if remaining < decel_zone:
            scale = max(remaining / decel_zone, 0.1)
            effective_params['v_ref'] = effective_params['v_ref'] * scale

        # Solve MPC
        try:
            v_cmd, omega_cmd, predicted_states = solve_mpc(
                self.solver, self.nlp_dims, effective_params,
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

        ws = np.zeros(self.nlp_dims['n_vars'])
        for k in range(N):
            ws[k * n_x:(k + 1) * n_x] = predicted_states[k + 1, :]
        ws[N * n_x:(N + 1) * n_x] = predicted_states[N, :]
        for k in range(N - 1):
            ws_u = n_state_vars + k * n_u
            ws[ws_u] = v_cmd
            ws[ws_u + 1] = omega_cmd
        ws[n_state_vars + (N - 1) * n_u] = v_cmd
        ws[n_state_vars + (N - 1) * n_u + 1] = omega_cmd
        self.warm_start = ws

        self.prev_u = np.array([v_cmd, omega_cmd])

        # Log MPC output every 20th cycle
        if self._loop_count % 20 == 0:
            pred_end = predicted_states[-1]
            pred_dir = np.degrees(np.arctan2(
                predicted_states[-1, 1] - predicted_states[0, 1],
                predicted_states[-1, 0] - predicted_states[0, 0]
            ))
            self.get_logger().info(
                f'[MPC CMD] v_cmd={v_cmd:.3f}m/s omega={omega_cmd:.4f}rad/s | '
                f'v_ref_eff={effective_params["v_ref"]:.3f}'
            )
            self.get_logger().info(
                f'[PRED]  pred_end=({pred_end[0]:.2f},{pred_end[1]:.2f}) '
                f'pred_yaw={np.degrees(pred_end[2]):.1f}deg pred_v={pred_end[3]:.2f}m/s | '
                f'pred_dir={pred_dir:.1f}deg'
            )

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
