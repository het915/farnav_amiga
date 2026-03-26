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
        dt_sim = 0.05
        warm = None

        for step in range(100):
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

        for step in range(60):
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
