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
        """Robot is 0.3m left of path, heading along path. Should steer right."""
        x0 = np.array([5.0, 0.3, 0.0, 0.0])
        theta_current = 5.0
        prev_u = np.array([0.0, 0.0])

        v_cmd, omega_cmd, predicted_states = solve_mpc(
            self.solver, self.nlp_dims, self.params,
            x0, theta_current, prev_u,
            self.path_x, self.path_y, self.arc_lengths
        )
        assert v_cmd > 0.1
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
        assert abs(omega_cmd) < 0.1

    def test_heading_error_corrects(self):
        """Robot on path but heading 30 deg off. Should steer to correct."""
        x0 = np.array([5.0, 0.0, 0.5, 0.3])
        theta_current = 5.0
        prev_u = np.array([0.3, 0.0])

        v_cmd, omega_cmd, predicted_states = solve_mpc(
            self.solver, self.nlp_dims, self.params,
            x0, theta_current, prev_u,
            self.path_x, self.path_y, self.arc_lengths
        )
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
        assert predicted_states.shape == (21, 4)
