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
        assert abs(psi - 0.0) < 0.05

    def test_curvature_straight_path(self):
        _, _, _, kappa = get_path_state(
            5.0, self.path_x, self.path_y, self.arc_lengths
        )
        assert abs(kappa) < 0.01

    def test_clamp_beyond_end(self):
        x, y, _, _ = get_path_state(
            15.0, self.path_x, self.path_y, self.arc_lengths
        )
        assert abs(x - 10.0) < 0.15


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
        assert abs(e_c - 1.0) < 0.15

    def test_heading_error(self):
        _, e_psi = compute_errors(
            robot_x=5.0, robot_y=0.0, robot_psi=0.5,
            theta=5.0,
            path_x=self.path_x, path_y=self.path_y,
            arc_lengths=self.arc_lengths
        )
        assert abs(e_psi - 0.5) < 0.05
