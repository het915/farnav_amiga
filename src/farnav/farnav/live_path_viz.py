#!/usr/bin/env python3
"""
Live visualization of global planner reference path, MPC predicted path,
and robot odometry trail.

Usage:
    ros2 run farnav live_path_viz
    ros2 run farnav live_path_viz --ros-args -p follow_robot:=true
    ros2 run farnav live_path_viz --ros-args -p headless:=true -p save_dir:=/tmp/viz
"""
import os
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny, cosy))


class PathVizNode(Node):
    def __init__(self):
        super().__init__("live_path_viz")

        # parameters
        self.declare_parameter("follow_robot", False)
        self.declare_parameter("follow_radius", 6.0)
        self.declare_parameter("update_ms", 100)
        self.declare_parameter("trail_max", 2000)
        self.declare_parameter("headless", False)
        self.declare_parameter("save_dir", "/tmp/farnav_viz")
        self.declare_parameter("save_every_n", 10)

        self.follow_robot = self.get_parameter("follow_robot").value
        self.follow_radius = self.get_parameter("follow_radius").value
        self.update_ms = self.get_parameter("update_ms").value
        self.trail_max = self.get_parameter("trail_max").value
        self.headless = self.get_parameter("headless").value
        self.save_dir = self.get_parameter("save_dir").value
        self.save_every_n = self.get_parameter("save_every_n").value

        # data buffers
        self.lock = threading.Lock()
        self.global_path_xy = np.empty((0, 2))
        self.mpc_path_xy = np.empty((0, 2))
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        self.odom_trail_x = []
        self.odom_trail_y = []
        self.cross_track_error = 0.0
        self.heading_error = 0.0
        self.frame_count = 0

        # subscribers
        self.create_subscription(Path, "/farnav/global_path", self._global_path_cb, 10)
        self.create_subscription(Path, "/farnav/mpc_predicted_path", self._mpc_path_cb, 10)
        self.create_subscription(Odometry, "/amiga/odom", self._odom_cb, 10)

        if self.headless:
            os.makedirs(self.save_dir, exist_ok=True)
            self.get_logger().info(f"Headless mode — saving frames to {self.save_dir}")

        self.get_logger().info(
            f"live_path_viz started (follow={self.follow_robot}, "
            f"headless={self.headless}, interval={self.update_ms}ms)"
        )

    def _global_path_cb(self, msg: Path):
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        with self.lock:
            self.global_path_xy = pts

    def _mpc_path_cb(self, msg: Path):
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        with self.lock:
            self.mpc_path_xy = pts

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        with self.lock:
            self.robot_x = pos.x
            self.robot_y = pos.y
            self.robot_yaw = yaw
            self.robot_vx = vx
            self.robot_wz = wz
            self.odom_trail_x.append(pos.x)
            self.odom_trail_y.append(pos.y)
            if len(self.odom_trail_x) > self.trail_max:
                self.odom_trail_x = self.odom_trail_x[-self.trail_max:]
                self.odom_trail_y = self.odom_trail_y[-self.trail_max:]

            # cross-track + heading error
            if self.global_path_xy.shape[0] > 1:
                diffs = self.global_path_xy - np.array([pos.x, pos.y])
                dists = np.linalg.norm(diffs, axis=1)
                idx = np.argmin(dists)
                self.cross_track_error = dists[idx]
                if idx < len(self.global_path_xy) - 1:
                    dx = self.global_path_xy[idx + 1, 0] - self.global_path_xy[idx, 0]
                    dy = self.global_path_xy[idx + 1, 1] - self.global_path_xy[idx, 1]
                    path_yaw = np.arctan2(dy, dx)
                    self.heading_error = (yaw - path_yaw + np.pi) % (2 * np.pi) - np.pi


def run_viz(node: PathVizNode):
    """Set up matplotlib and run animation loop."""
    if node.headless:
        matplotlib.use("Agg")

    fig = plt.figure(figsize=(15, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=[3, 1], hspace=0.35, wspace=0.25)
    ax_path = fig.add_subplot(gs[:, 0])
    ax_err = fig.add_subplot(gs[0, 1])
    ax_info = fig.add_subplot(gs[1, 1])

    fig.suptitle("FarNav Live Path Visualization", fontsize=13, fontweight="bold")

    # --- path axes ---
    (ln_global,) = ax_path.plot([], [], "g-", lw=2.5, label="Reference Path", zorder=2)
    (ln_mpc,) = ax_path.plot([], [], "r-", lw=2.5, label="MPC Predicted", zorder=3)
    (ln_trail,) = ax_path.plot([], [], "b-", lw=1.2, alpha=0.4, label="Odom Trail", zorder=1)
    (ln_robot,) = ax_path.plot([], [], "ko", ms=10, zorder=5)
    arrow = ax_path.annotate(
        "", xy=(0, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle="->", color="black", lw=2.5),
    )
    ax_path.set_xlabel("X (m)")
    ax_path.set_ylabel("Y (m)")
    ax_path.set_aspect("equal")
    ax_path.legend(loc="upper left", fontsize=9)
    ax_path.grid(True, alpha=0.3)

    # --- error bar chart ---
    bar_labels = ["CTE (m)", "Heading (°)"]
    bars = ax_err.bar(bar_labels, [0, 0], color=["#e74c3c", "#3498db"], width=0.5)
    ax_err.set_ylim(0, 1.0)
    ax_err.set_title("Tracking Errors", fontsize=10)
    ax_err.grid(True, alpha=0.3, axis="y")
    err_text = ax_err.text(
        0.5, 0.92, "", transform=ax_err.transAxes,
        ha="center", va="top", fontsize=10,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.7),
    )

    # --- info panel ---
    ax_info.axis("off")
    info_text = ax_info.text(
        0.05, 0.95, "", transform=ax_info.transAxes,
        va="top", fontsize=10, family="monospace",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.9),
    )

    def update(_frame):
        with node.lock:
            gp = node.global_path_xy.copy()
            mp = node.mpc_path_xy.copy()
            tx = list(node.odom_trail_x)
            ty = list(node.odom_trail_y)
            rx, ry, ryaw = node.robot_x, node.robot_y, node.robot_yaw
            vx, wz = node.robot_vx, node.robot_wz
            cte = node.cross_track_error
            he = node.heading_error

        # path lines
        if gp.shape[0] > 0:
            ln_global.set_data(gp[:, 0], gp[:, 1])
        if mp.shape[0] > 0:
            ln_mpc.set_data(mp[:, 0], mp[:, 1])
        if tx:
            ln_trail.set_data(tx, ty)

        # robot marker + heading arrow
        ln_robot.set_data([rx], [ry])
        alen = 0.8
        arrow.xy = (rx + alen * np.cos(ryaw), ry + alen * np.sin(ryaw))
        arrow.set_position((rx, ry))

        # axis limits
        if node.follow_robot:
            r = node.follow_radius
            ax_path.set_xlim(rx - r, rx + r)
            ax_path.set_ylim(ry - r, ry + r)
        elif gp.shape[0] > 0:
            pad = 8.0
            all_x = np.concatenate([gp[:, 0], [rx]])
            all_y = np.concatenate([gp[:, 1], [ry]])
            ax_path.set_xlim(all_x.min() - pad, all_x.max() + pad)
            ax_path.set_ylim(all_y.min() - pad, all_y.max() + pad)

        # error bars
        he_deg = abs(np.degrees(he))
        bars[0].set_height(cte)
        bars[1].set_height(he_deg)
        ax_err.set_ylim(0, max(cte, he_deg, 0.5) * 1.4)
        err_text.set_text(f"CTE: {cte:.3f} m\nHead: {he_deg:.1f}°")

        # info panel
        progress = ""
        if gp.shape[0] > 1 and tx:
            diffs = gp - np.array([rx, ry])
            idx = np.argmin(np.linalg.norm(diffs, axis=1))
            pct = idx / (gp.shape[0] - 1) * 100
            progress = f"Progress:  {pct:.0f}%\n"

        info_text.set_text(
            f"Robot:     ({rx:.2f}, {ry:.2f})\n"
            f"Yaw:       {np.degrees(ryaw):.1f}°\n"
            f"Speed:     {vx:.2f} m/s\n"
            f"Omega:     {np.degrees(wz):.1f} °/s\n"
            f"{progress}"
            f"Trail pts: {len(tx)}\n"
            f"MPC pts:   {mp.shape[0]}"
        )

        # headless frame saving
        if node.headless:
            node.frame_count += 1
            if node.frame_count % node.save_every_n == 0:
                path = os.path.join(node.save_dir, f"frame_{node.frame_count:06d}.png")
                fig.savefig(path, dpi=100, bbox_inches="tight")

        return ln_global, ln_mpc, ln_trail, ln_robot, arrow

    _ani = FuncAnimation(fig, update, interval=node.update_ms, blit=False, cache_frame_data=False)

    plt.tight_layout()
    if node.headless:
        # in headless mode, drive the animation manually via a ROS timer
        node.get_logger().info("Headless: using timer-driven rendering")
        def _tick():
            update(None)
        node.create_timer(node.update_ms / 1000.0, _tick)
    else:
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PathVizNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_viz(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
