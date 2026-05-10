# FarNav Amiga

Autonomous crop-row navigation system for the [Farm-ng Amiga](https://farm-ng.com/products/la-maquina-amiga) skid-steer robot, built on ROS 2 Humble.

The system consists of three components: a **gRPC-to-ROS 2 bridge** for robot communication, a **GPS-based global path planner**, and a **Model Predictive Controller (MPC)** for precise row following.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Amiga Robot (gRPC)                          │
│   CAN bus:6001  │  OAK cameras:50010  │  GPS:50010  │  Filter:20001│
└────────┬────────────────┬──────────────────┬──────────────┬────────┘
         │                │                  │              │
         ▼                ▼                  ▼              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      ROS 2 Bridge                                   │
│  /amiga/twist  /amiga/odom  /amiga/gps  /amiga/imu  /oak/rgb  /tf  │
└────────┬────────────────┬──────────────────┬────────────────────────┘
         │                │                  │
         ▼                ▼                  ▼
┌──────────────────┐  ┌─────────────────────────┐
│  Global Planner  │  │     MPC Controller      │
│  GPS → cubic     │──│  N=20 horizon, 20 Hz    │──► /cmd_vel
│  spline path     │  │  CasADi/IPOPT solver    │
└──────────────────┘  └─────────────────────────┘
```

## Features

**ROS 2 Bridge**
- Bridges all Amiga gRPC services to standard ROS 2 topics
- Streams: CAN bus (16 Hz), OAK stereo cameras, GPS, IMU, odometry
- Bidirectional: receives sensor data, sends velocity commands
- Configurable throttling, TF publishing, RViz 2 preset

**Global Path Planner**
- Converts lat/lon GPS waypoints to cubic spline paths in odom frame
- Automatic GPS/odom alignment via UTM coordinate transform
- Arc-length parameterized output for MPC consumption
- Multi-row YAML config with A-start, waypoints, B-end per row

**MPC Controller**
- Receding horizon optimal control (N=20 steps, 2s lookahead)
- Unicycle dynamics with first-order velocity lag model
- Cost function: cross-track error, heading, velocity tracking, smoothness
- Constraints: row width (0.45m), wheel speed, angular rate limits
- CasADi/IPOPT NLP solver with warm-starting
- End-of-path deceleration and smooth stop

## Quick Start

### Prerequisites

- Ubuntu 22.04, ROS 2 Humble, Python 3.10
- `farm-ng-amiga`, `farm-ng-core` (gRPC SDK)
- `casadi`, `scipy`, `pyproj`, `numpy`

### Build

```bash
cd ~/lab/amiga_project
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
pip install -r requirement.txt
```

### Run

**Terminal 1 — Bridge:**
```bash
ros2 launch amiga_ros2_bridge amiga_bridge.launch.py host:=172.16.107.128
```

**Terminal 2 — Global Planner:**
```bash
ros2 run farnav global_planner --ros-args --params-file src/farnav/config/row_data.yaml
```

**Terminal 3 — MPC Controller:**
```bash
ros2 launch farnav controller_mpc.launch.py
```

**Terminal 4 — Enable & Monitor:**
```bash
ros2 topic pub /farnav/controller_enable std_msgs/msg/Bool "{data: true}" --once
```

Set the pendant to **AUTO** mode. The robot transitions AUTO_READY → AUTO_ACTIVE on the first command.

### Visualize

```bash
rviz2 -d src/amiga_ros2_bridge/config/amiga_bridge.rviz
```

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/amiga/twist` | TwistStamped | Measured velocity from CAN bus |
| `/amiga/odom` | Odometry | Filtered pose + velocity |
| `/amiga/gps` | NavSatFix | GPS position |
| `/amiga/imu` | Imu | Accelerometer + gyroscope |
| `/amiga/oak/rgb/compressed` | CompressedImage | RGB camera stream |
| `/amiga/battery` | Float32 | Battery charge level |
| `/amiga/control_state` | UInt32 | Pendant state (4=AUTO_READY, 5=AUTO_ACTIVE) |
| `/cmd_vel` | Twist | Velocity commands (input) |
| `/farnav/global_path` | Path | Cubic spline path from planner |
| `/farnav/mpc_predicted_path` | Path | MPC predicted trajectory |
| `/tf` | TransformStamped | odom → base_link |

## Project Structure

```
├── src/
│   ├── amiga_ros2_bridge/       # gRPC ↔ ROS 2 bridge
│   │   ├── amiga_bridge_node.py # Combined bridge node
│   │   ├── cmd_vel_node.py      # Velocity command node
│   │   ├── streams_node.py      # Sensor streams node
│   │   ├── config/              # Bridge YAML + RViz config
│   │   └── launch/              # Launch files
│   └── farnav/                  # Navigation package
│       ├── controller_mpc.py    # MPC trajectory tracker
│       ├── global_planner.py    # GPS waypoint → spline path
│       ├── path_utils.py        # Path projection & error math
│       ├── path_overlay_node.py # Camera overlay visualization
│       ├── config/              # MPC params + row waypoints
│       ├── launch/              # Launch files
│       └── test/                # Unit + integration tests
├── scripts/                     # Manual gRPC test scripts
├── presentations/               # Project presentations
└── docs/                        # Design docs, diagrams, setup guide
```

## Tests

```bash
cd src/farnav
python -m pytest test/ -v
```

Covers path projection, error computation, MPC solver construction, and closed-loop convergence.

## Configuration

**Bridge** — `src/amiga_ros2_bridge/config/amiga_bridge.yaml`
- Robot host/ports, stream enable/disable, publish throttling, TF frames

**MPC** — `src/farnav/config/controller_mpc.yaml`
- Horizon (N, dt), cost weights, constraints, vehicle params, cruise speed

**Waypoints** — `src/farnav/config/row_data.yaml`
- GPS coordinates for crop rows (A-start, intermediates, B-end)

## License

See [LICENSE](LICENSE) for details.
