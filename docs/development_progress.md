# FarNav Amiga — Development Progress & Documentation

**Project:** Autonomous Crop-Row Navigation for Farm-ng Amiga
**Platform:** ROS 2 Humble | Ubuntu 22.04 | Python 3.10
**Robot:** Farm-ng Amiga skid-steer (hostname: lavender-latency, IP: 172.16.107.128)
**Repository:** [github.com/het915/farnav_amiga](https://github.com/het915/farnav_amiga)
**Last Updated:** March 2026

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Phase 1: gRPC-to-ROS 2 Bridge](#3-phase-1-grpc-to-ros-2-bridge)
4. [Phase 2: Global Path Planner](#4-phase-2-global-path-planner)
5. [Phase 3: MPC Controller](#5-phase-3-mpc-controller)
6. [Phase 4: Testing & Validation](#6-phase-4-testing--validation)
7. [Phase 5: Visualization & Utilities](#7-phase-5-visualization--utilities)
8. [Phase 6: Documentation & Repository Organization](#8-phase-6-documentation--repository-organization)
9. [Configuration Reference](#9-configuration-reference)
10. [ROS 2 Topic & Service Reference](#10-ros-2-topic--service-reference)
11. [Known Issues & Resolutions](#11-known-issues--resolutions)
12. [Design Decisions & Rationale](#12-design-decisions--rationale)
13. [Future Work](#13-future-work)

---

## 1. Project Overview

FarNav Amiga is an autonomous crop-row navigation system for the Farm-ng Amiga skid-steer robot. The goal is to autonomously follow crop rows using GPS waypoints and real-time MPC control.

The system has three core components:

| Component | Purpose | Key File |
|-----------|---------|----------|
| **gRPC-to-ROS 2 Bridge** | Connects Amiga hardware to ROS 2 ecosystem | `amiga_bridge_node.py` |
| **Global Path Planner** | Converts GPS waypoints to smooth spline paths | `global_planner.py` |
| **MPC Controller** | Tracks the planned path with optimal control | `controller_mpc.py` |

### Project Structure

```
amiga_project/
├── src/
│   ├── amiga_ros2_bridge/           # Package 1: Hardware bridge
│   │   ├── amiga_ros2_bridge/
│   │   │   ├── amiga_bridge_node.py # Combined bridge (streams + cmd_vel)
│   │   │   ├── streams_node.py      # Sensor streams only (legacy)
│   │   │   └── cmd_vel_node.py      # Velocity commands only (legacy)
│   │   ├── config/
│   │   │   ├── amiga_bridge.yaml    # Bridge parameters
│   │   │   └── amiga_bridge.rviz    # RViz2 visualization preset
│   │   └── launch/
│   │       └── amiga_bridge.launch.py
│   │
│   └── farnav/                      # Package 2: Navigation & control
│       ├── farnav/
│       │   ├── controller_mpc.py    # MPC trajectory tracker
│       │   ├── global_planner.py    # GPS waypoint → spline planner
│       │   ├── path_utils.py        # Path math (no ROS deps)
│       │   ├── gps_odom_node.py     # Fallback GPS odometry
│       │   ├── path_overlay_node.py # Camera path overlay
│       │   └── live_path_viz.py     # Matplotlib path debug
│       ├── config/
│       │   ├── controller_mpc.yaml  # MPC tuning parameters
│       │   └── row_data.yaml        # GPS waypoint definitions
│       ├── launch/
│       │   └── controller_mpc.launch.py
│       └── test/                    # Unit & integration tests
│           ├── test_path_utils.py
│           ├── test_controller_mpc.py
│           └── test_mpc_integration.py
│
├── scripts/                         # Manual gRPC test scripts
├── presentations/                   # Project slides (pptx/pdf)
└── docs/                            # Design docs, diagrams, guides
```

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Amiga Robot (gRPC Services)                  │
│   CAN bus : 6001  │  OAK cameras : 50010  │  GPS : 50010  │  Filter : 20001  │
└────────┬────────────────┬──────────────────┬──────────────┬────────┘
         │                │                  │              │
         ▼                ▼                  ▼              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      amiga_bridge_node (ROS 2)                      │
│                                                                     │
│  Publishers:                      Subscribers:                      │
│  /amiga/twist     (TwistStamped)  /cmd_vel       (Twist)           │
│  /amiga/odom      (Odometry)      /amiga/cmd_vel (TwistStamped)    │
│  /amiga/gps       (NavSatFix)                                      │
│  /amiga/imu       (Imu)          TF Broadcaster:                   │
│  /amiga/oak/*/compressed          odom → base_link                 │
│  /amiga/battery   (Float32)                                        │
│  /amiga/control_state (UInt32)                                     │
└────────┬────────────────────────────┬──────────────────────────────┘
         │                            │
         ▼                            ▼
┌──────────────────┐         ┌─────────────────────────┐
│  Global Planner  │         │     MPC Controller      │
│                  │         │                         │
│  Subscribes:     │         │  Subscribes:            │
│  /amiga/odom     │ ──────► │  /farnav/global_path    │
│  /amiga/gps      │  path   │  /amiga/odom            │
│                  │         │  /farnav/controller_    │
│  Publishes:      │         │    enable               │
│  /farnav/        │         │                         │
│   global_path    │         │  Publishes:             │
│  /farnav/        │         │  /amiga/cmd_vel         │
│   path_info      │         │  /farnav/mpc_           │
│  /farnav/        │         │   predicted_path        │
│   active_row_    │         └────────────┬────────────┘
│   marker         │                      │
└──────────────────┘                      ▼
                                   gRPC /twist → Amiga motors
```

### Data Flow

1. **Sensors → Bridge:** Amiga gRPC services stream CAN bus, camera, GPS, and filter data. The bridge converts these to standard ROS 2 messages.
2. **Bridge → Planner:** The global planner subscribes to `/amiga/gps` and `/amiga/odom` to align GPS waypoints with the robot's local coordinate frame.
3. **Planner → Controller:** The planner publishes a cubic spline path on `/farnav/global_path`. The MPC controller receives this and the robot's current odometry.
4. **Controller → Bridge → Robot:** The MPC computes optimal velocity commands at 20 Hz, published to `/amiga/cmd_vel`, which the bridge forwards to the Amiga via gRPC `/twist`.

### TF Frame Tree

```
odom
 └── base_link
      ├── oak_link   (camera)
      └── gps_link   (GPS antenna)
```

---

## 3. Phase 1: gRPC-to-ROS 2 Bridge

**Status: COMPLETE**
**Package:** `amiga_ros2_bridge`
**Key Files:** `amiga_bridge_node.py`, `streams_node.py`, `cmd_vel_node.py`

### 3.1 Objective

Create a bidirectional bridge between the Amiga's gRPC services and the ROS 2 ecosystem. All sensor data streams into ROS 2 topics; velocity commands flow back to the robot.

### 3.2 Development Steps

1. **gRPC API study** — Studied farm-ng-core and farm-ng-amiga Python SDK. Identified EventClient pattern for streaming and request-reply.
2. **Port discovery** — Default ports from farm-ng docs were wrong for our robot. Performed full port scan to discover actual service ports:
   - Canbus: 6001
   - OAK + GPS: 50010 (shared port, different service names)
   - Filter: 20001
3. **streams_node.py** — Initial implementation streaming only sensor data (canbus, OAK cameras, GPS, filter, IMU). Each stream runs as an async task in a background thread.
4. **cmd_vel_node.py** — Separate node handling velocity command forwarding. Subscribes to both `/cmd_vel` (Twist) and `/amiga/cmd_vel` (TwistStamped).
5. **Combined bridge** — Merged streams and cmd_vel into a single `amiga_bridge_node.py` for simpler deployment. Legacy nodes kept for flexibility.
6. **Bug fixes** — Multiple endpoint corrections discovered during live testing (see Section 11).
7. **Configuration** — YAML config with all ports, service names, enable flags, throttling, and TF frame IDs. Launch file with host override argument.
8. **RViz2 preset** — Pre-configured `.rviz` file with TF, odometry trail, camera feeds, GPS, and IMU displays.

### 3.3 gRPC Service Mapping

| Service | Port | Service Name | Stream Paths | Status |
|---------|------|--------------|-------------|--------|
| Canbus | 6001 | canbus | `/state`, `/twist` | Working |
| OAK Cameras | 50010 | oak/0..oak/3 | `/left`, `/rgb`, `/right`, `/disparity`, `/imu` | Working |
| GPS | 50010 | gps | `/pvt`, `/ecef`, `/relposned` | Working |
| Filter | 20001 | filter | `/state` | Working |
| Track Follower | 20101 | track_follower | — | Available |
| System Monitor | 20201 | system_monitor | — | Available |

### 3.4 Key Implementation Details

- **Async architecture:** gRPC streams run in a background asyncio event loop (daemon thread) to avoid blocking ROS 2 spin.
- **Throttling:** Configurable `every_n` parameter per stream to reduce bandwidth (e.g., skip every other camera frame).
- **TF broadcasting:** Publishes `odom → base_link` transform from filter/odometry data.
- **Control state monitoring:** Publishes pendant state (AUTO_READY=4, AUTO_ACTIVE=5) so downstream nodes know when the robot accepts commands.
- **Battery monitoring:** Extracts battery charge from canbus TPDO1 data.

### 3.5 Entry Points

```
amiga_bridge   → amiga_bridge_node.main()   # Combined (recommended)
amiga_streams  → streams_node.main()         # Streams only
amiga_cmd_vel  → cmd_vel_node.main()         # Commands only
```

---

## 4. Phase 2: Global Path Planner

**Status: COMPLETE**
**Package:** `farnav`
**Key File:** `global_planner.py`

### 4.1 Objective

Convert GPS waypoints (latitude/longitude) into smooth cubic spline paths in the robot's local odometry frame, parameterized by arc length for consumption by the MPC controller.

### 4.2 Development Steps

1. **Coordinate pipeline design** — Established the transformation chain: WGS84 (GPS) → UTM (meters) → Odom frame (robot-local).
2. **UTM conversion** — Used `pyproj` with EPSG:32616 (UTM Zone 16N, central North America) for GPS-to-meters conversion.
3. **GPS/Odom alignment** — Waits for first GPS fix + first odometry reading. Computes a pure translation offset (UTM is ENU-aligned, same as odom axes). No rotation needed.
4. **Cubic spline interpolation** — Used `scipy.interpolate.CubicSpline` with clamped boundary conditions. Fits separate splines for x(t) and y(t).
5. **Arc-length parameterization** — Samples spline at 500 points, computes cumulative arc-length (theta) for each point. Theta stored in Path message z-field.
6. **Multi-row YAML config** — Each row defined with A (start), B (end), and optional intermediate waypoints. Rows indexed and switchable at runtime.
7. **ROS 2 integration** — Publishes path at 1 Hz, path info as JSON, and RViz LINE_STRIP marker. Provides `/farnav/set_active_row` service for row switching.

### 4.3 Coordinate Transform Pipeline

```
GPS (WGS84)          UTM (meters)           Odom Frame
[lat, lon]  ──────►  [easting, northing]  ──────►  [x, y]
            pyproj                        translation
            EPSG:32616                    (anchor at alignment)
```

**Alignment procedure:**
- Robot sits at known position at startup
- First GPS fix provides UTM anchor: `utm_anchor = transform(gps_lat, gps_lon)`
- First odometry reading provides odom anchor: `odom_anchor = [odom.x, odom.y]`
- Offset: `offset = odom_anchor - utm_anchor`
- All future waypoints: `odom_point = utm_point + offset`

### 4.4 Published Topics

| Topic | Type | Rate | Content |
|-------|------|------|---------|
| `/farnav/global_path` | Path | 1 Hz | Spline path (x, y, z=arc_length) |
| `/farnav/path_info` | String | 1 Hz | JSON: row name, total arc length, UTM zone |
| `/farnav/active_row_marker` | Marker | 1 Hz | LINE_STRIP for RViz visualization |

### 4.5 Services

| Service | Type | Action |
|---------|------|--------|
| `/farnav/set_active_row` | Trigger | Cycle to next row in config |

---

## 5. Phase 3: MPC Controller

**Status: COMPLETE**
**Package:** `farnav`
**Key Files:** `controller_mpc.py`, `path_utils.py`

### 5.1 Objective

Implement a Model Predictive Controller that tracks the planned spline path in real time, respecting the Amiga's skid-steer kinematics and crop-row width constraints.

### 5.2 MPC Formulation

**State vector:** `x = [X, Y, psi, v]` (position, heading, linear velocity)
**Control vector:** `u = [v_cmd, omega]` (commanded velocity, angular velocity)

**Dynamics (unicycle with velocity lag):**
```
X_{k+1}   = X_k   + dt * v_k * cos(psi_k)
Y_{k+1}   = Y_k   + dt * v_k * sin(psi_k)
psi_{k+1} = psi_k + dt * omega_k
v_{k+1}   = v_k   + dt * (v_cmd_k - v_k) / tau_v
```

**Cost function (summed over N=20 prediction steps):**
```
J = SUM_{k=0}^{N-1} [
    w_cte  * e_c^2           +    (cross-track error)
    w_head * e_psi^2         +    (heading error)
    w_v    * (v - v_ref)^2   +    (velocity tracking)
    w_dv   * (delta_v_cmd)^2 +    (acceleration smoothness)
    w_dw   * (delta_omega)^2      (angular rate smoothness)
]
```

**Constraints:**
- Cross-track: `|e_c| <= 0.45 m` (row half-width)
- Wheel speed: `|v + omega*b| <= v_wheel_max` and `|v - omega*b| <= v_wheel_max`
- Angular velocity: `|omega| <= 1.0 rad/s`
- Angular rate: `|delta_omega| <= 0.5 rad/s per step`

**Solver:** CasADi NLP with IPOPT backend, warm-starting from previous solution.

### 5.3 path_utils.py — Shared Math Library

Pure NumPy library (no ROS 2 dependency) used by both planner and controller:

| Function | Purpose | Returns |
|----------|---------|---------|
| `project_onto_path()` | Find closest point on path to robot | Arc-length theta |
| `get_path_state()` | Interpolate path position & heading at theta | (x, y, psi, kappa) |
| `compute_errors()` | Signed cross-track & heading errors | (e_c, e_psi) |

Key design: Uses warm-start hint + local search window for fast projection. Falls back to global search if hint is stale.

### 5.4 Development Steps

1. **MPC formulation design** — Documented in `docs/plans/2026-03-08-controller-mpc-design.md`. Chose unicycle model with first-order velocity lag to match Amiga's response characteristics.
2. **path_utils.py** — Implemented path projection, state interpolation, and error computation. No ROS deps for testability.
3. **Unit tests for path_utils** — Full pytest coverage: straight path projection, lateral offsets, heading computation, curvature, error signs.
4. **CasADi solver** — `build_mpc_solver()` constructs the NLP problem with symbolic variables, dynamics constraints, and bounds. IPOPT configured with warm-start and reduced print level.
5. **solve_mpc()** — Projects robot onto path, generates N-step reference trajectory, solves optimization, returns first control input + full predicted trajectory.
6. **ROS 2 node** — 20 Hz control loop. Subscribes to odometry and global path. Publishes velocity commands and predicted trajectory for visualization.
7. **End-of-path deceleration** — Scales v_ref from cruise speed (0.5 m/s) to zero as robot approaches the B endpoint. Detects path end via arc-length + distance check.
8. **Skid-steer constraints** — Enforces individual wheel speed limits using track half-width b=1.105 m.
9. **MPC unit tests** — Solver builds correctly, straight path tracking, heading correction, predicted trajectory shape.
10. **Integration tests** — Closed-loop convergence: robot starts offset from path, MPC drives it back within tolerance.
11. **YAML config + launch file** — All tunable parameters externalized.

### 5.5 Control Loop Flow (20 Hz)

```
1. Check: enabled? path received? odometry received?
   └── No → publish zero velocity, return

2. Project robot position onto path → arc-length theta

3. Compute errors (e_c, e_psi) for logging

4. Generate N-step reference trajectory ahead on path

5. Check: near end of path?
   ├── Yes → scale v_ref toward zero
   └── At end → publish zero, disable controller

6. Solve CasADi/IPOPT NLP (warm-started)

7. Extract first control step: (v_cmd, omega_cmd)

8. Publish /amiga/cmd_vel (TwistStamped)

9. Publish /farnav/mpc_predicted_path (Path)

10. Store solution for warm-start on next cycle
```

### 5.6 Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| N | 20 | Prediction horizon steps |
| dt | 0.1 s | Time step (2s total lookahead) |
| w_cte | 10.0 | Cross-track error weight |
| w_head | 5.0 | Heading error weight |
| w_v | 1.0 | Velocity tracking weight |
| w_dv | 2.0 | Acceleration smoothness weight |
| w_dw | 5.0 | Angular smoothness weight |
| v_ref | 0.5 m/s | Cruise speed |
| v_wheel_max | 1.0 m/s | Max individual wheel speed |
| omega_max | 1.0 rad/s | Max angular velocity |
| delta_omega_max | 0.5 rad/s | Max angular acceleration |
| row_half_width | 0.45 m | Lateral constraint |
| track_half_width | 1.105 m | Amiga half track width |
| tau_v | 0.3 s | Velocity lag time constant |
| control_rate_hz | 20.0 Hz | Control frequency |

---

## 6. Phase 4: Testing & Validation

**Status: Unit/Integration COMPLETE, Field Testing IN PROGRESS**

### 6.1 Manual gRPC Test Scripts

Located in `scripts/`, used during initial hardware bring-up:

| Script | Purpose |
|--------|---------|
| `test_bridge_minimal.py` | Minimal canbus + ROS publisher test |
| `test_canbus.py` | Low-level canbus gRPC stream test |
| `test_cmd.py` | Single velocity command to robot |
| `test_cmd_sustained.py` | Continuous velocity command stream |
| `test_cmd_v2.py` | Improved command test |
| `test_oak.py` | OAK camera image capture |
| `test_save_frame.py` | Save camera frame to JPEG file |

### 6.2 Unit Tests (pytest)

Located in `src/farnav/test/`:

**test_path_utils.py** — Path math validation:
- Projection onto straight path (midpoint, offset, start, end)
- Path state interpolation (position, heading, curvature, clamping)
- Error computation (cross-track sign, heading error)

**test_controller_mpc.py** — MPC solver validation:
- Solver builds without error
- Straight path: offset robot steers back to path
- Straight path: on-path robot drives straight
- Heading error: misaligned robot corrects heading
- Predicted trajectory has correct shape (N+1 states)

**test_mpc_integration.py** — Closed-loop validation:
- Robot starts offset from path, converges within tolerance
- End-to-end MPC control loop convergence

**Linting tests:**
- `test_flake8.py` — PEP 8 compliance
- `test_pep257.py` — Docstring conventions
- `test_copyright.py` — Copyright header check

### 6.3 Running Tests

```bash
cd ~/lab/amiga_project/src/farnav
python -m pytest test/ -v
```

### 6.4 Live Robot Testing

Performed on the physical Amiga robot (lavender-latency):
- Canbus streaming verified at 16 Hz
- OAK camera frames captured and saved
- GPS fix acquired and UTM conversion validated
- Filter odometry streamed successfully
- Teleop via keyboard confirmed (AUTO_READY → AUTO_ACTIVE transition)
- Full bridge validated with all streams simultaneously

---

## 7. Phase 5: Visualization & Utilities

**Status: COMPLETE**

### 7.1 RViz2 Configuration

Pre-configured `amiga_bridge.rviz` with displays for:
- TF tree (odom → base_link)
- Odometry pose trail
- OAK RGB and left camera feeds
- GPS position
- IMU data
- Fixed frame: `odom`, ThirdPersonFollower view

### 7.2 path_overlay_node.py

Projects the global path and MPC predicted path onto the OAK RGB camera image using a pinhole camera model:
- OAK-D RGB intrinsics (fx, fy, cx, cy)
- Camera mount: 2.15 m height, ~57 deg downward tilt
- Publishes annotated images on `/farnav/path_overlay/compressed`

### 7.3 live_path_viz.py

Real-time matplotlib-based path visualization for debugging:
- Shows global path, robot position, MPC predicted trajectory
- Useful during development and tuning

### 7.4 gps_odom_node.py

Fallback odometry node when the farm-ng filter service is unavailable:
- Fuses GPS (position via UTM), canbus twist (speed), and IMU gyro (heading)
- Complementary filter with configurable gyro weight (default 0.98)
- Publishes `/amiga/odom` and `/tf` (odom → base_link)

### 7.5 MPC Predicted Path

The MPC controller publishes its N+1 predicted trajectory on `/farnav/mpc_predicted_path` at 20 Hz, viewable in RViz as a Path display.

---

## 8. Phase 6: Documentation & Repository Organization

**Status: COMPLETE**

### 8.1 Documentation Produced

| Document | Location | Content |
|----------|----------|---------|
| README.md | Root | Project overview, architecture, quick start, topic reference |
| environment-setup.md | docs/ | Full setup guide (611 lines): ROS 2, SDK, deps, config, troubleshooting |
| Bridge testing summary | docs/ | Port discovery, endpoint fixes, validation results |
| MPC design doc | docs/plans/ | MPC formulation, constraints, cost function |
| MPC implementation plan | docs/plans/ | Task-by-task TDD implementation plan |
| Architecture diagram | docs/ | Mermaid system architecture |
| Planner diagram | docs/ | Mermaid global planner data flow |
| Project presentation | presentations/ | PowerPoint + PDF slides |

### 8.2 Repository Organization

- `src/` — ROS 2 packages (bridge + farnav)
- `scripts/` — Manual test scripts (moved from scattered locations)
- `presentations/` — Project presentations
- `docs/` — All design docs, diagrams, and guides
- `.gitignore` — Updated to exclude build artifacts, Python caches, IDE files

---

## 9. Configuration Reference

### 9.1 Bridge Configuration

**File:** `src/amiga_ros2_bridge/config/amiga_bridge.yaml`

```yaml
amiga_bridge:
  ros__parameters:
    host: "localhost"              # Robot IP (override with launch arg)
    canbus_port: 6001
    oak_port: 50010
    gps_port: 50010                # Shared port with OAK
    filter_port: 20001

    oak_service_name: "oak/0"
    gps_service_name: "gps"
    filter_service_name: "filter"

    enable_canbus: true
    enable_oak: true
    enable_gps: true
    enable_filter: true

    canbus_every_n: 1              # Publish every message
    oak_every_n: 2                 # Skip frames to reduce bandwidth
    gps_every_n: 1
    filter_every_n: 1

    odom_frame: "odom"
    base_frame: "base_link"
    gps_frame: "gps_link"
    camera_frame: "oak_link"
```

### 9.2 MPC Configuration

**File:** `src/farnav/config/controller_mpc.yaml`

See Section 5.6 for full parameter table.

### 9.3 Waypoint Configuration

**File:** `src/farnav/config/row_data.yaml`

```yaml
/**:
  ros__parameters:
    utm_epsg: "EPSG:32616"
    spline_resolution: 500
    publish_rate_hz: 1.0
    path_frame_id: "odom"
    active_row: 0
    rows:
      row_1:
        name: "Row 1"
        A: [lat, lon]              # Start GPS coordinate
        B: [lat, lon]              # End GPS coordinate
        waypoints:                 # Optional intermediate points
          - [lat, lon]
```

---

## 10. ROS 2 Topic & Service Reference

### 10.1 Published by Bridge (Amiga → ROS 2)

| Topic | Message Type | Rate | Source |
|-------|-------------|------|--------|
| `/amiga/twist` | geometry_msgs/TwistStamped | 16 Hz | Canbus — measured velocity |
| `/amiga/odom` | nav_msgs/Odometry | varies | Filter — pose + velocity |
| `/amiga/gps` | sensor_msgs/NavSatFix | varies | GPS — lat/lon/alt + covariance |
| `/amiga/imu` | sensor_msgs/Imu | varies | OAK — accelerometer + gyroscope |
| `/amiga/oak/rgb/compressed` | sensor_msgs/CompressedImage | varies | OAK — RGB camera (JPEG) |
| `/amiga/oak/left/compressed` | sensor_msgs/CompressedImage | varies | OAK — left stereo (JPEG) |
| `/amiga/battery` | std_msgs/Float32 | varies | Canbus — charge level |
| `/amiga/control_state` | std_msgs/UInt32 | 16 Hz | Canbus — pendant state |
| `/tf` | tf2_msgs/TFMessage | varies | Filter — odom → base_link |

### 10.2 Subscribed by Bridge (ROS 2 → Amiga)

| Topic | Message Type | Action |
|-------|-------------|--------|
| `/cmd_vel` | geometry_msgs/Twist | Forward to gRPC `/twist` |
| `/amiga/cmd_vel` | geometry_msgs/TwistStamped | Forward to gRPC `/twist` |

### 10.3 Published by Navigation

| Topic | Message Type | Rate | Source |
|-------|-------------|------|--------|
| `/farnav/global_path` | nav_msgs/Path | 1 Hz | Global planner — spline path |
| `/farnav/path_info` | std_msgs/String | 1 Hz | Global planner — JSON metadata |
| `/farnav/active_row_marker` | visualization_msgs/Marker | 1 Hz | Global planner — RViz marker |
| `/farnav/mpc_predicted_path` | nav_msgs/Path | 20 Hz | MPC controller — predicted trajectory |

### 10.4 Services

| Service | Type | Node | Action |
|---------|------|------|--------|
| `/farnav/set_active_row` | std_srvs/Trigger | Global planner | Cycle to next row |

### 10.5 Control States

| Value | State | Description |
|-------|-------|-------------|
| 0 | STATE_BOOT | Robot booting |
| 1 | STATE_MANUAL_READY | Manual mode ready |
| 2 | STATE_MANUAL_ACTIVE | Pendant joystick active |
| 3 | STATE_CC_ACTIVE | Cruise control |
| 4 | STATE_AUTO_READY | Auto mode — waiting for commands |
| 5 | STATE_AUTO_ACTIVE | Auto mode — executing commands |
| 6 | STATE_ESTOPPED | Emergency stop |

---

## 11. Known Issues & Resolutions

### 11.1 Wrong gRPC Ports (RESOLVED)

**Problem:** Default farm-ng documentation listed ports 6002 (oak), 6003 (gps), 6004 (filter).
**Resolution:** Full port scan revealed actual ports — 50010 (OAK+GPS shared), 20001 (filter). Updated config.

### 11.2 Wrong Service Names (RESOLVED)

**Problem:** Used `oak0` as service name.
**Resolution:** Robot uses `oak/0` (with slash). Added configurable `oak_service_name` parameter.

### 11.3 Wrong OAK Subscription Paths (RESOLVED)

**Problem:** Subscribed to `/state` expecting a combined payload.
**Resolution:** OAK uses separate paths per stream: `/left`, `/rgb`, `/right`, `/disparity`, `/imu`, `/calibration`.

### 11.4 Wrong GPS Path (RESOLVED)

**Problem:** Subscribed to `/state`.
**Resolution:** GPS uses `/pvt` (position/velocity/time).

### 11.5 Wrong Twist Command Endpoint (RESOLVED)

**Problem:** Used `/canbus/amiga_twist2d` — command was accepted but did not activate the robot.
**Resolution:** Correct endpoint is `/twist`. This properly transitions AUTO_READY → AUTO_ACTIVE.

### 11.6 ROS 2 DDS Daemon Issues (RESOLVED)

**Problem:** `ros2 topic list` and RViz2 hung or showed stale data.
**Cause:** Stuck ROS 2 daemon using CycloneDDS while nodes used FastDDS (mismatch).
**Resolution:** Kill stuck daemon (`pkill -9 ros2_daemon`), use `--no-daemon` flag, or ensure consistent DDS implementation across all terminals.

---

## 12. Design Decisions & Rationale

### 12.1 Single Combined Bridge Node

**Decision:** Merged streams and cmd_vel into one node (`amiga_bridge_node.py`).
**Rationale:** Simpler deployment (one process), shared gRPC connection management. Legacy separate nodes kept for cases where only streams or only commands are needed.

### 12.2 Arc-Length Parameterization

**Decision:** Path parameterized by cumulative arc-length theta, not raw index or distance.
**Rationale:** Arc-length is invariant to path curvature — a 1-meter advance means the same physical distance on curves and straights. Critical for consistent MPC reference generation.

### 12.3 Warm-Start MPC

**Decision:** Reuse previous solution as initial guess for next optimization.
**Rationale:** IPOPT converges much faster (~2-3x) with a good initial guess. At 20 Hz, consecutive problems are nearly identical.

### 12.4 Pure path_utils.py

**Decision:** No ROS 2 dependency in path math library.
**Rationale:** Fully testable with plain pytest (no ROS test runner needed). Reusable outside ROS context.

### 12.5 GPS/Odom Alignment via Translation Only

**Decision:** Pure translation offset, no rotation between UTM and odom frames.
**Rationale:** UTM is ENU (east-north-up), and the Amiga's odom frame is also ENU. A single anchor point aligns them. Simpler and more robust than estimating rotation.

### 12.6 CasADi + IPOPT

**Decision:** CasADi symbolic framework with IPOPT nonlinear solver.
**Rationale:** CasADi provides automatic differentiation for the NLP. IPOPT handles the nonlinear constraints efficiently. Both are well-established in robotics MPC.

### 12.7 Skid-Steer Wheel Constraints

**Decision:** Enforce `|v ± omega * b| <= v_wheel_max` in the MPC.
**Rationale:** The Amiga is a skid-steer robot. The left and right wheels spin at different speeds during turns. This constraint ensures physically realizable commands.

### 12.8 Fallback GPS Odometry

**Decision:** Separate `gps_odom_node.py` for when filter service is unavailable.
**Rationale:** The farm-ng filter service provides best odometry, but may not always be running. GPS+IMU fusion provides a degraded but functional alternative.

---

## 13. Future Work

| Item | Priority | Description |
|------|----------|-------------|
| Field testing | High | Validate full system on actual crop rows |
| Multi-row automation | Medium | Automatic headland turns between rows |
| Obstacle detection | Medium | Use OAK stereo depth for obstacle avoidance |
| Imitation learning | Low | Learn navigation policies from demonstration |
| Path recording | Low | Record driven paths for replay without GPS waypoints |
| Dynamic re-planning | Low | Adjust path based on real-time perception |

---

*Generated from the FarNav Amiga project codebase, March 2026.*
