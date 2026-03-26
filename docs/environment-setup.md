# Amiga ROS2 Bridge — Full Environment Setup

## Prerequisites

- **OS:** Ubuntu 22.04 (Jammy)
- **Python:** 3.10
- **ROS2:** Humble Hawksbill
- **Network access** to the Amiga robot

### Robot Details

| Field | Value |
|---|---|
| Hostname | `lavender-latency` |
| IP Address | `172.16.107.128` |

---

## 1. Install ROS2 Humble

If ROS2 Humble is not already installed:

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble desktop (includes rviz2, rqt, etc.)
sudo apt update
sudo apt install -y ros-humble-desktop

# Install dev tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip
```

Add to your `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
```

---

## 2. Install farm-ng SDKs

The bridge communicates with the Amiga over gRPC using farm-ng's Python packages:

```bash
pip install farm-ng-amiga farm-ng-core
```

Verified working versions: `farm-ng-amiga==2.3.4`, `farm-ng-core==2.3.2`.

---

## 3. Install Additional Dependencies

### Teleop

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

### farnav (Global Planner)

```bash
pip install pyproj scipy
```

### Optional: CycloneDDS (recommended)

```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

Add to `~/.bashrc` if using CycloneDDS:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 4. Clone and Build the Workspace

```bash
cd ~/lab/amiga_project

# Install ROS dependencies
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build
source install/setup.bash
```

### Workspace Packages

| Package | Description | Executables |
|---|---|---|
| `amiga_ros2_bridge` | gRPC-to-ROS2 bridge (canbus, cameras, GPS, filter, cmd_vel) | `amiga_bridge`, `amiga_streams`, `amiga_cmd_vel` |
| `farnav` | GPS waypoint navigation with cubic spline path planning | `global_planner` |

Add to your `~/.bashrc` for convenience:

```bash
source ~/lab/amiga_project/install/setup.bash
```

---

## 5. Configure the Bridge

Edit `src/amiga_ros2_bridge/config/amiga_bridge.yaml`:

```yaml
amiga_bridge:
  ros__parameters:
    host: "localhost"          # Robot IP or hostname
    canbus_port: 6001          # Canbus service port
    oak_port: 50010            # OAK camera service port
    gps_port: 50010            # GPS service port (shared with OAK)
    filter_port: 20001         # Filter/state estimation service port

    oak_service_name: "oak/0"
    gps_service_name: "gps"
    filter_service_name: "filter"

    enable_canbus: true
    enable_oak: true
    enable_gps: true
    enable_filter: true

    canbus_every_n: 1          # Publish every message
    oak_every_n: 2             # Publish every 2nd frame
    gps_every_n: 1
    filter_every_n: 1

    odom_frame: "odom"
    base_frame: "base_link"
    gps_frame: "gps_link"
    camera_frame: "oak_link"
```

### Service Port Reference

| Service | Port | gRPC Service Names |
|---|---|---|
| Canbus | 6001 | `canbus` |
| Filter / Odom | 20001 | `filter` |
| OAK Cameras + GPS | 50010 | `oak/0`, `oak/1`, `oak/2`, `oak/3`, `gps` |
| Track Follower | 20101 | — |
| System Monitor | 20201 | — |

### gRPC Endpoint Reference

| Endpoint | Service | Path | Description |
|---|---|---|---|
| Twist command | Canbus | `/twist` | Send velocity commands to the robot |
| Canbus state | Canbus | `/state` | Measured velocity, control state, battery |
| Filter state | Filter | `/state` | Pose estimation (odometry) |
| GPS PVT | GPS | `/pvt` | Position, velocity, time fix |
| OAK left | OAK | `/left` | Left stereo camera frames |
| OAK RGB | OAK | `/rgb` | RGB camera frames |
| OAK IMU | OAK | `/imu` | IMU accelerometer + gyroscope |

Rebuild after config changes:

```bash
colcon build --packages-select amiga_ros2_bridge
source install/setup.bash
```

---

## 6. Launch the Bridge

```bash
ros2 launch amiga_ros2_bridge amiga_bridge.launch.py host:=172.16.107.128
```

Or use the default host from the config:

```bash
ros2 launch amiga_ros2_bridge amiga_bridge.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `host` | `localhost` | Robot IP or hostname |
| `config_file` | `<pkg_share>/config/amiga_bridge.yaml` | Path to config YAML |

### Verify It's Running

```bash
# List all active topics
ros2 topic list

# Expected output:
# /amiga/battery
# /amiga/cmd_vel
# /amiga/control_state
# /amiga/gps
# /amiga/imu
# /amiga/oak/left/compressed
# /amiga/oak/rgb/compressed
# /amiga/odom
# /amiga/twist
# /cmd_vel
# /parameter_events
# /rosout
# /tf

# Quick checks
ros2 topic echo /amiga/odom --once
ros2 topic echo /amiga/control_state --once
ros2 topic echo /amiga/battery --once
```

### Standalone Nodes (Alternative to Launch File)

The bridge also provides separate executables if you only need specific functionality:

```bash
# Streams only (no cmd_vel)
ros2 run amiga_ros2_bridge amiga_streams --ros-args -p host:=172.16.107.128

# Cmd_vel only (no streams)
ros2 run amiga_ros2_bridge amiga_cmd_vel --ros-args -p host:=172.16.107.128
```

---

## 7. Robot Control — Control States

The Amiga pendant must be set to **Auto** mode before the robot accepts velocity commands.

| State | Value | Description |
|---|---|---|
| AUTO_READY | 4 | Pendant set to auto, waiting for commands |
| AUTO_ACTIVE | 5 | Robot is accepting and executing velocity commands |

The robot transitions from AUTO_READY (4) to AUTO_ACTIVE (5) when it receives its first `/twist` command.

```bash
# Check current control state
ros2 topic echo /amiga/control_state --once
```

---

## 8. Teleoperation (Teleop)

### Keyboard Teleop

**Important:** The bridge subscribes to `/cmd_vel` (Twist) and `/amiga/cmd_vel` (TwistStamped). `teleop_twist_keyboard` publishes to `/cmd_vel` by default, which the bridge picks up — no remap needed.

Run in a **separate, focused terminal** (keys only register when terminal has focus):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

If you need to remap to the stamped topic instead:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/amiga/cmd_vel
```

### Key Bindings

```
Moving around:
   u    i    o
   j    k    l
   m    ,    .
```

| Key | Action |
|---|---|
| `i` | Forward |
| `,` | Backward |
| `j` | Turn left |
| `l` | Turn right |
| `k` | Stop |
| `u` / `o` | Forward + turn left / right |
| `m` / `.` | Backward + turn left / right |

| Key | Speed Control |
|---|---|
| `q` / `z` | Increase / decrease all speeds by 10% |
| `w` / `x` | Increase / decrease linear speed by 10% |
| `e` / `c` | Increase / decrease angular speed by 10% |

### Manual Topic Pub Commands

```bash
# Drive forward at 0.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Spin in place (0.3 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}" --once

# Arc (forward + turn)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.2}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Using the Stamped Topic

```bash
ros2 topic pub /amiga/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}" --once
```

### Continuous Publish (Hold Velocity)

Use `--rate` instead of `--once` to send commands continuously:

```bash
# Send 0.2 m/s forward at 10 Hz
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10
```

Press `Ctrl+C` to stop publishing.

---

## 9. RViz2 Visualization

### Launch with Included Config

A pre-configured `.rviz` file ships with the bridge:

```bash
rviz2 -d ~/lab/amiga_project/src/amiga_ros2_bridge/config/amiga_bridge.rviz
```

### Pre-Configured Displays

| Display | Topic | Description |
|---|---|---|
| TF | `/tf` | Transform tree (`odom` -> `base_link`) |
| Odometry | `/amiga/odom` | Pose trail from filter |
| OAK RGB | `/amiga/oak/rgb/compressed` | Front-facing RGB camera |
| OAK Left | `/amiga/oak/left/compressed` | Left stereo camera |
| GPS | `/amiga/gps` | NavSatFix position |
| IMU | `/amiga/imu` | Accelerometer + gyroscope |

### RViz Config Details

- **Fixed Frame:** `odom`
- **View:** ThirdPersonFollower targeting `base_link`
- **Frame Rate:** 30 fps

### Launch RViz Without Config (Manual Setup)

```bash
rviz2
```

Then:
1. Set **Fixed Frame** to `odom` (top-left dropdown)
2. Click **Add** -> **By topic** -> select desired topics
3. For images: Add -> By topic -> `/amiga/oak/rgb/compressed` -> `Image`

### Useful Visualization Commands

```bash
# View TF tree as a PDF graph
ros2 run tf2_tools view_frames
# Generates frames.pdf in the current directory

# Live TF echo between frames
ros2 run tf2_ros tf2_echo odom base_link

# Monitor topic publish rates
ros2 topic hz /amiga/odom
ros2 topic hz /amiga/imu

# Inspect topic message structure
ros2 topic info /amiga/odom -v

# rqt (GUI tools for plotting, topic monitoring, etc.)
rqt
```

---

## 10. farnav — Global Planner

The `farnav` package provides GPS waypoint navigation with cubic spline interpolation.

### Configure Rows

Edit `src/farnav/config/row_data.yaml`:

```yaml
/**:
  ros__parameters:
    utm_epsg: "EPSG:32616"        # UTM zone for coordinate conversion
    spline_resolution: 500         # Number of spline sample points
    publish_rate_hz: 1.0           # Path publish rate
    path_frame_id: "utm"           # TF frame for path
    active_row: 0                  # Starting row index (0-based)
    rows:
      row_1:
        name: "Row 1"
        A: [17.385001, 78.486700]           # Start point [lat, lon]
        waypoints:
          - [17.385301, 78.486720]          # Intermediate waypoints
          - [17.385601, 78.486750]
        B: [17.385901, 78.486780]           # End point [lat, lon]
      row_2:
        name: "Row 2"
        A: [17.385001, 78.486900]
        waypoints:
          - [17.385301, 78.486920]
          - [17.385601, 78.486940]
        B: [17.385901, 78.486960]
```

### Run the Global Planner

```bash
ros2 run farnav global_planner --ros-args --params-file src/farnav/config/row_data.yaml
```

### farnav Topics

| Topic | Type | Description |
|---|---|---|
| `/farnav/global_path` | `nav_msgs/Path` | Spline-interpolated path (z = arc-length theta) |
| `/farnav/path_info` | `std_msgs/String` | JSON metadata (row name, arc length, UTM zone) |
| `/farnav/active_row_marker` | `visualization_msgs/Marker` | LINE_STRIP marker for RViz |

### farnav Services

| Service | Type | Description |
|---|---|---|
| `/farnav/set_active_row` | `std_srvs/Trigger` | Cycle to next row |

```bash
# Switch to the next row
ros2 service call /farnav/set_active_row std_srvs/srv/Trigger
```

---

## 11. Full Topic Reference

### Published by Bridge (Amiga -> ROS2)

| Topic | Type | Source |
|---|---|---|
| `/amiga/twist` | `geometry_msgs/TwistStamped` | Canbus — measured velocity |
| `/amiga/odom` | `nav_msgs/Odometry` | Filter — state estimation (pose + velocity) |
| `/amiga/gps` | `sensor_msgs/NavSatFix` | GPS — lat/lon/alt with accuracy covariance |
| `/amiga/imu` | `sensor_msgs/Imu` | OAK — accelerometer + gyroscope |
| `/amiga/oak/rgb/compressed` | `sensor_msgs/CompressedImage` | OAK — front RGB camera (JPEG) |
| `/amiga/oak/left/compressed` | `sensor_msgs/CompressedImage` | OAK — left stereo camera (JPEG) |
| `/amiga/battery` | `std_msgs/Float32` | Canbus — battery charge level |
| `/amiga/control_state` | `std_msgs/UInt32` | Canbus — pendant control state |
| `/tf` | `tf2_msgs/TFMessage` | Filter — `odom` -> `base_link` transform |

### Subscribed by Bridge (ROS2 -> Amiga)

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Standard velocity commands (teleop default) |
| `/amiga/cmd_vel` | `geometry_msgs/TwistStamped` | Stamped velocity commands |

### TF Frames

```
odom
 └── base_link
      ├── oak_link
      └── gps_link
```

---

## 12. Quick Start Cheat Sheet

Open **4 terminals**. In each one, first run `source ~/lab/amiga_project/install/setup.bash`.

**Terminal 1 — Bridge:**
```bash
ros2 launch amiga_ros2_bridge amiga_bridge.launch.py host:=172.16.107.128
```

**Terminal 2 — Teleop:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 3 — RViz:**
```bash
rviz2 -d ~/lab/amiga_project/src/amiga_ros2_bridge/config/amiga_bridge.rviz
```

**Terminal 4 — Monitoring:**
```bash
# Watch control state
ros2 topic echo /amiga/control_state

# Or watch odometry
ros2 topic echo /amiga/odom
```

Set the pendant to **Auto**, then press `i` in the teleop terminal to drive forward.

---

## Troubleshooting

### ROS2 daemon stuck

If `ros2 topic list` hangs or shows stale data:

```bash
pkill -9 ros2_daemon
ros2 topic list --no-daemon
```

### CycloneDDS vs FastDDS mismatch

Force CycloneDDS if you see DDS-related errors:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

### Cannot reach robot

```bash
# Verify connectivity
ping 172.16.107.128

# Check that gRPC ports are open
nc -zv 172.16.107.128 6001
nc -zv 172.16.107.128 50010
nc -zv 172.16.107.128 20001
```

### Teleop not working

1. Make sure the bridge is running and subscribed:
   ```bash
   ros2 topic info /cmd_vel -v
   # Should show Subscription count: 1 (from amiga_bridge)
   ```

2. Make sure the teleop terminal **has focus** — keys only register when the terminal is active.

3. Check control state is 4 or 5:
   ```bash
   ros2 topic echo /amiga/control_state --once
   ```

4. Test with manual topic pub to isolate the issue:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
   ```

5. Check bridge logs for gRPC errors:
   ```bash
   ros2 topic echo /rosout --field msg
   ```

### Bridge not sending commands (gRPC debug)

Test gRPC directly without the bridge:

```bash
python3 -c "
import asyncio
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig

async def test():
    config = EventServiceConfig(name='canbus', host='172.16.107.128', port=6001)
    client = EventClient(config)
    cmd = Twist2d(linear_velocity_x=0.1, linear_velocity_y=0.0, angular_velocity=0.0)
    try:
        resp = await client.request_reply('/twist', cmd)
        print(f'SUCCESS: {resp}')
    except Exception as e:
        print(f'FAILED: {type(e).__name__}: {e}')

asyncio.run(test())
"
```

### Rebuild after code changes

```bash
colcon build --packages-select amiga_ros2_bridge
source install/setup.bash
# IMPORTANT: restart the bridge node after rebuild
```
