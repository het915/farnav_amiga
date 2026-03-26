# Amiga ROS2 Bridge — Testing Summary (2026-03-10)

## Robot Details
- **Robot hostname:** lavender-latency
- **Robot IP:** 172.16.107.128

## Service Port Mapping (Discovered)

| Service | Port | Service Name | Status |
|---------|------|-------------|--------|
| Canbus | 6001 | canbus | Working |
| Filter (odometry) | 20001 | filter | Available |
| OAK Cameras (x4) | 50010 | oak/0, oak/1, oak/2, oak/3 | Available |
| GPS | 50010 | gps | Available |
| Track Follower | 20101 | track_follower | Available |
| System Monitor | 20201 | system_monitor | Available |

**Note:** Default ports (6002-6004) from farm-ng docs are wrong for this robot. Actual ports discovered via full port scan.

## What Works

### Canbus Stream (port 6001)
- Twist (measured velocity) at ~16 Hz
- Battery charge level
- Control state
- Motor states

### OAK Camera (port 50010)
- RGB, Left, Right, Disparity frames (JPEG, ~290KB per frame)
- IMU data (OakImuPackets)
- 4 cameras available: oak/0, oak/1, oak/2, oak/3
- Streams use individual paths: `/left`, `/rgb`, `/right`, `/disparity`, `/imu`, `/calibration`

### GPS (port 50010, shared with OAK)
- PVT (position/velocity/time) via `/pvt` path
- ECEF coordinates via `/ecef` path
- Relative position via `/relposned` path

### Filter/Odometry (port 20001)
- FilterState with pose (position + quaternion) via `/state` path

### Teleop (cmd_vel → Amiga)
- Correct gRPC endpoint: `/twist` (NOT `/canbus/amiga_twist2d`)
- `/twist` activates the robot from AUTO_READY → AUTO_ACTIVE
- Works with `teleop_twist_keyboard` on `/cmd_vel`

## Control States
| Value | State | Description |
|-------|-------|-------------|
| 0 | STATE_BOOT | Booting |
| 1 | STATE_MANUAL_READY | Manual mode, not active |
| 2 | STATE_MANUAL_ACTIVE | Pendant joystick controls robot |
| 3 | STATE_CC_ACTIVE | Cruise control active |
| 4 | STATE_AUTO_READY | Auto mode ready, waiting for commands |
| 5 | STATE_AUTO_ACTIVE | Auto mode active, accepts velocity commands |
| 6 | STATE_ESTOPPED | Emergency stopped |

**To teleop:** Robot must be in AUTO_READY (4) via pendant, then sending commands via `/twist` transitions it to AUTO_ACTIVE (5).

## Issues Found & Fixed

### 1. Wrong service ports
- **Problem:** Default config had ports 6002 (oak), 6003 (gps), 6004 (filter)
- **Fix:** Updated to 50010 (oak+gps), 20001 (filter)

### 2. Wrong service names
- **Problem:** Used `oak0`, now the robot uses `oak/0`
- **Fix:** Added `oak_service_name` parameter, default `oak/0`

### 3. Wrong OAK subscription paths
- **Problem:** Subscribed to `/state` with combined payload
- **Fix:** Separate subscriptions to `/left`, `/rgb`, `/imu` per OAK stream

### 4. Wrong GPS subscription path
- **Problem:** Subscribed to `/state`
- **Fix:** Changed to `/pvt`

### 5. Wrong twist command endpoint
- **Problem:** Used `/canbus/amiga_twist2d` — accepted but didn't activate robot
- **Fix:** Changed to `/twist` — correctly transitions AUTO_READY → AUTO_ACTIVE

### 6. ROS2 DDS discovery broken
- **Problem:** `ros2 topic list`, `ros2 topic echo`, and RViz2 all hung
- **Cause:** Stuck ROS2 daemon process using CycloneDDS while nodes used FastDDS
- **Fix:** Kill stuck daemon (`pkill -9 ros2_daemon`), restart tools

## ROS2 Topics Published
| Topic | Type | Source |
|-------|------|--------|
| `/amiga/twist` | TwistStamped | Canbus measured velocity |
| `/amiga/odom` | Odometry | Filter pose + velocity |
| `/amiga/gps` | NavSatFix | GPS position |
| `/amiga/imu` | Imu | OAK IMU |
| `/amiga/oak/rgb/compressed` | CompressedImage | OAK RGB camera |
| `/amiga/oak/left/compressed` | CompressedImage | OAK left camera |
| `/amiga/battery` | Float32 | Battery charge level |
| `/amiga/control_state` | UInt32 | Robot control state |
| `/tf` | TransformStamped | odom → base_link |

## ROS2 Topics Subscribed
| Topic | Type | Action |
|-------|------|--------|
| `/amiga/cmd_vel` | TwistStamped | Forward to Amiga via gRPC `/twist` |
| `/cmd_vel` | Twist | Forward to Amiga via gRPC `/twist` |

## Files Modified
- `src/amiga_ros2_bridge/config/amiga_bridge.yaml` — Updated ports and added service name params
- `src/amiga_ros2_bridge/amiga_ros2_bridge/amiga_bridge_node.py` — Fixed ports, service names, subscription paths, twist endpoint
- `src/amiga_ros2_bridge/config/amiga_bridge.rviz` — New RViz2 config file
