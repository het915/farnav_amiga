# controller_mpc Design

## Overview

MPC controller node for the FarNav system. Follows crop rows on the Farm-ng Amiga (4-wheel skid-steer) by tracking spline paths from the global planner.

## Files

```
farnav/
├── config/controller_mpc.yaml    (NEW)
├── farnav/path_utils.py          (NEW)
├── farnav/controller_mpc.py      (NEW)
├── launch/controller_mpc.launch.py (NEW)
└── setup.py                      (modify - add entry point)
```

## MPC Formulation

**State**: `[X, Y, ψ, v]`
**Controls**: `[v_cmd, ω_cmd]`

### Dynamics (discrete, dt=0.1s)

```
X_{k+1} = X_k + dt·v_k·cos(ψ_k)
Y_{k+1} = Y_k + dt·v_k·sin(ψ_k)
ψ_{k+1} = ψ_k + dt·ω_k
v_{k+1} = v_k + dt·(v_cmd_k - v_k)/τ_v
```

### Cost (summed over N=20 steps)

```
w_cte·e_c² + w_head·e_ψ² + w_v·(v - v_ref)² + w_dv·Δv_cmd² + w_dω·Δω_cmd²
```

### Constraints

- `|e_c| ≤ row_half_width` — stay inside crop row
- `|v + ω·b| ≤ v_wheel_max` and `|v - ω·b| ≤ v_wheel_max` — individual wheel limits
- `|ω| ≤ ω_max`
- `|Δω| ≤ Δω_max`

## path_utils.py

Shared library (no ROS2 dependency) with:

- `project(path_x, path_y, robot_x, robot_y, theta_hint)` → closest arc-length θ
- `get_path_state(theta, cs_x, cs_y)` → `(x, y, psi, kappa)`
- `compute_errors(robot_x, robot_y, robot_psi, theta, cs_x, cs_y)` → `(e_c, e_psi)`

## controller_mpc.py — ROS2 Node

### Subscriptions

| Topic | Type | Purpose |
|-------|------|---------|
| `/amiga/odom` | Odometry | Current robot state |
| `/farnav/global_path` | Path | Reference spline path |
| `/farnav/controller_enable` | Bool | Enable/disable controller |

### Publications

| Topic | Type | Purpose |
|-------|------|---------|
| `/amiga/cmd_vel` | TwistStamped | Velocity command to robot |
| `/farnav/mpc_predicted_path` | Path | Predicted trajectory (RViz debug) |

### Control Loop (20 Hz)

1. If disabled / no path / no odom → publish zero
2. Project robot onto path → θ
3. Compute errors (e_c, e_ψ)
4. Build reference for N steps ahead
5. Solve CasADi/IPOPT
6. Publish first control input
7. Publish predicted trajectory
8. Warm-start next solve with shifted solution

## YAML Parameters

```yaml
controller_mpc:
  ros__parameters:
    # Horizon
    N: 20
    dt: 0.1

    # Weights
    w_cte: 10.0
    w_head: 5.0
    w_v: 1.0
    w_dv: 2.0
    w_dw: 5.0

    # Limits
    v_wheel_max: 1.5
    omega_max: 1.0
    delta_omega_max: 0.5
    row_half_width: 0.45

    # Vehicle
    track_half_width: 0.435
    tau_v: 0.3

    # Reference
    v_ref: 0.5

    # Node
    control_rate_hz: 20.0
    path_frame_id: "utm"
```

## Assumptions (to be tuned)

- τ_v = 0.3s (velocity lag, needs system ID)
- track_half_width = 0.435m (Amiga ~87cm track)
- v_wheel_max = 1.5 m/s
- row_half_width = 0.45m (narrow crop rows)
- v_ref = 0.5 m/s (safe crop speed)
