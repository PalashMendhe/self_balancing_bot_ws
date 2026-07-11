# Self-Balancing Bot — ROS2 + Gazebo Harmonic

A two-wheeled self-balancing robot simulated in Gazebo Harmonic with a cascade PID controller. Peviosuly implemented using Lagrangian mechanics.

**GitHub:** [github.com/PalashMendhe/self_balancing_bot_ws](https://github.com/PalashMendhe/self_balancing_bot_ws)

---

## Stack

- ROS2 Jazzy, Gazebo Harmonic, `ros-gz-bridge`
- Python: `numpy`, `scipy`

## Project Structure

```
self_balancing_ws/src/
├── self_balancing_bot/       # URDF, launch, world SDF
└── balance_controller/       # PID controller node
```

---

## Robot

| Parameter | Value |
|-----------|-------|
| Base | 0.15 × 0.10 × 0.25 m, 0.6 kg |
| Wheels | r=0.05 m, separation 0.16 m, 0.1 kg each |
| IMU | base_link, 100 Hz |
| Drive | DiffDrive, `/cmd_vel` → `/odom` |

---

## Control

Cascade PID: `x_error → [Outer PID] → theta_setpoint → [Inner PID] → cmd_vel`

| Loop | Gains |
|------|-------|
| Inner (pitch) | kp=30, kd=5, ki=0.05 |
| Outer (position) | kp=0.15, kd=0.08, ki=0.005 |

Pitch estimated via complementary filter (alpha=0.85) fusing gyro integration and `arctan2(ax, az)`.

---

## Build & Run

```bash
cd ~/self_balancing_ws && colcon build && source install/setup.bash
ros2 launch self_balancing_bot gazebo.launch.py   # Terminal 1
ros2 run balance_controller balance_controller     # Terminal 2
```

---

## Critical Gotchas

**SDF:** No `<?xml?>` declaration — Gazebo's loader rejects it. First line must be `<sdf version="1.8">`.

**IMU system plugin** goes in the world SDF `<world>` tag, not in the URDF sensor block. Without it the topic publishes zeros silently.

**IMU orientation quaternion** is always identity in Gazebo Harmonic. Use complementary filter on `angular_velocity` + `linear_acceleration`, not `msg.orientation`.

**Control sign:** `u = -(kp * theta_error - kd * theta_dot)` — the negation is required.

**Gyro bias** must be subtracted before integration, not after.

**x_dot filtering:** Raw odom `x_dot` is too noisy for derivative control. Use `0.95 * filtered + 0.05 * raw` minimum before applying `kd_x`.

**angular.z roll correction** causes rotation, not stability. Remove it entirely.

---

## Diagnostics

```bash
ros2 topic echo /imu --once          # verify real IMU data
gz topic -l | grep imu               # verify Gazebo side sensor
ros2 topic hz /imu                   # should be ~100 Hz
```
