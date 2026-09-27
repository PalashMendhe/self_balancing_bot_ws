# Implementation Plan: Workspace Error Resolution & Stabilization

This document outlines the detailed step-by-step resolution plan for all identified errors, linter failures, algorithmic bugs, model configuration issues, and repository hygiene problems in the `self_balancing_ws` workspace.

---

## 1. Summary of Issues & Resolution Stages

| Stage | Target Area | Key Changes | Impact / Outcome |
|---|---|---|---|
| **Phase 1** | Linter & Style (Flake8) | Fix formatting, import orders, indentations in Python files | 100% test pass rate on `colcon test` |
| **Phase 2** | Balance Controller Logic | Fix IIR filter coefficients, latch fall state, fix sim time and startup timing | Eliminates setpoint doubling, floor thrashing, and timing jitter |
| **Phase 3** | Package Dependencies | Update `package.xml` for both packages with correct `exec_depend` tags | Resolves missing dependency declarations for rosdep / packaging |
| **Phase 4** | URDF & Simulation Worlds | Fix CoM position in URDF; clean up `basic_world.sdf` XML/SDF format | Physically accurate kinematics/inertias and valid world files |
| **Phase 5** | Git Repository Hygiene | Untrack cached `install/` and `log/` files; sync remote `README.md` | Clean git working tree; prevent phantom diffs on rebuild |
| **Phase 6** | Verification & Validation | Run tests, launch simulation, verify telemetry | Validate end-to-end bot balancing in Gazebo Harmonic |
| **Phase 7** | Parallel Sandbox Setup | Multi-robot spawn / headless sandbox orchestration | High-throughput parallel simulation for rapid evaluations |
| **Phase 8** | Auto PID Calibration | Optuna/CMA-ES tuning loop triggered on each fall | Automated convergence to optimal inner & outer PID gains |

---

## Phase 1: Code Style & Flake8 Linter Fixes

Resolve all 14 test failures reported by `ament_flake8` so that `colcon test` passes completely.

### 1.1 Fix [`balance_controller.py`](file:///home/plsh/self_balancing_ws/src/balance_controller/balance_controller/balance_controller.py)
- **Import Ordering (`I100`)**:
  Order imports alphabetically according to PEP-8 / ROS guidelines:
  ```python
  from geometry_msgs.msg import Twist
  from nav_msgs.msg import Odometry
  import numpy as np
  import rclpy
  from rclpy.node import Node
  from sensor_msgs.msg import Imu
  ```
- **Blank Lines & Class Spacing (`E302`, `E305`, `CNL100`)**:
  - Insert two blank lines before `class BalanceController(Node):`.
  - Insert two blank lines before `def main(args=None):`.
  - Insert two blank lines before `if __name__ == '__main__':`.
  - Insert one blank line before `def odom_callback(self, msg):` (`E301`).
- **Whitespace & Formatting (`W293`, `E501`, `W292`)**:
  - Remove trailing whitespaces on blank lines 105 and 140.
  - Break line 145 (`u = -(self.kp_theta * ...)`) into multiple lines to remain within the 99-character limit:
    ```python
    u = -(
        self.kp_theta * theta_error -
        self.kd_theta * self.theta_dot +
        self.ki_theta * self.theta_integral
    )
    ```
  - Ensure a single trailing newline at the end of the file.

### 1.2 Fix [`setup.py`](file:///home/plsh/self_balancing_ws/src/balance_controller/setup.py)
- **Indentation (`E122`)**:
  Fix indentation of `entry_points`:
  ```python
      entry_points={
          'console_scripts': [
              'balance_controller = balance_controller.balance_controller:main'
          ],
      },
  ```

### 1.3 Fix [`gazebo.launch.py`](file:///home/plsh/self_balancing_ws/src/self_balancing_bot/launch/gazebo.launch.py)
- **Trailing Blank Lines (`W391`)**:
  Remove unnecessary blank line at line 70 so the file ends with a single newline after line 69 (`])`).

---

## Phase 2: Controller Algorithmic & Logic Bug Fixes

File: [`src/balance_controller/balance_controller/balance_controller.py`](file:///home/plsh/self_balancing_ws/src/balance_controller/balance_controller/balance_controller.py)

### 2.1 Fix Filter Coefficient Bug (Line 139)
- **Problem**: `self.theta_filtered = 0.9 * self.theta_filtered + 0.2 * theta_setpoint` sums to $1.1$, producing an unstable filter with DC gain of $2.0$.
- **Fix**: Use normalized coefficients summing to $1.0$:
  ```python
  self.theta_filtered = 0.9 * self.theta_filtered + 0.1 * theta_setpoint
  ```

### 2.2 Fix Fall Detection & Thrashing State (Lines 121–126)
- **Problem**: Setting `self.theta = 0.0` when fallen immediately unsets the fallen condition on the subsequent tick, causing the controller to drive wheels while on the floor.
- **Fix**: Add a persistent state flag `self.is_fallen`:
  ```python
  if abs(self.theta) > 0.8:
      self.is_fallen = True

  if self.is_fallen:
      msg = Twist()
      self.cmd_vel_publisher_.publish(msg)
      self.get_logger().warn_throttle(2.0, f'ROBOT FALLEN (theta={self.theta:.3f}). Stopped motor output.')
      return
  ```

### 2.3 Fix Startup Timing & Estimator Contention (Lines 114–117)
- **Problem**: The control loop forces `self.theta = 0.0` for 3 seconds while IMU calibration finishes at 2 seconds. When 3 seconds elapses, `self.theta` abruptly changes, causing a torque step response.
- **Fix**:
  - Do not zero `self.theta` in `control_loop`.
  - Instead, check `if not self.calibrated:` and publish zero velocity until calibration finishes.
  - Spawning drop: Ensure the robot spawn height in [`gazebo.launch.py`](file:///home/plsh/self_balancing_ws/src/self_balancing_bot/launch/gazebo.launch.py) matches wheel ground contact ($z = 0.10\text{ m}$) rather than $0.12\text{ m}$ to avoid falling during calibration.

### 2.4 Simulation Time & Dynamic `dt`
- **Sim Time**: Declare parameter `use_sim_time` in `__init__`:
  ```python
  self.declare_parameter('use_sim_time', False)
  ```
- **Dynamic `dt` in `control_loop`**:
  Instead of hardcoding `dt = 0.02`, track `self.last_control_time` and compute actual elapsed simulation time:
  ```python
  now = self.get_clock().now()
  if self.last_control_time is None:
      self.last_control_time = now
      return
  dt = (now - self.last_control_time).nanoseconds / 1e9
  self.last_control_time = now
  if dt <= 0.0 or dt > 0.1:
      return
  ```

### 2.5 Clean Up Dead Code and Redundant Variables
- Remove duplicate `self.theta` and `self.theta_dot` assignments in `__init__`.
- Remove unused variables `self.roll` and `self.az`.
- Either remove unused `quaternion_to_euler` or document why it is retained (e.g., fallback for simulators publishing valid quaternions).

---

## Phase 3: Package Metadata & Dependencies (`package.xml`)

### 3.1 Update [`balance_controller/package.xml`](file:///home/plsh/self_balancing_ws/src/balance_controller/package.xml)
Add missing runtime dependencies:
```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
```

### 3.2 Update [`self_balancing_bot/package.xml`](file:///home/plsh/self_balancing_ws/src/self_balancing_bot/package.xml)
Add missing runtime/launch dependencies:
```xml
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
```
Remove unnecessary `python3-scipy` and `python3-numpy` from `self_balancing_bot` (since it contains no Python code).

---

## Phase 4: URDF & Simulation World Model Fixes

### 4.1 Correct Center of Mass & Inertia in [`robot.urdf.xacro`](file:///home/plsh/self_balancing_ws/src/self_balancing_bot/urdf/robot.urdf.xacro)
- Visual/collision box center is $z = 0.05$ with height $0.25$ ($z \in [-0.075, 0.175]$).
- Current `cz="0.225"` is positioned above the physical bounding box.
- Adjust `cz` to a realistic center of mass (e.g. `cz="0.08"` to `cz="0.10"` for an inverted pendulum with higher mass concentration).
- Re-align indentation on lines 132–144 for `<gazebo reference="base_link">`.

### 4.2 Fix Legacy [`basic_world.sdf`](file:///home/plsh/self_balancing_ws/src/self_balancing_bot/worlds/basic_world.sdf)
- Remove line 1 `<?xml version="1.0"?>` (Gazebo Harmonic rejects XML headers in SDF files).
- Update `<sdf version="1.6">` to `<sdf version="1.8">`.
- Add `gz-sim-imu-system` plugin if this world is intended to be used as an alternative environment.

---

## Phase 5: Git Repository & Workspace Hygiene

### 5.1 Untrack Build Products from Git Index
The folders `install/` and `log/` are tracked in git history despite being in `.gitignore`. Untrack them without deleting local files:
```bash
git rm -r --cached install/ log/
git commit -m "chore: stop tracking build and install artifacts"
```

### 5.2 Add `.vscode/` to `.gitignore`
Append `.vscode/` to [`.gitignore`](file:///home/plsh/self_balancing_ws/.gitignore).

### 5.3 Fast-Forward Main Branch
Pull the remote commit containing the project `README.md`:
```bash
git pull --ff-only origin main
```

---

## Phase 6: Verification & Validation Workflow

Execute the following steps sequentially to verify all fixes:

1. **Clean Rebuild**:
   ```bash
   cd ~/self_balancing_ws
   colcon build --symlink-install
   ```
2. **Execute Full Test Suite**:
   ```bash
   colcon test --event-handlers console_direct+
   colcon test-result --all
   ```
   *Expected Result*: 0 test failures across all packages.
3. **Launch Simulation**:
   ```bash
   source install/setup.bash
   ros2 launch self_balancing_bot gazebo.launch.py
   ```
4. **Run Controller Node (in second terminal)**:
   ```bash
   source install/setup.bash
   ros2 run balance_controller balance_controller --ros-args -p use_sim_time:=true
   ```
5. **Verify Telemetry & Balancing**:
   - Check IMU topic rate: `ros2 topic hz /imu` (~100 Hz).
   - Check controller velocity commands: `ros2 topic echo /cmd_vel`.
   - Verify that the robot balances stably without thrashing or runaway divergence.

---

## Phase 7: Parallel Simulation Sandboxes Setup

To accelerate PID calibration without burning time on real-time sequential testing, implement parallel sandbox execution.

### 7.1 Multi-Robot Swarm in Single World (Primary Architecture)
- Spawn $N$ identical balancing robots side-by-side along the Y-axis ($y = 0, 2, 4, 6, \dots\text{ m}$).
- Assign distinct namespaces to each robot: `/bot_0`, `/bot_1`, ..., `/bot_N`.
- Remap bridges and topics per namespace:
  - `/bot_{i}/imu`
  - `/bot_{i}/odom`
  - `/bot_{i}/cmd_vel`
- **Benefit**: Runs inside a single lightweight Gazebo Harmonic process, saving substantial CPU and RAM overhead compared to multiple simulator instances.

### 7.2 Headless Acceleration Configuration
- Launch Gazebo Harmonic headless: `gz sim -s -r -v 0 balance.sdf`.
- Configure physics engine with smaller step size and unlocked real-time factor, allowing simulations to execute 3x–10x faster than wall-clock time.

---

## Phase 8: Automated Episode-Based PID Calibration

Tune the inner and outer cascade loops automatically by evaluating performance on each fall.

### 8.1 Fall-Triggered Episodic Loop
1. **Spawn / Reset**: Robot is placed upright at $(x=0, z=0.10\text{ m})$ with zero velocity via Gazebo's `/world/balance_world/set_pose` service (eliminates simulator restart overhead).
2. **Episode Execution**: The controller runs with a candidate gain tuple $(K_{p,\theta}, K_{d,\theta}, K_{i,\theta})$ for the inner loop (and subsequent outer loop gains).
3. **Termination Trigger**:
   - **Fall**: $|\theta| > 0.8\text{ rad}$ triggers immediate episode conclusion.
   - **Timeout (Success)**: Surviving $T_{\max} = 30\text{ s}$ concludes a successful trial.
4. **Instant Reset**:
   ```bash
   gz service -s /world/balance_world/set_pose \
     --reqtype gz.msgs.Pose \
     --reptype gz.msgs.Boolean \
     --timeout 1000 \
     --req 'name: "my_robot", position: {x: 0, y: 0, z: 0.10}, orientation: {x: 0, y: 0, z: 0, w: 1}'
   ```

### 8.2 Loss / Reward Formulation
Each trial computes an objective metric $J$:

$$J = T_{\text{survived}} - \alpha \int_0^{T} \theta(t)^2 \, dt - \beta \int_0^{T} x(t)^2 \, dt - \gamma \int_0^{T} u(t)^2 \, dt$$

- Maximizing $J$ rewards longer balance duration while penalizing high angular deviations and aggressive control effort.

### 8.3 Optimization Strategy
- **Bayesian Optimization (via `optuna`)**:
  - Sample gain search space:
    - $K_{p,\theta} \in [10.0, 80.0]$
    - $K_{d,\theta} \in [1.0, 15.0]$
    - $K_{i,\theta} \in [0.0, 0.5]$
  - After each fall, feed the loss $J$ into the Gaussian Process / TPE sampler to suggest the next parameter set.
- **Dynamic Parameter Injection**:
  - Set controller gains on-the-fly via ROS 2 parameter client (`ros2 param set /balance_controller ...`), removing any need to recompile or restart nodes between falls.

