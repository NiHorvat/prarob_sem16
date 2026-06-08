# robot_controll — Robot_2.0 ROS2 Package

RRR manipulator package for the Robotics Practicum seminar (2025/2026).
Three revolute joints (Z, Y, Y), 350×350 mm drawing workspace, pen on the end effector.

---

## Launch files

| Launch file | Purpose |
|---|---|
| `robot_2_sim.launch.py` | RViz + joint sliders (no controllers, no real robot) |
| `robot_2_ik_sim.launch.py` | Full controller stack (mock hardware) + IK + manual control |
| `robot_2_ik_real_sim.launch.py` | Full controller stack (Dynamixel hardware) + IK + manual control |

```bash
source install/setup.bash

# Simulation only (sliders)
ros2 launch robot_controll robot_2_sim.launch.py

# IK + manual control (simulation)
ros2 launch robot_controll robot_2_ik_sim.launch.py

# IK + manual control (real robot + simulation mirror)
ros2 launch robot_controll robot_2_ik_real_sim.launch.py
```

---

## API

### 1  IK control — move by end-effector position

**Topic:** `/target_position`  
**Type:** `geometry_msgs/msg/Point`  
**Node:** `ik_solver`

Send a 3-D target position (metres, robot base frame). The node solves inverse
kinematics and forwards a `JointTrajectory` to the trajectory controller.

```bash
ros2 topic pub --once /target_position geometry_msgs/msg/Point \
    "{x: 0.27, y: 0.0, z: 0.0}"
```

Robot parameters:
- Shoulder height D1 = 0.0818 m
- Upper arm L1 = 0.224 m
- Forearm + pen L2 = 0.125 m
- All joint limits ±90°

**Verified test coordinates (0 mm FK error):**

| Position (x, y, z) m | Description | j1° | j2° | j3° |
|---|---|---|---|---|
| 0.27, 0.00, 0.00 | Board x=270 mm | 0.0 | +81.4 | +75.7 |
| 0.30, +0.10, 0.00 | Board x=300, y=+100 mm | +18.4 | +89.3 | +43.1 |
| 0.30, -0.10, 0.00 | Board x=300, y=−100 mm | −18.4 | +89.3 | +43.1 |
| 0.28, 0.00, 0.15 | Pen lifted, x=280 mm | 0.0 | +51.9 | +72.1 |
| 0.30, 0.00, 0.12 | Pen lifted low | 0.0 | +61.2 | +62.7 |
| 0.22, +0.10, 0.20 | Lifted, left side | +24.4 | +36.5 | +83.3 |
| 0.22, -0.10, 0.20 | Lifted, right side | −24.4 | +36.5 | +83.3 |
| 0.20, 0.00, 0.30 | Home / safe pose | 0.0 | +19.6 | +67.1 |
| 0.15, 0.00, 0.35 | High / safe pose | 0.0 | +8.8 | +59.3 |
| 0.20, +0.15, 0.25 | Yaw test +37° | +36.9 | +34.3 | +63.5 |
| 0.20, -0.15, 0.25 | Yaw test −37° | −36.9 | +34.3 | +63.5 |
| 0.25, +0.15, 0.15 | Yaw + low | +31.0 | +54.6 | +64.8 |

---

### 2  Manual joint control — sliders via topic

**Topic:** `/manual_joints`  
**Type:** `sensor_msgs/msg/JointState`  
**Node:** `manual_controller`

Publish joint angles in **degrees**. Any subset of joints is accepted; missing
joints default to 0°. Angles are clamped to ±90°.

```bash
# All three joints at once
ros2 topic pub --once /manual_joints sensor_msgs/msg/JointState \
    "{name: ['joint1','joint2','joint3'], position: [30.0, 45.0, -20.0]}"

# Single joint (others go to 0°)
ros2 topic pub --once /manual_joints sensor_msgs/msg/JointState \
    "{name: ['joint2'], position: [60.0]}"
```

---

### 3  Manual joint control — sliders via parameter

**Parameters:** `joint1_deg`, `joint2_deg`, `joint3_deg`, `duration_sec`  
**Node:** `manual_controller`

Change a parameter and the robot moves immediately. This is the closest
equivalent to a slider: call it repeatedly to sweep through angles.

```bash
# Move joint1 to +45°
ros2 param set /manual_controller joint1_deg 45.0

# Move shoulder to +60°
ros2 param set /manual_controller joint2_deg 60.0

# Slow down motion to 3 seconds
ros2 param set /manual_controller duration_sec 3.0

# Read current parameter values
ros2 param get /manual_controller joint1_deg
ros2 param dump /manual_controller
```

---

### 4  Reset all joints to 0°

**Topic:** `/reset_joints`  
**Type:** `std_msgs/msg/Empty`  
**Node:** `manual_controller`

Sends all joints to 0° in 1.5 s and resets the parameter values to 0.

```bash
ros2 topic pub --once /reset_joints std_msgs/msg/Empty "{}"
```

---

### 5  Read current joint state

**Topic:** `/joint_states`  
**Type:** `sensor_msgs/msg/JointState`  
**Published by:** `joint_state_broadcaster` (controller stack) or `joint_state_publisher_gui` (slider launch)

```bash
# Stream joint states
ros2 topic echo /joint_states

# Single reading
ros2 topic echo --once /joint_states
```

---

### 6  Controller management

```bash
# List active controllers
ros2 control list_controllers

# List hardware interfaces
ros2 control list_hardware_interfaces

# Switch to position mode (if needed)
ros2 control set_controller_state joint_trajectory_controller active
```

---

## Drawing board coordinate system

The 350×350 mm workspace is visualised in RViz as `drawing_board` (fixed to `world`).

| World frame | Board (mm) | RViz marker |
|---|---|---|
| (0.000, −0.175, 0) | (0, 0) — origin corner | RED cylinder |
| (0.100, −0.175, 0) | (100, 0) | ORANGE cylinder |
| (0.200, −0.175, 0) | (200, 0) | ORANGE cylinder |
| (0.300, −0.175, 0) | (300, 0) | ORANGE cylinder |
| (0.000, −0.075, 0) | (0, 100) | CYAN cylinder |
| (0.000,  0.025, 0) | (0, 200) | CYAN cylinder |
| (0.000,  0.125, 0) | (0, 300) | CYAN cylinder |
| (0.175,  0.000, 0) | (175, 175) — centre | GREEN cylinder |

Grid: light grey every 50 mm, dark grey every 100 mm.

---

## Kinematic parameters

```
D1  = 0.0818 m   shoulder height above world origin
L1  = 0.224  m   upper arm  (joint2 → joint3)
L2  = 0.125  m   forearm + pen  (joint3 → end_effector)
```

Forward kinematics:

```
x = (L1·sin(q2) + L2·sin(q2+q3)) · cos(q1)
y = (L1·sin(q2) + L2·sin(q2+q3)) · sin(q1)
z = D1 + L1·cos(q2) + L2·cos(q2+q3)
```

---

## Hardware configuration (`/dev/ttyUSB0`, 1 Mbaud)

| Joint | Dynamixel ID | Axis | Role | Limit |
|---|---|---|---|---|
| joint1 | 11 | Y | arm (shoulder) | ±90° |
| joint2 | 12 | Y | hand (end effector) | ±90° |
| joint3 | 13 | Z | base rotator | ±90° |
