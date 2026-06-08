# robot_controllv2

## Launch

### Real robot — `robot.launch.py`
Connects to the physical Dynamixel servos over USB. Starts the full ros2_control stack, RViz, and both nodes.

- `ros2_control_node` with `dynamixel_hardware/DynamixelHardware` (USB port `/dev/ttyUSB0`)
- Spawns: `joint_state_broadcaster`, `velocity_controller`, `joint_trajectory_controller`
- Starts: `robot_state_publisher`, `rviz2`, `move_servos_node`, `ik_node`

```bash
ros2 launch robot_controllv2 robot.launch.py
```

### Simulation — `sim.launch.py`
No physical robot required. Uses `mock_components/GenericSystem` hardware so all controllers and nodes run identically to the real robot launch.

- `ros2_control_node` with `mock_components/GenericSystem`
- Spawns: `joint_state_broadcaster`, `joint_trajectory_controller`
- Starts: `robot_state_publisher`, `rviz2`, `move_servos_node`, `ik_node`

```bash
ros2 launch robot_controllv2 sim.launch.py
```

---

## Run nodes individually

```bash
ros2 run robot_controllv2 move_servos_node
ros2 run robot_controllv2 ik_node
```

With config file:
```bash
ros2 run robot_controllv2 move_servos_node --ros-args --params-file $(ros2 pkg prefix robot_controllv2)/share/robot_controllv2/config/config.yaml
ros2 run robot_controllv2 ik_node --ros-args --params-file $(ros2 pkg prefix robot_controllv2)/share/robot_controllv2/config/config.yaml
```

---

## Topics

### `/ik_node/xyz`
Send a target position for the pen tip. The IK node computes joint angles and forwards them to `move_servos_node`.

- **Type:** `geometry_msgs/msg/Point`
- **Data:** `x`, `y`, `z` in metres

```bash
ros2 topic pub --once /ik_node/xyz geometry_msgs/msg/Point '{x: 0.15, y: 0.0, z: 0.0}'
```

---

### `/move_servos_node/angles`
Move the robot directly to a specific joint configuration.

- **Type:** `std_msgs/msg/Float32MultiArray`
- **Data:** 3 floats — `[joint1, joint2, joint3]` in radians

```bash
ros2 topic pub --once /move_servos_node/angles std_msgs/msg/Float32MultiArray '{data: [0.0, 0.0, 0.0]}'
```

---

### `/move_servos_node/commands`
Send string commands to the node.

- **Type:** `std_msgs/msg/String`
- **Supported values:**
  - `"reset"` — moves the robot to the initial configuration `[0, 0, -90°]`

```bash
ros2 topic pub --once /move_servos_node/commands std_msgs/msg/String '{data: "reset"}'
```
