# robot_controllv2

## Run

```bash
ros2 run robot_controllv2 move_servos_node
```

---

## Topics

### `/move_servos_node/angles`
Move the robot to a specific joint configuration.

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
