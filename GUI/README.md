# PRAROB GUI

Standalone PyQt5 sucelje za manualno upravljanje robotom i pripremu
autonomnog moda.

## Setup

```bash
pip3 install PyQt5
source /opt/ros/jazzy/setup.bash
source ~/Desktop/robo/prarob_sem16/ros2_ws/install/setup.bash
```

## Pokretanje

U jednom terminalu pokreni simulaciju ili stvarnog robota:

```bash
ros2 launch robot_controllv2 sim.launch.py
```

ili:

```bash
ros2 launch robot_controllv2 robot.launch.py
```

U drugom terminalu pokreni GUI:

```bash
python3 GUI/robot_gui.py
```

GUI koristi:

- `/move_servos_node/angles` za slanje kutova zglobova
- `/ik_node/xyz` kao pripremljeni publisher za buduci IK/autonomous flow
- `/joint_states` za live prikaz trenutnih zglobova i end-effector pozicije
