# Pokretanje PRAROB seminar projekta

Ovaj dokument je prakticni runbook za trenutno stanje projekta. Namjena mu je
da netko iz tima moze sjesti za Linux/VM i znati kojim redom buildati,
pokretati i testirati sustav.

Projekt se nalazi ovdje:

```bash
~/robo_work/prarob_sem16
```

Ako radis lokalno iz Windows foldera, odgovarajuci root je:

```text
C:\Users\Bruno\Desktop\robo\prarob_sem16
```

## Trenutno stanje

Implementirano:

- `robot_controllv2`: URDF, ROS 2 control, `sim.launch.py`,
  `robot.launch.py`, `ik_node`, `move_servos_node`.
- `prarob_interact`: DK/IK u `kinematics.py`, ROSA tools integracija,
  `path_planning.py`.
- `prarob_autonomous`: deterministicki parser, autonomous draw node,
  board calibration node, `autonomous.launch.py`, `challenge.launch.py`.
- `GUI/robot_gui.py`: PyQt5 GUI za manualni DK/IK i placeholder autonomous tab.
- YOLO infrastruktura: `yolo_msgs`, `yolo_ros`, `yolo_bringup`.
- Camera calibration package: `prarob_calib`.

Nije potpuno zavrseno / treba podesiti na stvarnom postavu:

- `board_homography.yaml` treba izmjeriti na stvarnoj ploci.
- `drawing_z`, `pen_up_z`, `seconds_per_waypoint` treba ugoditi na markeru.
- YOLO klase i threshold treba ugoditi za stvarne piktograme.
- RViz se preko SSH-a moze rusiti zbog X/GLX displaya. Najbolje ga je
  pokretati iz terminala unutar VM desktopa.

## 1. Priprema terminala

Otvori terminal u VM desktopu, ne preko SSH-a ako zelis RViz.

Uvijek prvo sourceaj ROS:

```bash
source /opt/ros/jazzy/setup.bash
```

Udji u workspace:

```bash
cd ~/robo_work/prarob_sem16/ros2_ws
```

Nakon builda sourceaj install:

```bash
source install/setup.bash
```

## 2. Ovisnosti koje trebaju biti instalirane

ROS controller koji je potreban za `joint_trajectory_controller`:

```bash
sudo apt-get install ros-jazzy-joint-trajectory-controller
```

GUI:

```bash
pip3 install PyQt5
```

ROSA terminalski agent, ako ga koristis:

```bash
pip3 install python-dotenv rich langchain langchain_openai jpl-rosa
```

Napomena: autonomous node ne treba ROSA/LLM. Parser je deterministicki.

## 3. Build svega sto je trenutno potrebno

Iz workspacea:

```bash
cd ~/robo_work/prarob_sem16/ros2_ws
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select \
  robot_controllv2 \
  prarob_calib \
  yolo_msgs \
  yolo_ros \
  yolo_bringup \
  prarob_interact \
  prarob_autonomous

source install/setup.bash
```

Ako buildas samo autonomous promjene:

```bash
colcon build --symlink-install --packages-select \
  yolo_msgs prarob_interact prarob_autonomous

source install/setup.bash
```

Ako `prarob_autonomous` launch kaze da ne postoji libexec direktorij:

```text
install/prarob_autonomous/lib/prarob_autonomous
```

provjeri da postoji:

```text
ros2_ws/src/prarob_autonomous/setup.cfg
```

sa sadrzajem:

```ini
[develop]
script_dir=$base/lib/prarob_autonomous

[install]
install_scripts=$base/lib/prarob_autonomous
```

Zatim rebuildaj:

```bash
colcon build --symlink-install --packages-select prarob_autonomous
source install/setup.bash
```

## 4. Brza provjera builda

Provjeri da ROS vidi pakete:

```bash
ros2 pkg prefix robot_controllv2
ros2 pkg prefix prarob_calib
ros2 pkg prefix yolo_msgs
ros2 pkg prefix yolo_ros
ros2 pkg prefix yolo_bringup
ros2 pkg prefix prarob_interact
ros2 pkg prefix prarob_autonomous
```

Provjeri executables:

```bash
ros2 run prarob_autonomous autonomous_draw_node --help
ros2 run prarob_autonomous board_calibration_node --help
```

Ako `--help` ne postoji za neki node, dovoljno je da ROS nadje executable.

## 5. Sanity test bez kamere i YOLO-a

Ovo je najbrzi test da vidis GUI, robot sim i autonomous node.

Pokreni iz VM desktop terminala:

```bash
cd ~/robo_work/prarob_sem16/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch prarob_autonomous challenge.launch.py \
  use_sim:=true \
  use_camera:=false \
  use_yolo:=false \
  use_gui:=true \
  device:=cpu \
  gui_path:=/home/bruno/robo_work/prarob_sem16/GUI/robot_gui.py
```

Ocekivani nodeovi:

```bash
ros2 node list
```

Trebao bi vidjeti barem:

```text
/autonomous_draw_node
/robot_gui_node
/move_servos_node
/ik_node
/robot_state_publisher
```

Ocekivani topici:

```bash
ros2 topic list | grep -E 'autonomous|joint_states|move_servos|ik_node'
```

Trebao bi vidjeti:

```text
/autonomous_draw_node/command
/autonomous_draw_node/status
/autonomous_draw_node/debug_image
/joint_states
/ik_node/xyz
/move_servos_node/angles
/move_servos_node/commands
```

## 6. Pracenje autonomous statusa

U drugom terminalu:

```bash
cd ~/robo_work/prarob_sem16/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo /autonomous_draw_node/status
```

## 7. Slanje test naredbe

U trecem terminalu:

```bash
cd ~/robo_work/prarob_sem16/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic pub --once /autonomous_draw_node/command std_msgs/msg/String \
  "{data: 'connect car and plane, avoid football'}"
```

Ako je launch pokrenut sa `use_yolo:=false`, ocekuj da parser primi naredbu,
ali da planiranje/detekcija ne moze zavrsiti jer nema `/yolo/detections`.
To je normalno za sanity test bez kamere.

## 8. Pokretanje samo autonomous nodea

Ako su robot, kamera i YOLO vec dignuti odvojeno:

```bash
cd ~/robo_work/prarob_sem16/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch prarob_autonomous autonomous.launch.py \
  board_homography_file:=$HOME/board_homography.yaml
```

Ako jos nemas homografiju:

```bash
ros2 launch prarob_autonomous autonomous.launch.py
```

Tada node koristi privremeni linearni mapping iz `autonomous_params.yaml`.

## 9. Board kalibracija

Kalibracija treba napraviti `board_homography.yaml`.

Pokreni kameru:

```bash
cd ~/robo_work/prarob_sem16/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch prarob_calib camera_world.launch.py
```

U drugom terminalu pokreni board calibration node:

```bash
ros2 run prarob_autonomous board_calibration_node \
  --ros-args \
  -p output_file:=$HOME/board_homography.yaml
```

Kad node uspjesno prepozna sahovnicu i spremi YAML, koristi ga u launchu:

```bash
ros2 launch prarob_autonomous autonomous.launch.py \
  board_homography_file:=$HOME/board_homography.yaml
```

ili u challenge launchu:

```bash
ros2 launch prarob_autonomous challenge.launch.py \
  board_homography_file:=$HOME/board_homography.yaml
```

## 10. Pokretanje s kamerom i YOLO-om

CPU varijanta:

```bash
cd ~/robo_work/prarob_sem16/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch prarob_autonomous challenge.launch.py \
  use_sim:=true \
  use_camera:=true \
  use_yolo:=true \
  use_gui:=true \
  device:=cpu \
  board_homography_file:=$HOME/board_homography.yaml \
  gui_path:=/home/bruno/robo_work/prarob_sem16/GUI/robot_gui.py
```

CUDA varijanta:

```bash
ros2 launch prarob_autonomous challenge.launch.py \
  use_sim:=true \
  use_camera:=true \
  use_yolo:=true \
  use_gui:=true \
  device:=cuda:0 \
  board_homography_file:=$HOME/board_homography.yaml \
  gui_path:=/home/bruno/robo_work/prarob_sem16/GUI/robot_gui.py
```

Ako radis sa stvarnim robotom:

```bash
ros2 launch prarob_autonomous challenge.launch.py \
  use_sim:=false \
  use_camera:=true \
  use_yolo:=true \
  use_gui:=true \
  device:=cpu \
  board_homography_file:=$HOME/board_homography.yaml \
  gui_path:=/home/bruno/robo_work/prarob_sem16/GUI/robot_gui.py
```

Prije real robot moda provjeri:

- robot je spojen
- Dynamixeli imaju napajanje
- USB port odgovara `/dev/ttyUSB0`
- imas permission za serial port

## 11. GUI

GUI mozes pokrenuti samostalno:

```bash
cd ~/robo_work/prarob_sem16
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

python3 GUI/robot_gui.py
```

Manual tab:

- slideri i spinboxovi salju kutove na `/move_servos_node/angles`
- FK preview racuna lokalno DK
- IK unos racuna lokalno IK i salje kutove
- live state cita `/joint_states`

Autonomous tab:

- trenutno je front-end placeholder
- backend se moze testirati preko `/autonomous_draw_node/command`

## 12. Parametri koje treba ugoditi

Datoteka:

```text
ros2_ws/src/prarob_autonomous/config/autonomous_params.yaml
```

Najbitniji parametri:

```yaml
drawing_z: 0.0
pen_up_z: 0.03
seconds_per_waypoint: 0.2
min_detection_score: 0.25
cell_size: 5
margin_cells: 3
workspace_x_min: 0.06
workspace_x_max: 0.28
workspace_y_min: -0.14
workspace_y_max: 0.14
```

Sto podesavati:

- `drawing_z`: visina vrha markera dok crta
- `pen_up_z`: koliko se marker digne iznad papira
- `seconds_per_waypoint`: brzina izvodjenja
- `min_detection_score`: YOLO threshold
- `margin_cells`: koliko planner zaobilazi prepreke
- workspace granice: dok nema `board_homography.yaml`, ovo je fallback mapping

## 13. Reachability / gdje staviti bazu

Zbog limita zglobova:

```text
q1, q2, q3 in [-pi/2, +pi/2]
```

robot ne doseze cijelu plocu jednako dobro. Dohvatljiv je uglavnom dalji
prsten workspacea, ne svaka tocka blizu baze.

Prakticno:

1. Postavi bazu tako da objekti koje treba spajati budu u dosezljivom dijelu.
2. Provjeri nekoliko tocaka preko GUI IK taba.
3. Ako IK kaze `No solution`, tocka je izvan dosega ili izvan joint limita.
4. Kalibraciju ploce radi tek kad je baza postavljena na finalno mjesto.

Brza IK provjera bez GUI-ja:

```bash
cd ~/robo_work/prarob_sem16/ros2_ws/src/prarob_interact
python3 - <<'PY'
from prarob_interact.kinematics import Kinematics
k = Kinematics()
for target in ([0.20, 0.00, 0.02], [0.25, 0.05, 0.02], [0.15, 0.10, 0.05]):
    print(target, k.get_ik(target))
PY
```

## 14. Poznati problemi

### RViz se rusi preko SSH-a

Ako vidis:

```text
Unable to open display
RenderingAPIException: Couldn't open X display
```

to je remote display problem. Pokreni launch iz terminala unutar VM desktopa,
ne preko SSH-a.

### `joint_trajectory_controller` nije nadjen

Ako log kaze:

```text
Loader for controller 'joint_trajectory_controller' not found
```

instaliraj:

```bash
sudo apt-get install ros-jazzy-joint-trajectory-controller
```

### `prarob_calib` ili `yolo_bringup` package not found

Buildaj dependencyje:

```bash
colcon build --symlink-install --packages-select \
  prarob_calib yolo_msgs yolo_ros yolo_bringup

source install/setup.bash
```

### `prarob_interact.path_planning` nije nadjen

Provjeri da postoji:

```text
ros2_ws/src/prarob_interact/prarob_interact/path_planning.py
```

Zatim:

```bash
colcon build --symlink-install --packages-select prarob_interact
source install/setup.bash
```

### `dotenv` ili LangChain dependency fali

Autonomous node ne bi trebao trebati ROSA/LLM dependencyje. Ako import
`prarob_interact.kinematics` trazi `dotenv`, onda je `prarob_interact/__init__.py`
stara verzija. Treba biti lazy import, bez direktnog importanja `config.py`.

## 15. Gasenje procesa

Ako nesto ostane visiti:

```bash
pkill -INT -f "ros2 launch prarob_autonomous"
pkill -INT -f "ros2 launch robot_controllv2"
pkill -INT -f "autonomous_draw_node"
pkill -INT -f "robot_gui.py"
pkill -INT -f "move_servos_node"
pkill -INT -f "ik_node"
pkill -INT -f "ros2_control_node"
pkill -INT -f "rviz2"
```

Ako se proces ne gasi:

```bash
pkill -9 -f "ime_procesa"
```

Koristi `-9` samo ako normalni `-INT` ne pomaze.

## 16. Preporuceni redoslijed za demo

1. Build:

```bash
colcon build --symlink-install --packages-select \
  robot_controllv2 prarob_calib yolo_msgs yolo_ros yolo_bringup \
  prarob_interact prarob_autonomous
source install/setup.bash
```

2. Sanity bez kamere/YOLO-a:

```bash
ros2 launch prarob_autonomous challenge.launch.py \
  use_sim:=true use_camera:=false use_yolo:=false use_gui:=true \
  gui_path:=/home/bruno/robo_work/prarob_sem16/GUI/robot_gui.py
```

3. Provjeri nodeove:

```bash
ros2 node list
ros2 topic list | grep autonomous
```

4. Napravi board kalibraciju:

```bash
ros2 run prarob_autonomous board_calibration_node \
  --ros-args -p output_file:=$HOME/board_homography.yaml
```

5. Pokreni full challenge:

```bash
ros2 launch prarob_autonomous challenge.launch.py \
  use_sim:=false \
  use_camera:=true \
  use_yolo:=true \
  use_gui:=true \
  device:=cpu \
  board_homography_file:=$HOME/board_homography.yaml \
  gui_path:=/home/bruno/robo_work/prarob_sem16/GUI/robot_gui.py
```

6. Posalji naredbu:

```bash
ros2 topic pub --once /autonomous_draw_node/command std_msgs/msg/String \
  "{data: 'connect car and plane, avoid football'}"
```

7. Gledaj status:

```bash
ros2 topic echo /autonomous_draw_node/status
```
