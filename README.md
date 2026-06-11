# PRAROB Seminar 2025/2026 - Connect the dots

Ovo je radni repozitorij za seminarski zadatak iz Praktikuma robotike. Cilj
projekta je napraviti 3-DOF RRR robotski manipulator koji crta po ploci:
prepoznaje objekte kamerom, spaja zadane objekte markerom i zaobilazi objekte
koje naredba oznaci kao prepreke.

Trenutno stanje: implementirani su ROS 2 model/upravljanje, osnovna kamera i
YOLO infrastruktura, direktna i inverzna kinematika za `prarob_interact`, te
standalone PyQt5 GUI za manualni rad. Autonomni mod je sada implementiran u
novom paketu `prarob_autonomous` (deterministicki parser naredbe ->
detekcije -> camera-to-board mapping -> A* planiranje -> IK -> izvrsavanje),
GUI autonomous tab je spojen na backend (status, timing i debug overlay), a
time challenge se pokrece jednim `challenge.launch.py`.

## Sadrzaj

- [Struktura projekta](#struktura-projekta)
- [Sto je trenutno implementirano](#sto-je-trenutno-implementirano)
- [Setup okoline](#setup-okoline)
- [Build ROS 2 workspacea](#build-ros-2-workspacea)
- [Pokretanje simulacije ili robota](#pokretanje-simulacije-ili-robota)
- [Pokretanje GUI-ja](#pokretanje-gui-ja)
- [ROS 2 topici](#ros-2-topici)
- [Kinematika](#kinematika)
- [Testovi](#testovi)
- [Status prema FAT zahtjevima](#status-prema-fat-zahtjevima)
- [Autonomous mode (prarob_autonomous)](#autonomous-mode-prarob_autonomous)
- [Time challenge (jedan klik)](#time-challenge-jedan-klik)
- [Sto jos treba validirati na stvarnom robotu](#sto-jos-treba-validirati-na-stvarnom-robotu)
- [Build prarob_autonomous](#build-prarob_autonomous)

## Struktura projekta

```text
prarob_sem16/
|-- 3D_model/
|   |-- Robot_2.0.step
|   |-- postolje.step
|   |-- Link 90.step
|   |-- Link RR.step
|   |-- Izvrsni clan.step
|   `-- kolicina.txt
|-- CV/
|   `-- calibrationdata/
|-- GUI/
|   |-- README.md
|   `-- robot_gui.py
|-- media/
|   |-- RoboticsPracticum_Seminar2026.pdf
|   `-- lab_exercise_pdfs/
`-- ros2_ws/
    |-- launch/
    |   `-- master.launch.py
    `-- src/
        |-- robot_controllv2/
        |-- prarob_interact/
        |-- prarob_autonomous/
        |-- prarob_calib/
        |-- prarob_yolo/
        |   |-- yolo_bringup/
        |   |-- yolo_processing/
        |   `-- yolo_ros/
        |-- robot_controll/
        `-- ros2_prarob/
```

Najvazniji paketi:

- `robot_controllv2` - aktualni ROS 2 control paket za URDF/RViz, real robot,
  simulaciju, IK node i node za slanje kutova na kontroler.
- `prarob_interact` - ROSA/LLM tekstualni agent, tools i sada implementirana
  cista kinematika.
- `prarob_autonomous` - deterministicki autonomni orkestrator: parser naredbe,
  camera-to-board homografija, A* planiranje s preprekama, IK izvrsavanje,
  debug overlay i `challenge.launch.py` za time challenge.
- `prarob_calib` - USB kamera, intrinzicna kalibracija i camera-to-world
  procjena preko sahovnice.
- `prarob_yolo` - vendoran `yolo_ros` paket s YOLO detekcijom i message
  definicijama; sadrzi i `yolo_processing` node koji crta YOLO bounding boxove.
- `GUI` - standalone PyQt5 aplikacija za manualni rad i pripremu autonomous
  moda.

## Sto je trenutno implementirano

### 1. Mehanicki dio

U `3D_model/` se nalaze STEP modeli za dijelove robota:

```text
postolje.step
Link 90.step
Link RR.step
Izvrsni clan.step
Robot_2.0.step
```

Popis komponenti je u `3D_model/kolicina.txt`:

```text
Jedno postolje
Jedan Link 90
Jedan Link RR
Jedan izvrsni clan
Tri XL430-W250 motora
Slika robota
```

### 2. URDF/RViz i ROS 2 control

Aktualni paket je:

```text
ros2_ws/src/robot_controllv2/
```

Bitne datoteke:

```text
urdf/prarob_manipulator.urdf.xacro
urdf/prarob_manipulator.xacro
urdf/prarob_manipulator.ros2_control.xacro
urdf/drawing_board.xacro
controllers/controllers.yaml
launch/sim.launch.py
launch/robot.launch.py
robot_controllv2/ik_node.py
robot_controllv2/move_servos_node.py
```

`sim.launch.py` koristi `mock_components/GenericSystem`, pa se moze testirati
bez fizickog robota. `robot.launch.py` koristi `dynamixel_hardware` i USB port
`/dev/ttyUSB0`.

### 3. IK node za robot

`robot_controllv2/ik_node.py` prima ciljnu tocku vrha markera na:

```text
/ik_node/xyz
```

Tip poruke:

```text
geometry_msgs/msg/Point
```

Primjer:

```bash
ros2 topic pub --once /ik_node/xyz geometry_msgs/msg/Point \
  "{x: 0.15, y: 0.0, z: 0.05}"
```

Node racuna IK i objavljuje kutove na:

```text
/move_servos_node/angles
```

### 4. Move servos node

`move_servos_node.py` cita kutove:

```text
/move_servos_node/angles
```

Tip poruke:

```text
std_msgs/msg/Float32MultiArray
```

Format:

```text
data: [q1, q2, q3]
```

Primjer:

```bash
ros2 topic pub --once /move_servos_node/angles std_msgs/msg/Float32MultiArray \
  "{data: [1.5708, 0.0, -1.5708]}"
```

Reset komanda:

```bash
ros2 topic pub --once /move_servos_node/commands std_msgs/msg/String \
  "{data: 'reset'}"
```

### 5. Direktna i inverzna kinematika u prarob_interact

Datoteka:

```text
ros2_ws/src/prarob_interact/prarob_interact/kinematics.py
```

Implementirane su metode:

```python
Kinematics.get_dk(q)
Kinematics.get_ik(w, q0=None)
Kinematics.get_closest_ik(q_all, q0)
```

Koristene konstante:

```python
D1 = 0.0579  # world -> joint1 visina
D2 = 0.0209  # joint1 -> joint2 offset
L1 = 0.224   # prvi clanak
L2 = 0.125   # joint3 -> vrh markera
```

Model:

```python
r = D2 + L1*cos(q2) + L2*cos(q2 + q3)
z = D1 + L1*sin(q2) + L2*sin(q2 + q3)
x = r*cos(q1)
y = r*sin(q1)
```

IK vraca sva validna rjesenja unutar limita:

```python
q1, q2, q3 in [-pi/2, +pi/2]
```

### 6. Integracija ROSA tools.py s kinematikom

Datoteka:

```text
ros2_ws/src/prarob_interact/prarob_interact/tools.py
```

Uskladeno je:

- `get_tool_pose()` cita `/joint_states`, izvlaci `[joint1, joint2, joint3]`,
  racuna DK i vraca:

```python
{"x": x, "y": y, "z": z}
```

- `move_to_pose()` racuna IK preko `Kinematics.get_ik()`, bira najblize
  rjesenje preko `get_closest_ik()` i salje kutove na:

```text
/move_servos_node/angles
```

### 7. PyQt5 GUI

Nova datoteka:

```text
GUI/robot_gui.py
```

GUI je standalone Python skripta i ne importira `prarob_interact`, nego u sebi
ima lokalnu kopiju DK/IK funkcija. To znaci da se GUI moze pokrenuti i ako
`prarob_interact` nije buildan kao Python paket.

GUI ima dva taba:

```text
Manual Control
Autonomous Mode
```

Manual tab sadrzi:

- slider + spinbox za `joint1`, `joint2`, `joint3`
- live FK preview
- `Send to Robot`
- IK unos `X`, `Y`, `Z`
- `Solve IK & Send`
- live `/joint_states` prikaz
- `Reset to Home`
- `Emergency Stop`

Autonomous tab trenutno sadrzi:

- natural language textbox
- `Send Command`
- status label
- live robot state widget

Autonomous backend jos nije implementiran; gumb trenutno samo javlja da je
naredba primljena.

## Setup okoline

Pretpostavka je Ubuntu s ROS 2 Jazzy.

```bash
source /opt/ros/jazzy/setup.bash
```

Instalacija Python ovisnosti za ROSA:

```bash
pip3 install python-dotenv rich langchain langchain_openai jpl-rosa
```

Instalacija GUI ovisnosti:

```bash
pip3 install PyQt5
```

Za `sim.launch.py` treba imati instaliran joint trajectory controller:

```bash
sudo apt-get install ros-jazzy-joint-trajectory-controller
```

Ako se koristi realni robot, treba imati dostupne ROS 2 control i Dynamixel
ovisnosti iz laboratorijske okoline.

## Build ROS 2 workspacea

Iz root foldera projekta:

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Ako treba buildati samo aktualni robot package:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_controllv2
source install/setup.bash
```

Ako treba buildati `prarob_interact` nakon promjene kinematike:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select prarob_interact
source install/setup.bash
```

### Build prarob_autonomous

Novi paket ovisi o `prarob_interact` (kinematika, path planning) i `yolo_msgs`:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select yolo_msgs prarob_interact prarob_autonomous
source install/setup.bash
```

Provjera ciste logike bez ROS-a (parser, mapping, planiranje, IK):

```bash
python3 src/prarob_autonomous/test/test_core_logic.py
```

## Pokretanje simulacije ili robota

### Simulacija

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_controllv2 sim.launch.py
```

### Stvarni robot

Provjeriti USB port u:

```text
ros2_ws/src/robot_controllv2/urdf/prarob_manipulator.ros2_control.xacro
```

Trenutno je postavljeno:

```xml
<param name="usb_port">/dev/ttyUSB0</param>
```

Pokretanje:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_controllv2 robot.launch.py
```

## Pokretanje GUI-ja

U jednom terminalu pokrenuti simulaciju ili robota.

U drugom terminalu:

```bash
cd ~/Desktop/robo/prarob_sem16
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 GUI/robot_gui.py
```

Ako se GUI pokrece preko SSH-a na VM-u s GNOME/Wayland sessionom, moze trebati:

```bash
export XDG_RUNTIME_DIR=/run/user/1000
export WAYLAND_DISPLAY=wayland-0
export QT_QPA_PLATFORM=wayland
python3 GUI/robot_gui.py
```

## ROS 2 topici

GUI i nodeovi koriste ove topice:

| Smjer | Topic | Tip | Opis |
| --- | --- | --- | --- |
| WRITE | `/move_servos_node/angles` | `std_msgs/msg/Float32MultiArray` | Direktno slanje kutova zglobova `[q1,q2,q3]`. |
| WRITE | `/ik_node/xyz` | `geometry_msgs/msg/Point` | Ciljna pozicija vrha markera `[x,y,z]`. |
| READ | `/joint_states` | `sensor_msgs/msg/JointState` | Trenutni zglobovi iz kontrolera. |
| WRITE | `/move_servos_node/commands` | `std_msgs/msg/String` | Komande tipa `reset`. |

Primjeri:

```bash
ros2 topic echo /joint_states
```

```bash
ros2 topic pub --once /move_servos_node/angles std_msgs/msg/Float32MultiArray \
  "{data: [1.5708, 0.0, -1.5708]}"
```

```bash
ros2 topic pub --once /ik_node/xyz geometry_msgs/msg/Point \
  "{x: 0.15, y: 0.05, z: 0.02}"
```

## Kinematika

### Direktna kinematika

Ulaz:

```python
q = [q1, q2, q3]
```

Izlaz:

```python
[x, y, z]
```

Formula:

```python
from math import cos, sin

D1 = 0.0579
D2 = 0.0209
L1 = 0.224
L2 = 0.125

r = D2 + L1 * cos(q2) + L2 * cos(q2 + q3)
z = D1 + L1 * sin(q2) + L2 * sin(q2 + q3)
x = r * cos(q1)
y = r * sin(q1)
```

### Inverzna kinematika

Ulaz:

```python
w = [x, y, z]
```

Postupak:

```python
q1 = atan2(y, x)
r = sqrt(x*x + y*y) - D2
zz = z - D1
dist = sqrt(r*r + zz*zz)
cos_q3 = (dist*dist - L1*L1 - L2*L2) / (2 * L1 * L2)
q3 = +/- acos(cos_q3)
q2 = atan2(zz, r) - atan2(L2*sin(q3), L1 + L2*cos(q3))
```

Rjesenja izvan limita `[-pi/2, +pi/2]` se odbacuju.

## Testovi

### Test 1 - DK bez ROS-a

```bash
cd ros2_ws/src/prarob_interact
python3 - <<'PY'
from prarob_interact.kinematics import Kinematics

k = Kinematics()
xyz = k.get_dk([1.5708, 0.0, -1.5708])
print(xyz)
PY
```

Ocekivano priblizno:

```text
x ~= 0.0
y ~= 0.245
z ~= -0.067
```

### Test 2 - IK round-trip za dohvatljivu tocku

```bash
cd ros2_ws/src/prarob_interact
python3 - <<'PY'
from prarob_interact.kinematics import Kinematics

k = Kinematics()
q0 = [0.4, 0.2, -0.6]
target = k.get_dk(q0)
solutions = k.get_ik(target)

assert solutions, "IK did not return any solution"

for q in solutions:
    recovered = k.get_dk(q)
    err = max(abs(recovered[i] - target[i]) for i in range(3))
    assert err < 1e-9, f"Round-trip error: {err}"

print("target:", target)
print("solutions:", solutions)
print("OK")
PY
```

Napomena: target `[0.15, 0.10, 0.05]` nije dobar test uz trenutne limite
`[-pi/2, +pi/2]`, jer zahtijeva `joint3` izvan limita. Zato round-trip test
treba raditi iz validne konfiguracije zglobova.

### Test 3 - GUI u simulaciji

Terminal 1:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_controllv2 sim.launch.py
```

Terminal 2:

```bash
cd ~/Desktop/robo/prarob_sem16
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 GUI/robot_gui.py
```

Terminal 3:

```bash
ros2 topic echo /joint_states
```

Provjere:

- pomicanje slidera mijenja FK preview
- `Send to Robot` objavljuje na `/move_servos_node/angles`
- unos `X/Y/Z` i `Solve IK & Send` racuna rjesenje i salje kutove
- live state prikazuje trenutne zglobove i DK poziciju
- `Reset to Home` salje `[pi/2, 0, -pi/2]`
- `Emergency Stop` salje `[0, 0, 0]`

## Kamera i YOLO

### Kamera

Konfiguracija:

```text
ros2_ws/src/prarob_calib/config/camera_params.yaml
```

Trenutno:

```yaml
video_device: "/dev/video4"
image_width: 640
image_height: 480
camera_info_url: "package://prarob_calib/config/camera_calibration_params.yaml"
```

Pokretanje kalibracije:

```bash
ros2 launch prarob_calib camera_calib.launch.py
```

Pokretanje camera-to-world nodea:

```bash
ros2 launch prarob_calib camera_world.launch.py
```

### YOLO

Primjer pokretanja YOLOv12:

```bash
ros2 launch yolo_bringup yolov12.launch.py \
  input_image_topic:=/image_raw \
  device:=cuda:0
```

Ako nema CUDA GPU-a:

```bash
ros2 launch yolo_bringup yolov12.launch.py \
  input_image_topic:=/image_raw \
  device:=cpu
```

Napomena: `usb_cam` iz `prarob_calib` objavljuje `/image_raw`, dok upstream
YOLO launch default ocekuje `/camera/rgb/image_raw`. Zato treba zadati
`input_image_topic:=/image_raw` ili napraviti remapiranje.

## Status prema FAT zahtjevima

| FAT dio | Status | Detalj |
| --- | --- | --- |
| Fizicki RRR robot | Djelomicno | 3D modeli postoje; treba potvrditi stvarni print, slaganje i mehanicke tolerancije. |
| 3D print komponente | Djelomicno | STEP datoteke postoje; treba provjeriti printabilnost i potrosnju materijala. |
| Kamera | Djelomicno | Intrinzicna kalibracija i PnP node postoje; finalni camera-to-base/board transform treba validirati. |
| Direktna kinematika | Implementirano | `prarob_interact/kinematics.py` sada vraca `[x,y,z]`. |
| Inverzna kinematika | Implementirano djelomicno | Cisti IK postoji u `kinematics.py`; `robot_controllv2/ik_node.py` postoji od prije. Treba validirati na stvarnom robotu. |
| GUI design | Implementirano osnovno | `GUI/robot_gui.py` ima manual i autonomous tab. |
| Manual DK preko GUI-ja | Implementirano | Slider/spinbox za zglobove i live FK preview. |
| Manual IK preko GUI-ja | Implementirano | `X/Y/Z` unos, IK solve i slanje kutova. |
| URDF/RViz | Implementirano | URDF/Xacro i RViz config postoje u `robot_controllv2`. |
| Natural language | Implementirano | Deterministicki parser (`prarob_autonomous/command_parser.py`, EN+HR); GUI textbox publisha na `/autonomous_draw_node/command`. ROSA agent ostaje za terminal. |
| Robot vision | Implementirano | YOLO + `detection_utils.py` razlucuju connect/avoid klase; `board_calibration_node` sprema homografiju za stabilno board mapiranje. |
| Tool path planning | Implementirano | `planning_pipeline.plan_drawing()` radi A* po segmentima, izbjegava prepreke, mapira u task-space i racuna IK; pokriveno lokalnim testovima. |
| Autonomous mode | Implementirano | `autonomous_draw_node` povezuje naredbu, YOLO, planner i executor; objavljuje status (JSON) i debug overlay. |
| Time challenge | Implementirano | `challenge.launch.py` pokrece sve jednim klikom; node mjeri detect/plan/execute/total vrijeme. |

## Autonomous mode (prarob_autonomous)

Novi paket `prarob_autonomous` implementira cijeli autonomni tok. Sve geometrijske
i planerske komponente su ROS-free i pokrivene lokalnim testovima
(`test/test_core_logic.py`), pa se mogu provjeriti i bez ROS-a:

```bash
python3 ros2_ws/src/prarob_autonomous/test/test_core_logic.py
```

Moduli:

```text
command_parser.py     # "connect car and plane, avoid football" -> {connect, avoid} (EN+HR)
board_mapping.py      # piksel -> board/world [m]; homografija ili linearni fallback
detection_utils.py    # YOLO DetectionArray -> boxevi; razlucivanje connect/avoid klasa
planning_pipeline.py  # segmenti -> A* -> task-space waypointi -> IK (pen up/down)
autonomous_draw_node.py  # ROS orkestrator (status JSON + debug overlay + izvrsavanje)
board_calibration_node.py # sprema homografiju iz sahovnice u YAML
```

### Tok izvrsavanja

```text
naredba (String) -> parse (connect/avoid)
                 -> zadnje stabilne YOLO detekcije (/yolo/detections)
                 -> razlucivanje objekata + prepreka
                 -> A* planiranje po segmentima (izbjegava prosirene bboxove)
                 -> board mapping (homografija) -> task-space
                 -> IK po waypointu (closest solution) -> /move_servos_node/angles
```

Node objavljuje status kao JSON na `~/status` s fazama
`idle/detecting/planning/executing/done/failed/aborted` i mjerenjem vremena
`detect/plan/execute/total`, te debug sliku (detekcije, prosirene prepreke,
planirana putanja) na `~/debug_image`. GUI autonomous tab prikazuje oboje.

### Pokretanje autonomnog noda zasebno

Uz vec pokrenut robot/sim + kameru + YOLO:

```bash
ros2 launch prarob_autonomous autonomous.launch.py \
  board_homography_file:=$HOME/board_homography.yaml
```

Slanje naredbe bez GUI-ja:

```bash
ros2 topic pub --once /autonomous_draw_node/command std_msgs/msg/String \
  "{data: 'connect car and plane, avoid football'}"
```

Prekid:

```bash
ros2 topic pub --once /autonomous_draw_node/command std_msgs/msg/String \
  "{data: 'stop'}"
```

### Board kalibracija (camera-to-board homografija)

Postavi sahovnicu na plocu i pokreni (uz kameru):

```bash
ros2 run prarob_autonomous board_calibration_node \
  --ros-args -p output_file:=$HOME/board_homography.yaml
```

Node detektira sahovnicu, fita homografiju piksel->robot XY (DLT), ispise srednju
reprojekcijsku gresku u mm i spremi YAML. Taj YAML se preda autonomnom nodu preko
`board_homography_file`. Ako YAML ne postoji, node koristi linearni
`workspace_*` mapping iz `config/autonomous_params.yaml`.

## Time challenge (jedan klik)

```bash
ros2 launch prarob_autonomous challenge.launch.py \
  use_sim:=true device:=cpu \
  board_homography_file:=$HOME/board_homography.yaml
```

Argumenti: `use_sim` (true=mock sim, false=stvarni robot), `use_camera`,
`use_yolo`, `use_gui`, `device` (`cpu`/`cuda:0`), `board_homography_file`,
`gui_path`. Launch pokrece robota/sim, `usb_cam`, YOLOv12 (remap na `/image_raw`),
`autonomous_draw_node` i PyQt GUI. Upises naredbu u GUI autonomous tab ili je
objavis na `/autonomous_draw_node/command`; vrijeme po fazama je u statusu.

## Sto jos treba validirati na stvarnom robotu

```text
1. DK/IK i limiti zglobova (+-90 deg) na stvarnoj mehanici - dohvatljivi dio ploce.
2. board_homography.yaml izmjeriti na stvarnom postavu (sahovnica + bazna mjera).
3. Ugoditi drawing_z / pen_up_z prema stvarnoj visini markera.
4. Provjeriti YOLO klase/threshold za stvarne piktograme (po potrebi custom model).
5. Ugoditi seconds_per_waypoint i cell_size/margin_cells za brzinu vs. tocnost.
```

## Plan za path planning

Postojece:

```text
ros2_ws/src/prarob_interact/prarob_interact/path_planning.py
```

Trenutno moze:

- napraviti occupancy grid iz YOLO bounding boxova
- planirati A* putanju u image-spaceu
- pojednostaviti putanju line-of-sight provjerom
- mapirati image path u task-space linearnim mappingom

Za dovrsiti:

### 1. Planirati u koordinatama ploce

Umjesto privremenog linearnog image-to-task mappinga:

```text
image pixel -> board coordinate [m] -> robot world coordinate [m]
```

Treba koristiti homografiju ili PnP rezultat iz kalibracije.

### 2. Napraviti prepreke

Za svaki `avoid` objekt:

```python
obstacle = {
    "class_name": "football",
    "polygon": [...],
    "margin": marker_radius + calibration_error
}
```

Grid celije unutar prosirenog poligona oznaciti kao zauzete.

### 3. Planirati segmente

Za naredbu:

```text
connect car and plane and bottle, avoid football
```

tok:

```text
car -> plane
plane -> bottle
```

Ako ima vise objekata, redoslijed se moze izabrati brute-force TSP-om jer je
broj objekata mali.

### 4. Pretvoriti putanju u waypointe

Primjer strukture:

```python
waypoints = [
    {"x": x0, "y": y0, "z": pen_up_z},
    {"x": x0, "y": y0, "z": drawing_z},
    {"x": x1, "y": y1, "z": drawing_z},
    {"x": x2, "y": y2, "z": drawing_z},
    {"x": x2, "y": y2, "z": pen_up_z},
]
```

### 5. Validirati prije izvrsenja

Prije slanja na robota:

```text
- svaki waypoint ima IK rjesenje
- putanja ne ulazi u avoid poligon
- putanja je unutar dosega robota
- broj waypointa nije nepotrebno velik
```

## Plan za autonomous mode

Predlozeni novi node:

```text
prarob_autonomous/autonomous_draw_node.py
```

Minimalni API:

```text
Service: /autonomous_draw/run
Type: custom service ili std_srvs + command topic
Input: natural language command
Output: status + debug message
```

Tok:

```text
1. GUI ili ROSA posalje naredbu.
2. Parser izvuce connect i avoid klase.
3. Node uzme zadnje stabilne YOLO detekcije.
4. Detekcije se pretvore u board/world koordinate.
5. Planner izracuna putanju.
6. Executor salje waypointe na /ik_node/xyz ili direktno kutove na /move_servos_node/angles.
7. Node vrati status: done ili failed.
```

Primjer parser outputa:

```json
{
  "connect": ["car", "plane"],
  "avoid": ["football"]
}
```

Statusi:

```text
idle
detecting
planning
executing
done
failed
```

GUI bi trebao samo pozvati backend, ne implementirati svu logiku unutar Qt koda.

## Plan za time challenge

Time challenge treba biti jedan klik, ne rucno pokretanje vise terminala.

Predlozeni launch:

```bash
ros2 launch prarob_seminar challenge.launch.py
```

Taj launch treba pokrenuti:

```text
usb_cam
yolo_bringup
yolo_processing
robot_controllv2
autonomous_draw_node
GUI
```

Priprema prije mjerenja:

```text
1. YOLO model ucitati unaprijed.
2. Kalibraciju ucitati iz spremljene datoteke.
3. Robot resetirati u HOME pozu.
4. Provjeriti da /joint_states dolazi.
5. Provjeriti da /yolo/detections dolazi.
```

Optimizacije:

```text
- koristiti najmanji YOLO model koji dovoljno dobro detektira objekte
- cacheirati stabilne detekcije
- planirati na gridu od 2-5 mm
- smoothing smanjiti broj waypointa
- slati manje, ali dovoljno guste waypointe
- logirati trajanje faza: detect, plan, execute
```

GUI za challenge:

```text
[Command textbox]
[Run challenge]
[Stop]
[Status: detecting/planning/executing/done]
[Timing: total, detect, plan, execute]
[Debug image path]
```

## Poznati problemi / napomene

- `master.launch.py` je jos placeholder; za cijeli sustav koristiti
  `prarob_autonomous/challenge.launch.py`.
- `Autonomous Mode` u GUI-ju je sada spojen na backend (command/status/timing/
  debug overlay).
- `planning_pipeline` je pokriven lokalnim testovima, ali jos nije validiran na
  stvarnom robotu (geometrija, limiti, brzina).
- `camera_to_world.py` racuna PnP uzivo; `board_calibration_node` taj transform
  pretvara u homografiju i sprema u YAML koji koristi cijeli pipeline.
- Zbog limita zglobova (+-90 deg) dohvatljiv je samo dalji prsten radnog
  prostora; kalibraciju i `workspace_*`/homografiju treba postaviti unutar tog
  prstena (vidi test `test_plan_avoids_obstacle_and_is_reachable`).
- RViz preko SSH-a moze imati X/Wayland probleme. Lokalno na VM desktopu je
  najjednostavnije pokretati launch iz terminala unutar same VM graficke
  sesije.
- Ako `sim.launch.py` javi da ne moze ucitati
  `joint_trajectory_controller/JointTrajectoryController`, instalirati:

```bash
sudo apt-get install ros-jazzy-joint-trajectory-controller
```

## Brzi cheat sheet

Build:

```bash
cd ~/Desktop/robo/prarob_sem16/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Sim:

```bash
ros2 launch robot_controllv2 sim.launch.py
```

GUI:

```bash
cd ~/Desktop/robo/prarob_sem16
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 GUI/robot_gui.py
```

Echo joints:

```bash
ros2 topic echo /joint_states
```

Send joints:

```bash
ros2 topic pub --once /move_servos_node/angles std_msgs/msg/Float32MultiArray \
  "{data: [1.5708, 0.0, -1.5708]}"
```

Send XYZ:

```bash
ros2 topic pub --once /ik_node/xyz geometry_msgs/msg/Point \
  "{x: 0.15, y: 0.05, z: 0.02}"
```
