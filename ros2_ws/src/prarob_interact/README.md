# prarob_interact
Repozitorij za seminarski zadatak kolegija "Praktikum robotike". (https://www.fer.unizg.hr/predmet/prarob)

## Upute
Ovaj paket pruža podlogu za početak implementacije seminarskog zadatka iz osnova robotike, namjenjen je za korištenje u ROS2 Jazzy distribuciji.

Implementacija se temelji na AI agentu ROSA namjenjenom za upravljanjem ROS i ROS2 sustavima. (https://github.com/nasa-jpl/rosa)

ROSA agent koristi API poziv za komunikaciju s LLM modelom i generiranje odgovora unesenom tekstu uz dodatne ROS mogućnosti, kao što je pregled dostupnih servisa.

Osim definicije LLM modela i dodatne konfiguracije osnovnog ponašanja agenta, moguće je definirati dodatne alate koje agent može koristiti.

Kako biste koristili agenta, trebate definirati API ključ u izvršnom okruženju.

Ključ će naknadno biti dostupan, a postaviti ga možete u .env datoteci. (pogledajte .env.example)

Prekopirajte ovaj repozitorij u svoj radni prostor:

```
cd ~/ros2_ws/src
git clone https://github.com/larics/prarob_interact.git
```

Instalirajte potrebne pakete koristeći pip:

```
pip3 install dotenv
pip3 install langchain_openai
pip3 install jpl-rosa
```

Nakon toga, pokrenite standardne ros naredbe za pripremu paketa:

```
cd ~/ros2_ws
colcon build --packages-select prarob_interact # Izborno dodajte --merge-install ili --symlink-install
source install/setup.bash
```

Agenta možete pokrenuti naredbom:
```
ros2 run prarob_interact text_interface
```

Kada razvijete novu funkcionalnost sustava, omogućite agentu korištenje tog koda definirajući novi alat u *tools.py*.

Među unarijed spremnim alatima možete naći alate koji pozivaju prazne metode za direktnu i inverznu kinematiku iz *kinematics.py*, ako i neimplementirani alat za filtriranje yolo detekcije.

Na vama je da implementirate alat za filtriranje yolo detekcije kao i metode direktne i inverzne kinematike sukladno parametrima vašeg robota.

## Planiranje putanje markera

U paketu je dodan modul `prarob_interact.path_planning` i ROSA alati:

- `plan_path` planira putanju u koordinatama slike pomoću A* algoritma i YOLO bounding boxove tretira kao prepreke.
- `execute_task_path` šalje listu `[x, y, z]` točaka na postojeći `/ik_node/xyz` topic.
- `plan_and_execute_path` kombinira oba koraka: planira image-space putanju, linearno ju pretvara u task-space i objavljuje waypointe IK nodeu.

`start` i `goal` mogu biti `{"x": px, "y": py}` ili YOLO box oblika `{"start_x": ..., "start_y": ..., "end_x": ..., "end_y": ...}`. `obstacles` je lista YOLO boxova koje treba izbjeći.

Primjer poziva alata:

```python
plan_and_execute_path(
    start={"x": 120, "y": 220},
    goal={"x": 520, "y": 240},
    obstacles=[{"start_x": 290, "start_y": 190, "end_x": 360, "end_y": 280}],
    workspace_x_min=0.06,
    workspace_x_max=0.28,
    workspace_y_min=-0.14,
    workspace_y_max=0.14,
    drawing_z=0.0,
)
```

Vrijednosti `workspace_x_min`, `workspace_x_max`, `workspace_y_min` i `workspace_y_max` treba uskladiti s kalibracijom kamere i stvarnim granicama ploče za crtanje.
