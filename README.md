# Autonomous Vehicle SIL-to-HIL Testing

This repository documents a staged autonomous-vehicle testing workflow developed with **CARLA** and **Gazebo Harmonic**. The project progresses from early perception and Automatic Emergency Braking (AEB) experiments to off-road sensor fusion, obstacle avoidance and multi-vehicle coordination in the Ruskotunturi environment.

The current baseline is a **multi-vehicle Software-in-the-Loop (SIL) demo in Gazebo**. The next planned stage is deployment toward **Hardware-in-the-Loop (HIL)** using a Raspberry Pi and CAN interface.

## Project goals

The project is intended to provide a reproducible workflow for developing and validating autonomous-vehicle functions in stages:

1. develop and test ADAS logic in simulation,
2. use virtual sensors for perception and safety,
3. validate functions independently before integrating them,
4. record repeatable PASS / FAIL results,
5. combine navigation, perception, safety and coordination,
6. prepare the same controller architecture for later embedded HIL testing.

The repository intentionally retains the earlier CARLA work as evidence of the development path rather than treating it as obsolete code.

## Development stages

### 1. CARLA perception and AEB

The first stage used CARLA for basic virtual-vehicle and ADAS development. It includes:

- RGB camera processing,
- edge detection and path prediction,
- static-obstacle AEB,
- repeatable target-speed tests,
- TTC-based braking,
- moving-obstacle / relative-TTC tests,
- route-navigation and early multi-vehicle coordination experiments.

The CARLA SIL scripts are mainly under:

```text
edge-path-project/
edge-path-project/aeb_sil/
```

CARLA test results are stored centrally in:

```text
edge-path-project/results/
```

### 2. Gazebo ADAS port

The controller workflow was then ported to Gazebo Harmonic. This phase validates the same general ADAS concepts with Gazebo vehicle models and virtual sensors.

Implemented Gazebo functions include:

- Automatic Emergency Braking,
- TTC-based collision avoidance,
- LiDAR obstacle avoidance,
- 360-degree LiDAR safety monitoring,
- RGB camera input,
- automated PASS / FAIL tests,
- CSV result logging.

Gazebo files are organised as:

```text
edge-path-project/gazebo/
├── models/
├── scripts/
├── worlds/
└── results/
```

### 3. Ruskotunturi off-road testing

The Ruskotunturi terrain is used as the main off-road SIL environment.

The staged tests include:

- baseline terrain navigation,
- static-obstacle avoidance,
- moving-obstacle interaction,
- oncoming TTC testing,
- camera + LiDAR sensor fusion,
- adaptive avoidance behaviour,
- two-vehicle coordination,
- final integrated multi-vehicle SIL demo.

The vehicle models use virtual sensors including:

- 360-degree LiDAR,
- front RGB camera,
- Gazebo pose / odometry data for mission and coordination logic.

## Final Ruskotunturi SIL demo

The final scenario combines the main project functions into one integrated run.

```text
Vehicle A --------\
                   \
                    MERGE -> SHARED ROUTE -> COMMON LOADING ZONE
                   /
Vehicle B --------/
```

Both vehicles start on the same side of the map on separate approach routes. The routes converge at a merge point and then use the same final route toward the common summit loading zone.

### Vehicle A

Vehicle A encounters a static obstacle before the merge.

- LiDAR provides range and hard-safety monitoring.
- The RGB camera provides semantic confirmation of the orange target.
- Camera + LiDAR fusion activates local obstacle avoidance.
- The avoidance controller uses a continuous offset-line path rather than a fixed waypoint orbit.

### Vehicle B

Vehicle B encounters a moving obstacle before the merge.

- The obstacle is unknown to the vehicle controller.
- Local LiDAR detects the collision risk.
- Vehicle B performs a stop / resume response based on LiDAR clearance.

### Merge coordinator

The merge is managed by a coordinator rather than a dedicated waiting area.

- Vehicle A has merge priority.
- Vehicle B receives a dynamic velocity profile while approaching the merge.
- The coordinator can slow or temporarily stop B if required.
- B receives merge permission after A creates sufficient headway.
- After the merge, both vehicles follow the same route.

Local safety remains independent of the coordinator. Each vehicle has a 360-degree LiDAR hard-safety layer that can stop the mission if the required body clearance is violated.

### Final SIL safety requirement

```text
Required body clearance: 1.00 m
```

The final recorded runs include successful PASS executions with:

- both vehicles completing their mission,
- static avoidance completed,
- camera used for Vehicle A control,
- moving obstacle handled by Vehicle B,
- merge coordinator profile / stop / release executed,
- no loading-zone overlap,
- no terrain safety violation,
- no 360-degree LiDAR safety violation.

The corresponding CSV is:

```text
edge-path-project/gazebo/results/ruskotunturi_final_sil_demo_results.csv
```

## Running the final Gazebo SIL demo

The current development environment is **WSL Ubuntu 24.04** with **Gazebo Harmonic**.

From Windows, start Ubuntu:

```bat
wsl -d Ubuntu-24.04
```

Go to the repository:

```bash
cd ~/edge-path-project-git
```

Set the Gazebo model resource path:

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/edge-path-project-git/edge-path-project/gazebo/models:$GZ_SIM_RESOURCE_PATH"
```

Start the final Ruskotunturi world:

```bash
gz sim edge-path-project/gazebo/worlds/ruskotunturi_final_sil_demo_world.sdf
```

Press **Play** in Gazebo.

Open a second WSL terminal and run:

```bash
cd ~/edge-path-project-git
python3 edge-path-project/gazebo/scripts/ruskotunturi_final_sil_demo.py
```

A successful run ends with:

```text
*** FINAL SIL DEMO COMPLETE ***
RESULT: PASS
```

## Repository structure

```text
EdgeDetection/
├── README.md
└── edge-path-project/
    ├── aeb_sil/                 # CARLA SIL controllers and test scenarios
    ├── results/                 # CARLA / early SIL CSV results
    ├── gazebo/
    │   ├── models/              # Gazebo vehicles and Ruskotunturi model
    │   ├── scripts/             # Gazebo controllers and automated tests
    │   ├── worlds/              # Gazebo test environments
    │   └── results/             # Gazebo and Ruskotunturi CSV results
    ├── carla_*.py               # Earlier CARLA experiments
    ├── main.py
    └── environment.yml
```

The older CARLA files are retained intentionally to document the progression from initial virtual-sensor experiments to the current Gazebo multi-vehicle SIL system.

## Testing philosophy

The project uses staged validation. Individual functions are tested separately before integration into the final scenario.

Examples include:

- AEB tests,
- TTC tests,
- LiDAR avoidance tests,
- static and moving Ruskotunturi obstacle tests,
- camera + LiDAR fusion,
- standalone two-vehicle coordination,
- final integrated SIL demo.

This helps keep failures diagnosable and makes the workflow easier to reproduce.

## Results

CSV result files are committed intentionally as validation evidence.

They should not be globally ignored by Git because they document the behaviour of the controller at different development stages.

The two main result locations are:

```text
edge-path-project/results/          # CARLA / early SIL
edge-path-project/gazebo/results/   # Gazebo / Ruskotunturi SIL
```

## Next stage: HIL

The next project stage is Hardware-in-the-Loop validation.

The intended architecture is:

```text
Gazebo virtual sensors
        |
        v
communication / signal mapping
        |
        v
Raspberry Pi + CAN interface
        |
        v
ADAS / coordination controller
        |
        v
actuator commands
        |
        v
Gazebo vehicle
```

Planned HIL work includes:

- Raspberry Pi deployment,
- CAN communication,
- mapping virtual sensor and actuator signals to CAN signals,
- packaging the controller for deployment,
- maintaining the same scenario and validation logic across SIL and HIL.

## Current status

**CARLA development:** retained and documented  
**Gazebo SIL:** implemented  
**Ruskotunturi final multi-vehicle SIL demo:** PASS baseline available  
**Raspberry Pi / CAN HIL:** next phase
