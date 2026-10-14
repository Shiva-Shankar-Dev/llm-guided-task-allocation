# Multi-Robot Natural Language Navigation

CoppeliaSim project for LiDAR mapping, saved-map navigation, and future fleet
coordination. Robot A has reached multiple selected goals, including a route
around the partition, using wheel odometry and local LiDAR scan matching.
Five-to-ten-robot task execution remains planned work; see the
[project checklist](docs/PROJECT_TODO.md).

## Project structure

```text
robot_navigation/          Python implementation
  map_environment.py      LiDAR mapping, keyboard driving, map resumption
  navigate_map.py          A*, scan matching, odometry, obstacle checks
  choose_goal.py           Click a destination on the map
  inspect_robot.py         Read the initial robot pose
  legacy_tasks.py          Original Gemini two-robot task demo
  paths.py                 Shared canonical map path
maps/mapping_fixed/        Latest scan: PGM, YAML, raw NPZ grid (tracked)
scenes/environment.ttt    CoppeliaSim scene, including current local scene edits
docs/                     Mapping instructions and project checklist
tests/                    Offline regression tests
requirements/             Mapping dependencies and historical alternative list
results/                  Generated paths, logs, and experimental maps (ignored)
```

Root Python scripts are small compatibility entry points, so familiar commands
such as `python3 navigate_map.py` still work. Implementation lives in the package.
The original `environment.ttt` is now `scenes/environment.ttt`. Older and
experimental scans remain locally under ignored `results/`; the authoritative
updated scan is **`maps/mapping_fixed/`**. Its three files were moved unchanged.

## Setup

Use Python 3.10+ and a CoppeliaSim installation with the ZMQ Remote API available:

```bash
cd /home/sshankar/minor
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` installs the current mapping tools and the Gemini client used
by the legacy demo. Mapping alone needs `requirements/mapping.txt`.
`requirements/legacy-alternative.txt` preserves the old alternative dependency
list for reference; it is not needed for the current workflow.

Open `scenes/environment.ttt` in CoppeliaSim. Stop simulation before launching a
controller. Use one controller at a time. The Python tools start and stop
simulation themselves. The scene needs `/Robot_A`, its `leftMotor` and
`rightMotor`, and `/Robot_A/fastHokuyo` with its standard sensor scripts enabled.

## Navigate on the latest map

The goal picker and navigator both default to the repository's
`maps/mapping_fixed/map.yaml`, independently of the shell's current directory.
The YAML references the PGM and contains its scale and origin.

```bash
python3 inspect_robot.py
python3 choose_goal.py
```

Use the printed pose and selected goal values (replace the capitalized tokens):

```bash
python3 navigate_map.py --initial-pose X Y YAW_DEG --goal GOAL_X GOAL_Y --plan-only
```

Inspect `results/navigation/path.png`, then remove `--plan-only` to execute.
Confirm the starting pose again if the robot has moved. Ctrl+C stops execution.
Defaults match the current Pioneer-style model; verify wheel dimensions and
footprint before using a different robot.

Package commands also work, for example:

```bash
python3 -m robot_navigation.choose_goal
```

## Extend the map

The canonical map is preserved when collecting additional scans into `results/`:

```bash
python3 map_environment.py --manual --resume maps/mapping_fixed/map_data.npz --output results/mapping_next --duration 600
```

W moves forward, A/D rotate, space stops, and Q saves/exits. Mapping uses simulator
poses; saved-map navigation uses an initial pose followed by odometry and local
scan matching. Experimental `navigate_map.py --slam` extends a map using estimated
poses, but has no global loop closure and has not been validated as a full SLAM
system. See [the complete mapping guide](docs/MAPPING.md).

## Original language-task demo

`python3 main.py` runs the original Gemini-driven two-robot demo. Supply
`GOOGLE_API_KEY` in the environment. This demo still uses predefined stations,
simulator poses, ultrasonic checks, and hardcoded partition routing; it is not
connected to the saved-map navigator yet. Its logs are written under
`results/logs/`. Do not run it alongside mapping or navigation.

## Validation and remaining work

```bash
python3 -m unittest discover -s tests -v
```

Offline tests cover map resumption, maximum-range readings, map coordinates,
clearance, planning, odometry, scan matching, and predicted-motion stopping.
Live single-robot goal completion was verified by the user. Fleet synchronization,
traffic reservations, task assignment, and scaling to 5–10 robots are tracked in
[PROJECT_TODO.md](docs/PROJECT_TODO.md).
