> Current entry point: `python3 main.py --model gemini-2.5-flash` (same as `run_session.py`). Start with [WORKFLOW.md](WORKFLOW.md) for the complete scan-to-task sequence. The old demo is now launched explicitly with `python3 legacy/run_demo.py`.

# Beginner guide: setup and first robot navigation

This guide assumes no prior knowledge of this project. Commands below are for a
Linux terminal. CoppeliaSim is the separate application that displays and
simulates the robots.

## What the project currently does

A robot uses a LiDAR scanner to measure distances to walls and objects. A saved
map represents those measurements as pixels: black is an obstacle, white is
observed free space, and gray is unknown space. The navigator plans a route
through free space and commands the robot's wheels to follow it.

The latest map is already included in `maps/mapping_fixed/`. You do **not** need
to scan the environment again to try navigation. Robot A has completed multiple
navigation tests. Coordinated task execution for 5–10 robots is future work.

Mapping currently uses simulator positions to align scans. Navigation starts
with a supplied position and then uses wheel rotations and LiDAR scan matching
to estimate where the robot is. Experimental SLAM map extension exists, but is
not needed for this walkthrough and has no global loop closure.

## 1. Get the project

You need Git, Python 3.10 or newer, and CoppeliaSim with its ZMQ Remote API.
Install CoppeliaSim from its official download page:
https://www.coppeliarobotics.com/downloads

If you do not already have the project:

```bash
git clone https://github.com/Shiva-Shankar-Dev/llm-guided-task-allocation.git
cd llm-guided-task-allocation
```

If your existing checkout is called `minor`, use its location instead:

```bash
cd ~/minor
```

Run all remaining commands from that project directory. Do not clone another
copy if you already have the checkout you intend to use.

## 2. Prepare Python

Create an isolated Python environment and install dependencies:

```bash
python3 -m venv venv
source venv/bin/activate
python3 -m pip install -r requirements.txt
```

The environment must be activated again whenever you open a new terminal:

```bash
cd ~/minor
source venv/bin/activate
```

Use your actual project folder if it has another name. No Gemini API key is
required for mapping or saved-map navigation.

## 3. Open the simulation scene

1. Launch CoppeliaSim.
2. Use File → Open scene and select `scenes/environment.ttt` in the project.
3. Confirm Robot A and the room are visible.
4. Leave simulation stopped. Do not press Play: the Python controller starts it.
5. Do not run `main.py`, another navigator, or a mapping script simultaneously.

The scene and map must describe the same walls, obstacles, and coordinate frame.
Do not move walls or boxes while using the supplied map.

## 4. Find Robot A's starting position

Run:

```bash
python3 inspect_robot.py
```

Example output:

```text
--initial-pose -1.0750 -0.8750 0.000
```

These are X and Y in metres and heading in degrees. Copy **your** output.
The example may differ from your robot's current position.

## 5. Choose a destination

Run:

```bash
python3 choose_goal.py
```

A map window opens. The terminal prints the loaded map path; it should end in
`maps/mapping_fixed/map.yaml`. Click white space away from walls and boxes.
For a first test, select a nearby destination. The window closes and prints,
for example:

```text
--goal 5.090 -0.810
```

Copy your two destination numbers. They are metres in the map frame, not pixels.

## 6. Preview the route without moving

Combine the numbers from the previous two steps. For the example above:

```bash
python3 navigate_map.py \
  --initial-pose -1.0750 -0.8750 0.000 \
  --goal 5.090 -0.810 \
  --plan-only
```

Replace those numbers with your outputs. `--plan-only` prevents simulator
connection and movement. Open `results/navigation/path.png` in an image viewer:
blue marks the start, red marks the goal, and green shows the route.
A route across the partition must go around its free end.

## 7. Drive to the destination

Keep simulation stopped and leave Robot A at the same starting position.
Run the same command with `--plan-only` removed:

```bash
python3 navigate_map.py \
  --initial-pose -1.0750 -0.8750 0.000 \
  --goal 5.090 -0.810
```

Watch the simulation. The terminal prints the estimated position and a scan
alignment score. The script checks LiDAR obstacles and stops if navigation
becomes unreliable. Press Ctrl+C in the terminal to stop it yourself.

Successful completion prints `Goal reached`, then stops simulation. CoppeliaSim
may restore the robot's starting pose when stopping. Before another run, repeat
`inspect_robot.py` rather than assuming the robot stayed at its destination.
You do not need to close and reopen CoppeliaSim between runs.

## Optional: add more scans

Only do this if the room has unobserved regions. Keep the same scene, stop
simulation, and run:

```bash
python3 map_environment.py --manual \
  --resume maps/mapping_fixed/map_data.npz \
  --output results/mapping_next --duration 600
```

Focus the terminal. W moves forward, A/D turn, space stops, and Q saves and
exits. Tap or hold a movement key; commands expire shortly after input stops.
There is no reverse command. Drive around obstacles to observe their far sides.
The new scan is saved separately; the supplied map is preserved.

To choose a goal and navigate on this new scan, pass the same map to both tools:

```bash
python3 choose_goal.py --map results/mapping_next/map.yaml
python3 navigate_map.py --map results/mapping_next/map.yaml --initial-pose X Y HEADING --goal GOAL_X GOAL_Y --plan-only
```

In the second command, replace the uppercase placeholders with actual numbers.
Map YAML, PGM, and NPZ files have different roles: YAML supplies coordinates and
scale, PGM is the occupancy image, and NPZ retains evidence for map resumption.
Keep each map's files together.

## Common problems

| Problem | What to do |
| --- | --- |
| Python cannot find a module | Activate `venv` and rerun `python3 -m pip install -r requirements.txt`. |
| Connection refused or cannot connect | Open CoppeliaSim with the scene; check its ZMQ Remote API add-on is available. |
| Simulation must be stopped | Press Stop, wait for simulation to stop, then rerun the script. |
| Goal picker opens the wrong map | Close the old window and run `python3 choose_goal.py --map maps/mapping_fixed/map.yaml`. |
| Map file not found | Run from the project directory; use `maps/mapping_fixed/`, not its old `results/` location. |
| Goal is blocked or no path exists | Pick white space farther from walls, boxes, and gray areas. |
| Localization unreliable | Stop; inspect the initial pose again and confirm the scene matches the map. Do not bypass the score check. |
| Safety-space error or no progress | The robot has stopped. Save the terminal output, inspect the pose after stopping, and investigate clearance or blockage before retrying. |
| Goal window does not appear | Use a desktop terminal with graphical display support and check matplotlib installation. |

Default wheel dimensions and clearance are configured for the current robot.
Changing robot geometry requires measuring its wheels and footprint; see
[MAPPING.md](MAPPING.md) before overriding those settings.

## Other files and next steps

- `robot_navigation/`: implementation. Root Python files are convenient launchers.
- `scenes/`: simulator scene.
- `maps/mapping_fixed/`: latest supplied scan.
- `results/`: generated paths, logs, and experimental maps; ignored by Git.
- `tests/`: offline checks. Run `python3 -m unittest discover -s tests -v`.
- [PROJECT_TODO.md](PROJECT_TODO.md): completed milestones and remaining fleet work.
- [MAPPING.md](MAPPING.md): detailed mapping/navigation settings and experimental SLAM.

The separate `python3 main.py` command runs the original language-driven task
demo and requires `GOOGLE_API_KEY` in the environment. It still uses its older
routing method and is not connected to saved-map fleet navigation. Use the
navigation walkthrough above first.

## Next milestone: test Robot B independently

The navigator and pose inspector accept `--robot Robot_B`. Robot paths and
wheel dimensions are in `config/robots.json`; Robot A remains the default.

1. Stop simulation. Ensure Robot B has a rigidly mounted, working fastHokuyo
   model with enabled simulation and vision scripts. Its configured path is
   `/Robot_B/lidar`; update the JSON if your scanner has another name.
   Scanner height must clear the body and intersect walls and obstacles.
2. Keep Robot A stationary, with its motor targets zero and other controllers
   disabled. The navigator commands only the selected robot; it no longer sets
   another robot's wheel targets.
3. Run `python3 inspect_robot.py --robot Robot_B` and copy its initial pose.
4. Run `python3 choose_goal.py` and choose nearby white space clear of Robot A.
5. Run `python3 navigate_map.py --robot Robot_B --initial-pose X Y HEADING --goal GOAL_X GOAL_Y --plan-only`, replacing the placeholders with your outputs.
6. Inspect the path, then remove `--plan-only` for a watched trial.

Only run one navigator at a time. Simultaneous robot operation needs the central
fleet loop and traffic coordination, which are not implemented yet. Validate
Robot A again after this change, then Robot B separately. New configuration and
offline tests do not establish live multi-robot functionality.
