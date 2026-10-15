# Start-to-finish workflow

Always run commands from `~/minor`. Open `~/minor/scenes/environment.ttt` in CoppeliaSim. Stop simulation before starting a Python controller; run only one controller at a time.

**You already have a map. For everyday use, skip to step 4.** Do not rescan simply because a task finishes or a robot waits.

## 1. Scan a new area (only when needed)

For a fresh scan in a new output directory:

```bash
cd ~/minor
python3 map_environment.py --manual --output results/scans/new_area_01 --duration 600
```

Keep focus in the terminal. W = forward, A/D = rotate, space = stop, Q = save and exit. Keep tapping/holding a key to move; motion expires after a short interval without input. Drive around obstacles and the open end of the partition to observe hidden areas. Other robots should remain still; their bodies can appear in scans. Use a new output name for each experiment so earlier scans remain available.

To extend the existing map in the same unchanged scene/world coordinate frame:

```bash
python3 map_environment.py --manual --resume maps/mapping_fixed/map_data.npz --output results/scans/extended_01 --duration 600
```

The scanner saves periodically and on exit:

| File | Meaning |
|---|---|
| `map.pgm` | Map picture: black obstacles, white observed free space, gray unknown |
| `map.yaml` | Image reference, resolution and world origin; pass this to navigation |
| `map_data.npz` | Raw mapping evidence; use this to resume scanning |

Open the chosen output directory in the file manager to see these files. Mapping aligns scans using simulator poses; it is distinct from localization during navigation.

## 2. Inspect and try the new map

View `results/scans/extended_01/map.pgm`. Walls should align without curved trails in free space. Gray regions may need another viewing angle; gray inside solid obstacles is normal.

Choose a test goal on this map:

```bash
python3 choose_goal.py --map results/scans/extended_01/map.yaml
python3 inspect_robot.py --robot Robot_A
```

Use the numbers printed by those tools in this template (replace every capitalized placeholder):

```bash
python3 navigate_map.py --robot Robot_A --map results/scans/extended_01/map.yaml --initial-pose X Y YAW_DEG --goal GOAL_X GOAL_Y --plan-only --output results/navigation/map_trial
```

Inspect `results/navigation/map_trial/path.png` and `path.csv`. Remove `--plan-only` for a watched navigation test with the same starting pose. Planning alone does not move anything.

## 3. Select the map and configure places

The approved default map remains `maps/mapping_fixed/map.yaml`. Scanning into `results/` does **not** replace it automatically. To try another map with the fleet, explicitly pass it:

```bash
python3 main.py --model gemini-2.5-flash --map results/scans/extended_01/map.yaml
```

Keep YAML, PGM and NPZ together. Do not mix files from different scans. Retain the approved map until a replacement has been checked.

`config/robots.json` contains model/motor/LiDAR paths and robot dimensions. `config/stations.json` contains destination aliases and arrival slot coordinates in world metres. Use `choose_goal.py` to obtain coordinates, then edit slot values there. Provide distinct separated slots for robots visiting the same place. Check that each slot lies inside the intended physical area and away from walls. A new map or changed scene requires rechecking the slots.

## 4. Run tasks (normal daily workflow)

With CoppeliaSim stopped:

```bash
cd ~/minor
python3 main.py --model gemini-2.5-flash
```

At the prompt:

```text
Send robots to charging_station and loading_zone.
```

Or, to exercise separate arrival slots at one destination:

```text
Send both robots to the loading dock.
```

The program starts simulation and reads starting poses once. It interprets the instruction, allocates routes, and executes tasks. It holds the current positions and prompts again after the batch finishes. Do not press Stop between batches. New instructions are currently accepted between batches, not during motion.

Commands: `/status`, `/stations`, `/intent config/example_intent.json` (offline, no LLM call), `/quit`. Ending the program stops simulation and restores initial scene positions. Failed robots remain unavailable until the session ends. Automatic reassignment may mean one robot completes both tasks; inspect the robot names in completion records.

## 5. Find your outputs

The console prints the exact session folder, for example `results/sessions/<timestamp>/`.

| Output | What to check |
|---|---|
| `batch_001/intent.json` | Structured tasks returned by interpretation |
| `batch_001/allocation.json` | Assigned robots and route-distance comparison |
| `batch_001/mission.json` | Initial estimates, robot goals, station and slot |
| `batch_001/llm_usage.json` | Application-level API invocation count and available tokens |
| `events.jsonl` | Timeline: running, waiting reasons, reassignment, completion |
| `tasks.json` | Final task outcomes and arrival evidence |
| `session_state.json` | Latest positions and availability at the input prompt |

Batch 2 goes in `batch_002/`, and so on. Session task/event logs cover all batches. Session navigation does not automatically produce path PNGs. To preview a saved mission without moving robots:

```bash
python3 run_fleet.py --mission results/sessions/SESSION_FOLDER/batch_001/mission.json --plan-only --output results/previews/session_trial
```

Replace `SESSION_FOLDER` with the printed folder name. Static previews do not include subsequent moving obstacles. Other commands have their own default outputs: standalone navigation in `results/navigation/`, allocation in `results/allocation/`, one-shot fleet in `results/fleet/`. Older experiment folders remain preserved.

## Simulation warnings

Yellow triangles beside `connection[...]` indicate objects that cannot participate in dynamics as configured. These appear to be force-sensor connection objects in this scene, but the screenshot alone does not establish the precise cause. During simulation, double-click the warning icon to read CoppeliaSim's diagnostic. A missing valid dynamic shape connection is one possible cause. Do not delete these objects or change the whole robot to static just to hide the warning.

Reference: [CoppeliaSim dynamic simulation guide](https://manual.coppeliarobotics.com/en/designingDynamicSimulations.htm). Share the diagnostic text to determine whether the connections are unused attachment points or a model structure problem requiring repair.

## Adjust cruising speed

The default cruise speed is 0.12 m/s. To trial 0.18 m/s, exit the existing session and restart with CoppeliaSim stopped:

```bash
python3 main.py --model gemini-2.5-flash --speed 0.18
```

This raises straight-line requested speed by 50%; uninterrupted travel time would fall by about one third. Turns, traffic waits, obstacle stops and computation still add time, so overall duration will not necessarily improve by that amount. Wheel limits, LiDAR stops, predicted motion checks and separation checks remain active. Margin recovery is still capped at 0.04 m/s. Speed is limited to at most 0.18 m/s in this prototype. Default 0.12 is unchanged; the faster setting requires a watched live trial. Console timestamps are simulation seconds, not necessarily elapsed wall-clock time.
