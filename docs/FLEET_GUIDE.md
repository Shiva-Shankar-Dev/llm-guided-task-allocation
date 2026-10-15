# First two-robot fleet test

Robot A and Robot B have each reached a selected goal independently (user
verified). This milestone tests both in **one Python process**, using one shared
simulation-stepping loop. It does not run two copies of `navigate_map.py`.

## What is implemented

- Robot configuration: `config/robots.json`.
- Mission configuration: `config/fleet_demo.json` (poses use metres and heading degrees).
- Explicit task assignment and an ordered queue for each robot.
- Independent wheel odometry, scan matching, and navigation state.
- Shared path-following logic with the single-robot CLI.
- Static-map and live LiDAR obstacles, plus other robots' estimated footprints.
- Predicted separation checks that stop both robots if their commands conflict.
- Task/event logs, timeouts, no-progress stopping, and all-robot shutdown.

Traffic reservations, right-of-way arbitration, automatic task allocation,
collision guarantees, and recovery from deadlock are **not** implemented yet.
A conflict may cause both robots to wait. After 30 seconds without position
progress, the stalled robot is parked and its unfinished queue is offered to a
reachable replacement, with bounded retries. If recovery is unavailable, tasks
fail explicitly. Sensor/localization failures still stop the entire fleet.

## 1. Prepare

Open `scenes/environment.ttt` containing both robots. Stop simulation. Ensure both
robots have working scanners (`lidar` in their own hierarchy), wheel targets are
zero, and no other controller or robot script is driving them. Hide sensor lines
in each sensor script if desired; keep measurement generation enabled.

From the project directory, activate your Python environment. The included
mission expects these initial poses:

- Robot A: `[-1.075, -0.875, 0]`.
- Robot B: `[0.925, -0.650, 0]`.

Confirm them with `python3 inspect_robot.py` and
`python3 inspect_robot.py --robot Robot_B`. Edit `initial_pose` in the mission if
they differ. The live runner optionally reads actual simulator poses once at
initialization; after that it uses wheels and LiDAR, not simulator pose feedback.

The supplied tasks send Robot A to `(0, 5)` and Robot B to `(4.864, 5.128)`.
These are distinct goals that pass the static-map planner. Live robot positions
can change the route. The first live fleet trial completed both tasks (user verified 2026-10-15): Robot A at 49.4 s and Robot B at 108.0 s. Robot B waited 24.5 simulation seconds before resuming. This validates the demo, not arbitrary crossing scenarios or deadlock recovery.

## 2. Preview both routes

```bash
cd ~/minor
python3 run_fleet.py --plan-only
```

Open:

- `results/fleet/Robot_A_task_1.png`
- `results/fleet/Robot_B_task_1.png`

These previews use mission initial poses and the static map only. They are not
reserved trajectories and do not prove the robots can traverse them together.

## 3. Run a watched trial

Keep simulation stopped and run:

```bash
python3 run_fleet.py --initialize-from-sim
```

This explicitly reads both starting positions once, starts simulation, and
executes the task queues. No need to paste individual goal commands. To use only
mission initial poses instead, omit `--initialize-from-sim` after verifying them.

Watch both robots. Ctrl+C stops the fleet. A robot sensor/localization failure
stops the entire fleet because peers can no longer safely rely on its position.
Only after all tasks finish does it print `All fleet tasks completed`.

## 4. Check outcomes

- `results/fleet/events.jsonl`: task transitions and reasons with simulation time.
- `results/fleet/tasks.json`: final task states.

Statuses are `pending`, `running`, `waiting`, `completed`, `failed`, or `cancelled`.
When the run aborts, completed tasks retain their result and unfinished tasks get
the failure or cancellation reason. Outputs are replaced by each new run; use a
new `--output results/fleet_trial_2` directory to preserve previous results.

If it stops, share the terminal output and `tasks.json`. Do not launch independent
stepping controllers to force a blocked robot through. The next milestone is
traffic coordination, including partition-passage reservations and bounded
recovery.

## Add tasks and robots later

Add tasks to the mission's `tasks` array with a unique ID, configured robot name,
and `[x, y]` goal. Tasks for one robot execute in array order. Different robots
must have separated goal locations in this prototype. Completed or idle robots
remain stationary obstacles while their peers continue.

To add a robot, prepare its scene model/scanner, add its geometry and paths to
`config/robots.json`, then add its initial pose and tasks to a new mission JSON.
Launch with `--mission config/your_mission.json`. First validate two robots, then
traffic coordination, before trying five or ten. Localization and obstacle
handling depend on sufficient scan coverage and correct wheel geometry.

## Offline validation

```bash
python3 -m unittest discover -s tests -v
```

Fleet tests exercise predicted conflict stopping, obstacle footprints, task
validation, sequential queues, shared stepping, and all-motor shutdown on sensor
failure using a fake simulator. Static previews were generated against the
latest `maps/mapping_fixed` map. These checks do not replace live fleet trials.

## Understand waiting reasons

The runner now prints a specific control reason whenever status or reason changes.
The same text is saved in `events.jsonl` and the current task's `reason` field:

- `predicted separation from Robot_A below 0.95 m`: the separation checker stopped this robot's proposed motion.
- `robot footprint blocks route: ...`: a route exists without the other robot footprint layer, but not with it. Names list the candidate peers, not a uniquely identified blocker when several peers exist.
- `live LiDAR obstacle blocks route ...`: recent measured obstacles prevent a route even without the robot footprint layer. The readings may include another robot; the sensor does not identify object names. Hits persist up to five simulation seconds.
- `static map/unknown-space clearance: ...`: no valid route exists even before live obstacles and peer footprints are added.
- `LiDAR obstacle too close: full safety stop`: immediate measured clearance is below the configured limit.
- `LiDAR obstacle ahead ...`: forward motion is stopped; turning may still be allowed.
- `predicted turning motion enters map safety margin`: the short predicted arc is blocked; turning in place may continue.
- `turning to align with path`: a normal running state, not necessarily a stationary wait.

These reasons identify the actual software check. They do not retroactively
explain old combined messages or prove the physical identity of a LiDAR return.
For a new diagnostic trial, preserve previous logs with a fresh output folder:

```bash
python3 run_fleet.py --initialize-from-sim --output results/fleet_wait_debug
```

Share the `waiting` lines and their later `running` transitions, or the complete
`results/fleet_wait_debug/events.jsonl`, when investigating a delay.

Bounded stalled-task replacement is now implemented; see the closed-loop feedback section in [TASK_ALLOCATION.md](TASK_ALLOCATION.md). Traffic arbitration and reservations remain future work.

### Recovering from the extra planning margin

The October 15 loading-dock run left Robot B at estimated position (0.514, 3.020), in free space with about 0.585 m clearance. That satisfies the 0.40 m base clearance but fails the 0.60 m planning clearance. Previously, every subsequent route rejected this start until the no-progress timer reassigned the task. The saved log does not establish whether tracking error or a localization correction put it there.

The fleet now attempts a bounded escape from this extra margin: at most 0.30 m from the current estimated position, at 0.04 m/s, only with more than 0.10 m clearance beyond the base requirement. The escape path must preserve at least its starting grid clearance from all static, live, and peer obstacles, and end outside the full planning margin. LiDAR stopping and predicted peer separation remain active. Otherwise the robot continues to stop, with normal bounded reassignment. The log reports `recovering extra planning margin` and records estimated poses and scan scores on control-status changes.

Regression checks replay the recorded position and verify the escape bounds, occupied-start rejection, and live-obstacle rejection. Live simulator verification remains necessary. Exit and restart the persistent controller to load this change; a robot already marked unavailable is not automatically re-enabled within the old session.
