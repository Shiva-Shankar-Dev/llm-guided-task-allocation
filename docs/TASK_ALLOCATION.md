# Main workflow: natural language and distance-based task allocation

The main project goal is allocating and executing robot tasks from natural
language. Mapping, localization, and obstacle avoidance support that goal.
The new entry point is `allocate_tasks.py`; `main.py` remains the old demo.

## How it works

1. Gemini translates the instruction into station visits and any explicitly
   requested robot. Unspecified robots remain null; the model does not guess
   coordinates or invent motor commands.
2. Python validates every station, robot name, unique job ID, and visit sequence.
3. A* measures traversable route length on the saved map, including detours around
   walls. Straight-line distance is not used for allocation.
4. The allocator picks the robot with the least **queued route distance + new job
   route distance**. Explicit robot requests are honored if feasible. Robot names
   break ties deterministically.
5. A generated mission is passed to the existing central fleet executor.

This is a deterministic greedy allocation baseline, not a globally optimal
assignment solver. Explicit jobs are allocated first; unspecified jobs follow
input order. Within each job, ordered station visits stay with the same robot.
Queued distance assumes equal robot speeds and is not a prediction of traffic
wait time. It supports planning a batch while stopped, not adding jobs to an
already moving fleet. Reassignment, capabilities, batteries, station occupancy,
priorities, and richer actions remain future work.

## Station grounding

`config/stations.json` contains navigation destinations:

- `charging_station`: (0, 5)
- `loading_zone`: (4.5, 4.5)
- `left_corridor`: (5, 0)

These reuse the original project's landmark coordinates and pass planning on the
current map. Verify they correspond to your intended physical zones before the
live demonstration. Visiting a station does not perform actual charging, loading,
or object manipulation. Update coordinates here rather than letting the LLM
invent them.

## 1. Try allocation without a model or API key

```bash
cd ~/minor
python3 allocate_tasks.py --intent-file config/example_intent.json
```

This is an offline structured-intent test, **not** a natural-language LLM test.
It prints each candidate's route metres, queued metres, and selection reason.
With the supplied initial poses, the example assigns charging to Robot B, then
loading to Robot A because Robot B already has queued travel. Results:

- `results/allocation/intent.json`: interpreted request.
- `results/allocation/allocation.json`: candidate costs and allocation explanation.
- `results/allocation/mission.json`: executable fleet mission.

Nothing moves by default. These default pose values come from
`config/fleet_demo.json`; refresh them if your robots moved.

## 2. Use natural-language input

Install current dependencies (`python3 -m pip install -r requirements.txt`).
Configure `GOOGLE_API_KEY` or `GEMINI_API_KEY` in the terminal environment and set
`GEMINI_MODEL` to a model available to your Gemini account. You can alternatively
pass the exact model ID using `--model`. No model is silently selected or swapped.
Do not commit real API keys. These tools do not automatically load `.env` files.

For example, after those environment variables are configured:

```bash
python3 allocate_tasks.py \
  --instruction "Send robots to the charging station and the loading zone. Choose the robots based on travel distance." \
  --capture-poses
```

Keep CoppeliaSim open and stopped. `--capture-poses` reads robot positions once
for allocation and localization initialization. It does not move the robots.
All robots listed in the pose file are considered available and idle initially;
additional jobs in the same batch accumulate queued route distance.

Other supported examples:

- “Send Robot B to the loading zone.” (preserves the explicit robot)
- “Have a robot visit charging station, then left corridor.” (same robot, ordered visits)
- “Patrol charging station and loading zone once.” (bounded ordered visits)

Ambiguous instructions or unsupported dependencies/actions produce a clarification
error and do not execute. Unknown stations/robots and unreachable jobs are
rejected. The LLM can still misinterpret an instruction even when its JSON is
valid, so review the saved intent and allocation before initial live trials.

Provider integration uses the Google GenAI SDK's structured JSON output:
https://ai.google.dev/gemini-api/docs/generate-content/structured-output
No live model call was made during implementation; model/API compatibility must
be verified with your configured account.

## 3. Preview and execute

Preview the allocated mission:

```bash
python3 run_fleet.py --mission results/allocation/mission.json \
  --plan-only --output results/allocation/preview
```

After reviewing the paths, leave the robots at the captured starting poses and
execute the mission:

```bash
python3 run_fleet.py --mission results/allocation/mission.json \
  --output results/allocation/execution
```

If the robots have moved, recapture poses and reallocate; otherwise candidate
costs are stale. Do not run another controller at the same time. Ctrl+C stops
the fleet. The executor records task results, so allocation alone is not proof
of successful execution.

Once the workflow is validated, `allocate_tasks.py ... --execute` performs the
interpretation, validation, allocation, and execution in one command. Use
`--capture-poses` when executing from current simulator positions. Save distinct
`--output` directories for comparison runs.

The prototype prevents overlapping goal ownership across different robots,
because it does not yet schedule shared station occupancy. Repeated visits to
one station are assigned to its existing owner unless explicitly incompatible,
in which case the request is rejected.

## Debug-run change

The original diagnostic run completed both tasks. Robot B waited from 19.25 to
43.85 simulation seconds. The source first identified Robot A's planning footprint
and later the cached live LiDAR layer. Recognized peer returns are now excluded
from the five-second generic obstacle cache; peers still have their own moving
footprints and raw LiDAR safety checks. This should reduce stale robot obstacles,
but the effect on live waiting duration needs another trial. It does not remove
separation checks or implement traffic arbitration.

## Validation

```bash
python3 -m unittest discover -s tests -v
```

Nineteen offline tests pass, including an example where Euclidean-nearest Robot A
has a longer wall-detour route than Robot B, explicit assignment, queued work,
ordered visits, rejected intent, and exclusion of known peer returns from the
obstacle cache. The example allocation compiles into a fleet mission and both
routes pass the actual map planner. Live LLM parsing and allocated-mission
execution remain to be tested.

## Closed-loop feedback and bounded replacement

The executor now monitors position progress. After `--stall-timeout` seconds
(default 30) without at least 0.1 m of position progress, it:

1. Records an `attempt_failed` event and parks the stalled robot for this run.
2. Considers its active task followed by pending tasks as an ordered bundle.
3. Evaluates each remaining available robot using current mapped routes, live
   obstacle/peer layers, and its active/pending queue distance.
4. Transfers the bundle to the reachable candidate with the smallest projected
   queued route distance. It records a `pending` event explaining the reassignment.
5. Executes the transferred tasks after that candidate's existing commitments.

Tasks retain their IDs. `original_robot`, `attempt_history`, `reassignments`, and
`replacement_candidates_m` in `tasks.json` make the decision auditable. Completed
tasks are not rerun. Explicit initial assignments can be overridden on failure,
as required by the replacement policy. The parked robot remains an obstacle,
continues sensing/localization, and receives no further tasks during this run.

`--max-reassignments` defaults to 1 per task, preventing endless handoffs. Set 0
to disable replacement. If no candidate can safely reach the remaining queue, or
the limit is exhausted, those tasks become `failed` while other work continues.
A bundle is not partially split; preserving ordered work takes priority over
finding a different robot for every step. Tasks may wait behind existing work on
the replacement. This is not global optimal replanning or capability matching.

A robot's sensor/localization failure still stops the entire fleet: its position
is no longer sufficiently trustworthy to let peers move around it. The new
recovery handles blocked or nonmoving robots **with valid position tracking**.
Long traffic waits can also trigger this simple no-progress policy; increase
`--stall-timeout` for crowded scenarios rather than treating every wait as a
hardware failure. Oscillation that moves more than the progress threshold can
still evade this detector; richer task progress tracking remains future work.

### Demonstrate replacement without damaging or moving scene objects

With both robots in the stopped scene, run:

```bash
cd ~/minor
python3 run_fleet.py --initialize-from-sim \
  --simulate-stall Robot_A --stall-timeout 10 \
  --output results/fleet_recovery_demo
```

The demo deliberately commands Robot A to remain stationary. Robot B receives
its own task, then can take Robot A's remaining task when the timeout expires and
a reachable replacement route is found. Watch the first run; Ctrl+C stops all
robots. Remove `--simulate-stall` for normal operation.

Look for `attempt_failed`, then `pending ... reassigned from Robot_A to Robot_B`,
then `running` and `completed`. If no reachable replacement exists, the log
reports failure instead of claiming completion. Outputs are in
`results/fleet_recovery_demo/events.jsonl` and `tasks.json`.

Twenty-one offline tests pass, including recovery completion through the central
loop, nearest-route replacement, task order preservation, retry bounds, and
all-robot shutdown. Live recovery timing and completion still need validation.

Research positioning and API accounting are in [RESEARCH_DESIGN.md](RESEARCH_DESIGN.md). Allocation outputs now also include `llm_usage.json` with application-level interpretation invocations and provider token metadata when available. The user has verified the simulated-stall recovery trial; live LLM interpretation remains pending.

### Destination aliases and shared arrival points

`config/stations.json` now defines each place with `aliases` and named `slots`. For example, loading dock/bay/area resolve deterministically to `loading_zone` before interpretation. Unknown names are still rejected or clarified; having only one known destination does not justify guessing every unfamiliar name.

“Send both robots to the loading dock” is interpreted as one explicitly assigned job per robot. Allocation chooses reachable slots by route distance, reserving different slots for different robots. Goals are separated by robot clearances, the normal separation margin, and both robots' 0.15 m arrival tolerances. Current peer positions also exclude occupied slots in persistent sessions. Capacity exhaustion rejects the instruction; automatic waiting queues for occupied slots are not implemented. Reservations are conservative for the whole batch, including intermediate visits.

The provided two slots per place pass static route checks against `maps/mapping_fixed/map.yaml`. These are navigation arrival points, not verified charging contacts or cargo-handling poses. Confirm that the proposed additional positions lie within the intended physical area in CoppeliaSim. Old custom station files with a single `[x,y]` remain supported as one-slot places.

Each task retains `station`, `slot`, and `goal`. On completion, `tasks.json` and `events.jsonl` record the estimated arrival pose, distance to slot, tolerance, and scan score. A robot completes its visit within 0.15 m of its assigned point. To confirm both visits, inspect each robot's completion record at the same station with different slots. This is a localization-based arrival check, not an independent physical docking sensor. If failure recovery transfers a task, the actual replacement robot is recorded; this does not prove the failed robot arrived.
