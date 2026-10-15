# Multi-robot navigation project tracker

Last updated: 2026-10-15

Goal: use natural-language instructions to assign and execute tasks with 5–10
robots in CoppeliaSim, using a shared map and coordinated navigation.

Checked items are completed. Unchecked items still need implementation or the
specified validation. Update this file as work finishes; a feature is complete
only after its acceptance checks pass. This is a manual tracker, not a background
monitor.

## 1. Mapping and single-robot navigation — completed baseline

- [x] Attach working fastHokuyo LiDAR to Robot A.
- [x] Generate a 2D occupancy map from LiDAR and simulator poses.
- [x] Correct maximum-range readings that produced false obstacle arcs.
- [x] Add keyboard driving and resuming an existing map.
- [x] Scan the previously missing area; latest map is `maps/mapping_fixed/map.yaml` and its referenced PGM.
- [x] Add initial-pose inspection and click-to-select destinations.
- [x] Make goal selection and navigation default to `mapping_fixed`.
- [x] Plan paths with obstacle inflation and blocked unknown areas.
- [x] Estimate position using wheel odometry and local LiDAR scan matching during navigation.
- [x] Add live obstacle checks, temporary-obstacle replanning, and failure stopping.
- [x] Add extra path clearance and predicted-motion checks near corners.
- [x] Pass offline regression checks for mapping and navigation.
- [x] Reach the first goal across the partition in simulation (user verified).
- [x] Reach a second selected goal in simulation (user verified).

The completed map was built using simulator poses. Navigation uses an initial
pose followed by wheel odometry and scan matching. These successes do not prove
global localization, full SLAM, or multi-robot collision avoidance.

## 2. Reusable robot controller — next milestone

- [ ] Extract simulator connection, scan reading, odometry, localization, planning, and motor control into reusable components.
- [x] Replace hardcoded Robot A paths in navigation and pose inspection with a configurable robot name.
- [x] Define robot configuration for names, model/motor/LiDAR paths, wheel dimensions, and clearance (`config/robots.json`); initial poses remain explicit per-run CLI inputs.
- [x] Remove hardcoded Robot B motor stopping from the configurable navigator.
- [x] Give every robot independent localization, progress tracking, and navigation state in the fleet runner.
- [x] Retain the existing single-robot command and reuse its path follower in the fleet runner (offline regression passed; live regression pending).
- [ ] Verify Robot A still reaches the existing test goal after refactoring.
- [x] Verify Robot B can independently navigate using the same map (user verified).

Acceptance: the same navigation implementation works for either robot through
configuration, without editing its source.

## 3. Central fleet loop and task queue

- [x] Create one stepping loop that reads and commands all robots before advancing simulation once (`run_fleet.py`, offline validated).
- [x] Prepare two scene robots with distinct names and working LiDARs (user verified individually); LiDAR display lines hidden.
- [x] Add per-robot active/idle navigation state and task states pending/running/waiting/completed/failed/cancelled.
- [x] Define tasks with unique IDs, destinations, explicit assigned robot, status, and failure reason. Unassigned automatic allocation remains section 5.
- [x] Add a queue and explicit per-robot task dispatch.
- [x] Record assigned tasks, starts, waits, failures, and completions in a fleet event log.
- [x] Handle Ctrl+C by attempting to stop every robot and simulation (live verification pending).
- [x] Handle one robot's localization/sensor failure by stopping the entire fleet and recording unfinished tasks as failed (offline tested).
- [x] Verify both robots reach separate goals in one concurrent fleet run, with Robot B waiting during a conflict (user verified 2026-10-15; task logs confirm completion).

Acceptance: a single process controls both robots, records task completion, and
stops them predictably on shutdown.

## 4. Traffic coordination

- [ ] Include other robots as obstacles when planning and checking immediate motion.
- [ ] Define minimum separation using both robot footprints plus margin.
- [ ] Identify narrow passages, including the route around the partition tip.
- [ ] Reserve narrow passages for one robot at a time; define safe waiting positions.
- [ ] Release reservations on completion, timeout, cancellation, or robot failure.
- [ ] Add deterministic priority and starvation prevention.
- [ ] Detect stalled tasks and stop safely when recovery is unavailable.
- [ ] Test opposite-direction passage crossing.
- [ ] Test two robots requesting the same destination.
- [ ] Test a stopped robot blocking a passage.
- [ ] Test intersecting paths and a temporary obstacle.

Acceptance: these scenarios finish or report a clear blocked result, without
collisions, overlapping goals, or indefinite waits.

## 5. Task assignment and natural-language commands

- [ ] Define named stations and verify their coordinates are reachable in the map.
- [ ] Support ordered multi-stop tasks such as visit, inspect, and patrol.
- [ ] Assign unallocated tasks to eligible idle robots using planned path length.
- [ ] Preserve explicit robot assignments in user instructions.
- [ ] Support queueing, cancellation, and bounded retries without duplicate execution.
- [ ] Connect the existing language planner in `robot_navigation/legacy_tasks.py` to the fleet task schema.
- [ ] Validate robot names, station names, actions, and reachability before dispatch.
- [ ] Replace old hardcoded wall-bypass execution with map-based fleet execution.
- [ ] Report each task's actual outcome; do not report failed tasks as completed.
- [ ] Test ambiguous instructions, unknown destinations, and unreachable goals.

Acceptance: an instruction such as “Send Robot A to charging and Robot B to
loading” produces validated tasks and accurate completion/failure reports.

## 6. Scale from two robots to five, then ten

- [ ] Add Robot C–E with validated geometry, LiDAR, and unique configuration entries.
- [ ] Verify all five robots can localize from their supplied initial poses.
- [ ] Run a five-robot mixed-task scenario with traffic coordination.
- [ ] Measure simulation speed, update latency, task durations, waits, and failures.
- [ ] Optimize scan sampling and localization/replanning rates based on measurements.
- [ ] Add Robot F–J and repeat individual sensor/localization checks.
- [ ] Run a ten-robot mixed-task scenario and a congested-passage scenario.
- [ ] Verify no collision or indefinite wait in the defined validation scenarios.

Acceptance: repeatable five- and ten-robot runs with recorded results and explicit
limits on performance and congestion. More robots alone do not prove scalability.

## 7. SLAM validation — separate from saved-map fleet navigation

- [x] Implement an experimental local scan-matching map-extension mode (`--slam`).
- [ ] Validate map extension in simulation; implementation alone is not completion.
- [ ] Evaluate drift and localization errors against simulator ground truth used only for evaluation.
- [ ] Decide whether the project requires global loop closure and relocalization.
- [ ] If required, integrate a tested SLAM/localization stack and its ROS bridge.
- [ ] Keep experimental SLAM isolated from the validated fleet-navigation baseline until tested.

## 8. Final demonstration and delivery

- [ ] Document scene setup, dependencies, robot configuration, and launch commands.
- [ ] Add repeatable fleet tests for assignment, reservations, failures, and shutdown.
- [ ] Save demonstration instructions and their expected task outcomes.
- [ ] Run the full demonstration from a stopped scene without manual code changes.
- [ ] Capture logs and a demo video for the completed fleet scenario.
- [ ] Document remaining limitations and distinguish simulator-pose mapping, localization, and SLAM.

## Immediate next work

Repository organization is complete: implementation is in `robot_navigation/`,
offline tests in `tests/`, scene in `scenes/`, documentation in `docs/`, and the
latest unchanged scan is tracked under `maps/mapping_fixed/`. Root scripts remain
available as compatibility entry points. This organization does not complete the
fleet-controller refactoring in section 2.

Next implement and test traffic coordination: specific wait reasons, priority/yielding, and narrow-passage reservations. The first concurrent two-robot mission completed successfully; crossing and deadlock scenarios remain unvalidated.
Do not duplicate ten independent stepping scripts; all robots will share the
central loop introduced in section 3.

Configuration milestone: nine offline tests pass, including robot-specific handle resolution and wheel commands. Both robot selections pass planning with a synthetic shared test start; this does not verify Robot B's actual scene pose, sensors, or live navigation. Reusable motion/scene access is in `robot_navigation/robot.py`; the navigation loop still needs extraction for the central fleet runner.

Fleet prototype implemented: see [FLEET_GUIDE.md](FLEET_GUIDE.md). Fourteen offline tests pass. Both demo routes pass static planning. Concurrent goal completion passed the first watched simulation trial. General shared-path safety, passage reservations, and deadlock recovery remain unchecked.

### First live fleet result — 2026-10-15

Both tasks completed in the shared simulation loop. Robot A completed at 49.4 s; Robot B completed at 108.0 s. Robot B waited from 19.3 s to 43.8 s (24.5 simulation seconds), then resumed. The recorded reason combines path blockage and separation, so the exact trigger cannot be distinguished from that log alone. This verifies the demo mission, not general collision avoidance or deadlock recovery. Next improve traffic coordination before scaling robot count or increasing speed.

Waiting diagnostics implemented: planning-layer failures, LiDAR clearance stops, motion-margin checks, and predicted peer separation now have distinct reasons. Reason changes are logged even if status stays waiting. Fifteen offline tests pass; validate the messages in the next live run. This does not add priority arbitration or reservations.

## Priority update: task allocation is the main project objective

Natural-language interpretation, grounded task decomposition, and robot selection
using map-based travel distance take priority. Avoidance and traffic coordination
are supporting execution components. New workflow: `allocate_tasks.py` -> validated
intent -> greedy queued-distance allocator -> generated mission -> central fleet.

- [x] Implement structured Gemini intent extraction with explicit robot preservation (SDK configuration validated; live API test pending).
- [x] Ground station names in `config/stations.json` and reject unknown/unreachable tasks.
- [x] Compute A* route distance per candidate robot instead of Euclidean distance.
- [x] Include already allocated route distance when selecting robots for additional jobs.
- [x] Record candidate distances and allocation reasons in `allocation.json`.
- [x] Preserve ordered visits on one robot and generate executor-compatible tasks.
- [x] Verify offline allocation and route previews on the current map; nineteen tests pass.
- [x] Verify live LLM interpretation with Gemini (gemini-2.5-flash, 2026-10-15; structured station visits validated).
- [ ] Execute an automatically allocated natural-language mission and verify outcomes.
- [ ] Evaluate greedy assignment against a global assignment baseline and traffic delays.
- [ ] Add online arrival of tasks, capabilities, and reassignment after execution failure.

Debug change: peer returns are no longer cached as five-second generic obstacles.
Raw sensor stops and peer footprint/separation checks remain active; live effect
on wait time is unverified. See [TASK_ALLOCATION.md](TASK_ALLOCATION.md).

### Closed-loop replacement milestone

- [x] Monitor lack of position progress and record failed attempts.
- [x] Park the stalled robot and transfer its unfinished queue in order.
- [x] Select a reachable replacement using current route plus queued distance.
- [x] Bound retries and retain task IDs, original assignment, and attempt history.
- [x] Verify replacement completion and retry exhaustion offline (21 tests pass).
- [x] Verify the live `--simulate-stall Robot_A` recovery demonstration (user confirmed; saved tasks show Robot B completed the transferred task).
- [ ] Improve progress detection beyond displacement and distinguish planned waiting from hardware stalls.

Sensor/localization loss remains a fleet-wide fail-stop; online task arrival,
capability matching, and global replanning remain future work.

Research alignment documented in [RESEARCH_DESIGN.md](RESEARCH_DESIGN.md): SMART-LLM structured planning and COHERENT-inspired execution feedback, with local distance-based allocation and bounded recovery. Routine execution and recovery make no LLM calls. New allocation runs record application invocations and available token usage; monetary cost comparisons remain unmeasured.

First live natural-language interpretation succeeded: results/llm_first_trial contains intent, allocation, mission, usage, and static previews. Captured current poses assigned charging to Robot A (5.91 m versus Robot B 6.05 m), then loading to Robot B (8.93 m versus Robot A projected 10.69 m). Usage: one application interpretation invocation, 399 total provider-reported tokens. Executing this saved mission needs no new LLM call; live mission execution remains pending. Automatic function calling is explicitly disabled for subsequent interpretation requests.

### Persistent natural-language session

- [x] Add `run_session.py`: consecutive instruction batches in one simulation lifecycle.
- [x] Retain estimated poses, failed-robot availability, and task history between batches.
- [x] Reject malformed requests without ending the session; support offline intent replay.
- [x] Test two batches, retained poses, API/offline behavior, unique IDs, and fault shutdown (24 tests pass).
- [ ] Rehearse two consecutive batches live in CoppeliaSim before review; follow `docs/REVIEW_DEMO.md`.
- [ ] Future: accept/queue instructions during execution, station reservations, larger-fleet evaluation.

### Points of interest and shared destinations

- [x] Explicit destination synonyms normalized before LLM interpretation.
- [x] Named arrival slots with route-distance selection and separated goal reservations.
- [x] Current occupied slots excluded across session batches.
- [x] Per-robot arrival evidence in task/event logs.
- [x] Static routes checked for two robots at each configured destination; 27 automated tests pass.
- [ ] Live two-robot loading-dock trial and visual confirmation of extra slot placement.
- [ ] Capacity waiting queues and temporal station reservations.

### Robot B planning-margin stop

- [x] Diagnose saved loading-dock session: estimated clearance 0.585 m versus planning requirement 0.60 m.
- [x] Add slow, bounded escape from extra planning margin, retaining base clearance and live safety checks.
- [x] Record estimated pose/scan score on control status changes; 30 automated tests pass.
- [ ] Repeat both-robots loading-dock scenario live and confirm separate arrivals without reassignment.

### Five-robot expansion

- [x] Set fleet/session default cruise to 0.18 m/s while preserving wheel and safety limits.
- [x] Configure C/D/E and a short five-robot simultaneous mission.
- [x] Add model-copy setup with scene backup and separate five-robot scene output.
- [x] Add five loading arrival slots in a separate station profile.
- [x] Distinguish known traffic waits from no-progress failure; bound traffic deadlock wait.
- [x] Test actual controller with five concurrent kinematic robots and compare speeds.
- [ ] Run setup/live smoke test: local simulator connection timed out during development.
- [ ] Validate five-robot shared-destination navigation, recovery and wall-clock performance.
