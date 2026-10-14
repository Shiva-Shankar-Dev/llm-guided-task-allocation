# Multi-robot navigation project tracker

Last updated: 2026-10-14

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
- [ ] Replace hardcoded Robot A paths with a configurable robot name.
- [ ] Define a fleet configuration: name, model path, motor paths, LiDAR path, wheel radius, wheel separation, footprint radius, and initial pose.
- [ ] Remove code that automatically stops Robot B from the reusable controller.
- [ ] Give every robot independent localization, progress tracking, and navigation state.
- [ ] Retain the existing single-robot command as a working entry point.
- [ ] Verify Robot A still reaches the existing test goal after refactoring.
- [ ] Verify Robot B can independently navigate using the same map.

Acceptance: the same navigation implementation works for either robot through
configuration, without editing its source.

## 3. Central fleet loop and task queue

- [ ] Create one stepping loop that reads and commands all robots before advancing simulation once.
- [ ] Prepare two scene robots with distinct names and working LiDARs.
- [ ] Add robot states: idle, navigating, waiting, completed, and failed.
- [ ] Define tasks with unique IDs, destinations, optional requested robot, status, and failure reason.
- [ ] Add a queue and explicit per-robot task dispatch.
- [ ] Record assignments, starts, waits, failures, and completions in a fleet event log.
- [ ] Handle Ctrl+C by stopping every robot and simulation.
- [ ] Handle one robot's localization/sensor failure by stopping it and notifying the coordinator.
- [ ] Verify two robots reach separate goals concurrently in open space.

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

Start with section 2: make the working navigator accept Robot A or Robot B through
configuration. Then validate each separately before enabling concurrent motion.
Do not duplicate ten independent stepping scripts; all robots will share the
central loop introduced in section 3.
