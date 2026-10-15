# Persistent fleet review demonstration

Start CoppeliaSim with `scenes/environment.ttt`, with Robot_A and Robot_B and their LiDAR sensors. Keep the simulation **stopped** and close other robot controllers. Keep your API key in the environment; never show it in screenshots.

From the project directory:

```bash
cd ~/minor
python3 run_session.py --model gemini-2.5-flash
```

The session reads actual starting poses once. Subsequent navigation and allocation use wheel/LiDAR pose estimates. The authoritative map is `maps/mapping_fixed/map.yaml`. The default mission configuration selects Robot_A and Robot_B; its old tasks are ignored in session mode.

At `fleet>` type:

```text
Send robots to the charging_station and loading_zone. Choose robots based on travel distance.
```

Tasks start after validation. Wait for both to finish. The session zeros motor commands and holds simulation time at their current positions, then shows `fleet>` again. **Do not press Stop in CoppeliaSim.**

Check the positions and available named destinations:

```text
/status
/stations
```

Then enter another instruction, for example:

```text
Send Robot_A to left_corridor, then charging_station.
```

This is an ordered visit sequence for A; B stays parked. Allocation for this batch starts from the latest estimated poses, not the original starting coordinates. Use the names printed by `/stations`; confirm their configured coordinates represent your intended scene locations before presenting.

Natural-language requests execute automatically after validation. There is one application-level interpretation request per new natural-language submission, including a rejected submission if sent to the provider. SDK/network retries are not counted. There are no LLM calls for motion, waiting, or failure reassignment.

For a repeatable offline demonstration, reuse validated structured intent:

```text
/intent config/example_intent.json
```

This still reallocates from current poses and uses zero LLM requests. Invalid inputs print an explanation and return to the prompt. Parked failed robots remain unavailable for the rest of the session. A destination occupied by a robot with no departure task is rejected; move that robot first.

To end:

```text
/quit
```

Ctrl+C also ends the session. Ending deliberately stops the simulation, so CoppeliaSim restores the original scene positions. Keeping positions across completely separate program sessions is not implemented.

## Evidence and rehearsal checklist

Each run creates a fresh `results/sessions/<timestamp>/` folder:

- `events.jsonl` and `tasks.json`: execution, waiting reasons, completion, and reassignment across all batches.
- `session_state.json`: latest positions and availability when awaiting input (poses in metres/radians).
- `batch_001/`, etc.: structured intent, allocated mission, candidate distance costs, API usage metadata, and errors if rejected. Mission headings are degrees.

Rehearse these before the review:

- [ ] Complete the first natural-language batch in CoppeliaSim.
- [ ] Observe that the simulation remains active and positions are retained at the next prompt.
- [ ] Complete a second batch starting from those positions.
- [ ] Try an unknown destination; confirm no motion and the prompt returns.
- [ ] Inspect the two batch mission files and usage logs.
- [ ] Finish with `/quit` and confirm every robot stops.

Automated validation: 24 tests pass, including successive batches in one simulator lifecycle, updated poses, rejected input, offline input, unique task IDs, and sensing-error shutdown. The new persistent-session flow still needs the live rehearsal above; automated tests do not establish collision-free performance in every scene.

## Current scope

New instructions are accepted **between completed batches**, not while robots are executing. Simulation time is held during input and API requests. Each batch has a 300 simulation-second timeout (override with `--duration`). Normal bounded no-progress reassignment remains active; it does not call the LLM. A sensing/localization fault or batch timeout stops the session for inspection. Shared-station scheduling and guaranteed deadlock resolution are not implemented. Do not present this two-robot prototype as already validated for 5–10 robots.

## Shared-destination demonstration

Restart the controller to load the new station configuration. Enter:

```text
Send both robots to the loading dock.
```

Loading dock is a configured alias of loading_zone. Each robot should receive a different arrival slot. Check both completed task records for `station: loading_zone`, distinct `slot` values, and `arrival.distance_to_slot_m` below 0.15. Then use `/status` to inspect the two positions. This new scenario still requires a live trial; the two-slot geometry and routes have been checked offline.
