# Input configuration

- `robots.json`: robot scene paths, motors, LiDAR, wheel geometry, and clearance.
- `stations.json`: named places, aliases, and separate arrival slots in world metres.
- `fleet_demo.json`: robot membership/initial poses and example tasks for the one-shot fleet. Persistent sessions use its membership and capture starting poses from the scene; they ignore its example tasks.
- `example_intent.json`: structured tasks for an offline allocation/session trial.

These files are inputs. Generated missions and task outcomes belong in `results/`. Read `../docs/WORKFLOW.md` before editing coordinates or running a controller.
