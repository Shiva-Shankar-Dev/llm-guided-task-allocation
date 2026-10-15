# Five robots and faster default travel

The fleet/session default cruising speed is now **0.18 m/s**. `--speed 0.12` restores the old speed. Wheel caps remain 2 rad/s (about 0.195 m/s straight motion for the configured wheels), and turns/obstacle checks can reduce speed. Margin recovery remains at most 0.04 m/s.

## Create the five-robot scene once

Exit any running Python controller. Open the original `scenes/environment.ttt` with A/B in CoppeliaSim and stop simulation. Then:

```bash
cd ~/minor
python3 setup_five_robots.py --apply
```

This copies Robot A, including its model and sensor scripts, to create C/D/E. It places all five at map-checked separated starting positions, checks their motors and sensors, and saves **`scenes/environment_five.ttt`**. Your original scene file is not overwritten. A pre-change scene snapshot is saved in `results/five_robot_setup/<timestamp>/before_setup.ttt`. The setup rejects an existing output scene; for subsequent runs just open the saved five-robot scene. If interrupted during setup, reopen the original scene instead of saving the partial setup over it.

Without `--apply`, the setup command only checks map/configuration and prints proposed positions; it does not connect to the simulator. Setup does not verify physics or sensor accuracy. Visually check that all five models and scanners are correctly placed.

## First run: short simultaneous motion

With the five-robot scene open and stopped:

```bash
python3 run_fleet.py --mission config/fleet_five.json --initialize-from-sim --output results/five_robot_smoke
```

Each robot is assigned a short 0.6 m forward route. Confirm all five move and complete. Task/event evidence is saved in the specified output folder. This intentionally tests concurrent control before testing congestion around the partition. The one-shot run stops simulation when complete.

## Persistent five-robot instructions

```bash
python3 main.py --model gemini-2.5-flash --mission config/fleet_five.json --stations config/stations_five.json
```

The two-robot scene/session remains available using the ordinary `python3 main.py --model gemini-2.5-flash` command with the original scene. Registering C/D/E in `robots.json` alone does not select them: fleet membership comes from the `--mission` file.

For a shared-destination trial:

```text
Send all five robots to the loading dock.
```

The five-robot station file has five loading slots. Charging and corridor destinations still have two slots each; requests exceeding capacity can be rejected. Additional loading coordinates are checked against the saved map, but confirm their placement within your intended physical loading area in the scene. All five jobs can be active together; traffic checks may temporarily stop individual robots.

## Traffic waits and verification limits

Known peer-footprint/separation waits no longer mark a robot as faulty after the ordinary no-progress timeout. A continuous known traffic wait beyond `--traffic-timeout` (default 90 simulation seconds) stops the run for inspection. This is bounded deadlock detection, not a passage scheduler: crowded routes can still deadlock. Unrecognized LiDAR blockage can still trigger normal reassignment. Recovery with five robots still needs live validation.

A kinematic integration test runs the actual path follower, planner, wheel conversion, separation checks, and all five task queues. It substitutes perfect pose observations and clear live scans, so it does not validate CoppeliaSim physics or localization. The short trial took 6.0 simulated seconds at 0.12 m/s and 4.8 seconds at 0.18 m/s. Other tests check five distinct loading-slot allocations and traffic waits without robot quarantine.

CoppeliaSim connection attempts timed out during development; no live five-robot scene or performance result is claimed. First run the smoke test above, then shared destinations. The remaining risks are traffic congestion, localization/physics errors and real-time computation load from five LiDAR streams. Log timestamps are simulation time, not wall-clock runtime.
