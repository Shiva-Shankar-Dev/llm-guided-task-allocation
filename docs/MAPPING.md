# First map with Robot A

This is a beginner mapping demo using your existing ZMQ connection. It needs no
ROS, API key, or edits to the Hokuyo script. It reads the standard fastHokuyo
vision-sensor coordinate packets, projects actual laser measurements into a
2D occupancy grid, and uses simulator poses to align them. **It is not SLAM:**
there is no pose estimation, scan matching, or loop closure yet.

1. Install dependencies in your Python environment:
   ```bash
   pip install numpy coppeliasim-zmqremoteapi-client
   ```
2. Open your scene in CoppeliaSim. Keep the simulation stopped. Do not run
   `robot_navigation/legacy_tasks.py` or another controller at the same time.
3. Confirm the model path is `/Robot_A/fastHokuyo`. Keep its existing simulation
   and vision scripts enabled. Its scan plane should intersect the walls and
   boxes and clear the robot body. Rigid attachment alone does not verify this.
4. From the project directory, first test scans with stationary wheels:
   ```bash
   python map_environment.py --scan-only --duration 5
   ```
   It starts and stops simulation automatically. Check `results/mapping/map.pgm`
   in an image viewer: nearby walls should appear as black lines. Unknown regions
   are gray and observed free space is white. If all rays hit the robot, raise
   the scanner slightly. If obstacle lines are absent, check scanner height and
   the objects' visibility to vision sensors.
5. Run a three-minute drive:
   ```bash
   python map_environment.py --duration 180
   ```
   Watch the first run. Ctrl+C stops Robot A and saves the partial map. Robot B's
   wheel targets are set to zero. Disable any robot scripts that also control
   wheels. The scanner must cover the robot's front; the script stops with an
   error if it cannot find enough rays in the forward sectors.

The script drives slowly and turns toward the clearer side when something is
within 0.75 m. This reactive exploration can revisit the same places or get
trapped in a corner; it does not guarantee full room coverage or collision-free
motion. Run longer if needed, reposition the robot between runs, or later add
frontier exploration. Each run creates a fresh map, so use a different `--output`
directory to preserve earlier maps.

Outputs (updated every five simulation seconds and when stopping):

- `results/mapping/map.pgm`: map image, viewable with a PGM-capable image viewer.
- `results/mapping/map.yaml`: resolution and map origin for ROS map loading later.
- `results/mapping/map_data.npz`: raw grid values for Python inspection.

The default map spans 30 x 30 m around world (0,0) with 5 cm cells. For a larger
scene use `--size 50`. Change the model path with `--lidar /your/model/path`.
Robot forward is assumed to be its local +X axis, consistent with the existing
controller; use `--heading-offset 90` if your robot's forward is local +Y.

If the script reports no coordinate packet, your Hokuyo model's vision callback
differs from the standard fastHokuyo. Share that model's vision and simulation
scripts so its actual data format can be supported; do not replace them blindly.
The standard approach reads auxiliary packets via
[sim.readVisionSensor](https://manual.coppeliarobotics.com/en/sim/simReadVisionSensor.htm).

If an earlier map contains curved black bands in open space, rerun with the
updated script and a new output directory. Earlier code incorrectly treated
maximum-range rays as obstacles by testing their Z coordinate instead of their
radial distance. The saved grid cannot recover which old hits were artificial.

The scene was not run during development. Validate the stationary scan first,
then watch a short drive before attempting longer exploration.

## Finish the unscanned area with keyboard driving

Install the mapping/navigation dependencies:

```bash
pip install -r requirements-mapping.txt
```

Open the same scene and stop simulation. Run this in a regular terminal with
keyboard focus (not an IDE output panel). Do not run `robot_navigation/legacy_tasks.py` alongside it:

```bash
python map_environment.py --manual --resume maps/mapping_fixed/map_data.npz --output results/mapping_complete --duration 600
```

The existing observed cells are retained. The original `mapping_fixed` directory
is preserved. Use **W** to move forward, **A/D** to turn in place, **space** to
stop, **Q** to stop and save. Keep tapping/holding keys; motion expires after
0.3 seconds without a key repeat. There is no reverse command because the
scanner may not cover the rear. Forward motion is inhibited near obstacles.
Drive around the *free end* of the partition, then along the right-hand outer
wall into the remaining region. Screenshot orientation is not the robot's
heading: watch the robot in CoppeliaSim. Turn in open space rather than scraping
along walls. If the robot cannot fit past a wall, do not force it through.

Open `results/mapping_complete/map.pgm` to check progress. You can run the command
again with `--resume results/mapping_complete/map_data.npz` to retain that session.
Use only the same scene geometry and world coordinate frame when resuming.
This completion tool still uses simulator poses for mapping.

## Navigate using the saved PGM

`navigate_map.py` reads **map.yaml and its referenced map.pgm**, not the NPZ.
The YAML is essential: PGM pixels alone do not contain map scale or coordinates.
Unknown areas and obstacles are blocked; both are expanded by the robot radius
plus a safety margin. A* plans a path, and a slow differential-drive controller
follows it. Nearby unexpected LiDAR hits become temporary obstacles and trigger
replanning. The robot stops on missing paths, bad localization, missing scans,
insufficient clearance, timeout, or no progress. It does not reverse or attempt
aggressive recovery when trapped.

1. Stop simulation, with Robot A at its intended initial position. Read its
   starting coordinates once:
   ```bash
   python inspect_robot.py
   ```
   This prints `--initial-pose X Y YAW_DEG`. The inspection uses simulator pose
   **only to initialize** localization. Subsequent navigation reads wheel joint
   rotations and LiDAR measurements in the robot frame, not world robot poses.
   Supply these coordinates manually instead if testing without ground truth.
2. Choose a destination by clicking known white free space:
   ```bash
   python choose_goal.py --map results/mapping_complete/map.yaml
   ```
   This prints `--goal X Y`. The GUI needs a desktop and matplotlib GUI support.
   You can instead enter known world coordinates yourself. Never interpret pixel
   coordinates as metres.
3. Combine the printed arguments. First plan without moving the robot:
   ```bash
   python navigate_map.py --map results/mapping_complete/map.yaml --initial-pose X Y YAW_DEG --goal GOAL_X GOAL_Y --plan-only
   ```
   Replace **X Y YAW_DEG GOAL_X GOAL_Y** with the numbers from steps 1 and 2.
   Inspect `results/navigation/path.png`: blue=start, red=goal, green=planned path.
4. Remove `--plan-only` to execute the same command. Watch the first drive and
   use Ctrl+C to stop. A new simulation starts automatically and stops on exit.

The scene must use Robot A's local +X as its forward direction. Defaults are
Pioneer-style **wheel radius 0.0975 m**, **wheel-centre separation 0.33 m**, and
**clearance radius 0.4 m**. Measure your model and override `--wheel-radius`,
`--track`, and `--clearance` if needed. Clearance must include the entire robot
bounding radius plus margin. Wrong wheel dimensions, incorrect initial heading,
wheel slip, robot-body returns, or a changed scene can invalidate localization.
Do not lower the match threshold simply to bypass a localization error.

## Experimental local SLAM

Navigation against a fixed PGM is localization and planning. Add `--slam` to the
navigation command to also insert scans into the map **using the estimated
wheel-odometry/scan-matching poses**. This is a basic scan-to-map occupancy-grid
SLAM extension, seeded by the existing map. Outputs are written to
`results/navigation/slam_map/`; the input map is preserved. Only square maps are
currently supported in this mode.

This implementation performs local correlative scan matching with a distance
field and wheel odometry as the motion prior. It requires an accurate initial
pose and nearby mapped geometry. It cannot recover globally from a wrong pose,
has no pose graph or global loop closure, and cannot guarantee drift-free maps.
It is not an autonomous exploration planner: use keyboard mapping to finish
unknown areas first. Keep other robots stationary while extending the map;
moving objects can otherwise become part of it. Treat this as a teaching
implementation, not a substitute for a tested SLAM Toolbox/Nav2 system.

Validation: `python -m unittest discover -s tests -v`. Offline regression
checks cover maximum-range handling, map resumption, PGM orientation, obstacle
inflation, A* routing and corner cuts, odometry, and local scan alignment and
failure. Planning was also checked against the existing `mapping_fixed` map.
Live keyboard input and simulation motion still require validation in your scene.
