import json
import time
import os
import csv
import threading
from datetime import datetime
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from google.genai import types
from google import genai
from google.genai.errors import ServerError
from .paths import PROJECT_ROOT

# ==========================================
# 0. Logging (thread-safe)
# ==========================================
log_lock = threading.Lock()
_log_directory = PROJECT_ROOT / 'results' / 'logs'
_log_directory.mkdir(parents=True, exist_ok=True)
_log_file = open(_log_directory / 'execution_log.csv', "a", newline="")
_logger = csv.writer(_log_file)
if _log_file.tell() == 0:
    _logger.writerow(["timestamp", "robot", "event", "detail"])

def log_event(robot: str, event: str, detail: str = ""):
    with log_lock:
        _logger.writerow([datetime.now().isoformat(), robot, event, detail])
        _log_file.flush()
    print(f"[LOG] {robot} | {event} | {detail}")

# Shared lock protecting the per-robot task queues during concurrent execution
plan_lock = threading.Lock()

# ==========================================
# 1. Connect to CoppeliaSim & Fetch Handles
# ==========================================
client = RemoteAPIClient()
sim = client.getObject('sim')

robots = {
    "Robot_A": sim.getObject('/Robot_A'),
    "Robot_B": sim.getObject('/Robot_B'),
}

motors = {
    "Robot_A": {
        "left": sim.getObject('/Robot_A/leftMotor'),
        "right": sim.getObject('/Robot_A/rightMotor'),
    },
    "Robot_B": {
        "left": sim.getObject('/Robot_B/leftMotor'),
        "right": sim.getObject('/Robot_B/rightMotor'),
    },
}

# Resolve front proximity sensors (ultrasonicSensor[4] is the center-front sensor)
sensors = {}
# Resolve front-left, front-center, and front-right sonar sensors.
# Pioneer p3dx ships with 16 sonars (ultrasonicSensor[0..15]) arranged
# around the ring; indices 2/3/4/5 are the front-facing cluster on the
# standard CoppeliaSim model, with [3]/[4] closest to center and [2]/[5]
# angled slightly left/right. If your model's sonar layout differs, check
# the exact names under each robot in the scene hierarchy and adjust the
# indices below.
def _resolve_sensor(bot_name, index):
    try:
        return sim.getObject(f'/{bot_name}/ultrasonicSensor[{index}]')
    except Exception:
        return None

sensors = {}          # front-center sensor (used for the stop/target-area check)
sensors_lr = {}        # {"left": handle, "right": handle} -- used for steering direction
for bot_name in ["Robot_A", "Robot_B"]:
    center = _resolve_sensor(bot_name, 4) or _resolve_sensor(bot_name, 3)
    left = _resolve_sensor(bot_name, 2)
    right = _resolve_sensor(bot_name, 5)
    sensors[bot_name] = center
    sensors_lr[bot_name] = {"left": left or center, "right": right or center}
    if center is None:
        print(f"[WARN] Proximity sensor handle not resolved for {bot_name}.")

# ==========================================
# 2. Landmark Table (Scene Coordinates)
# ==========================================
landmarks = {
    "charging_station": (0.0, 5.0),    # Top-left (wood)
    "left_corridor": (5.0, 0.0),       # Bottom-left (checkered)
    "loading_zone": (4.5, 4.5),        # Top-right (concrete)
}

# ==========================================
# 2b. Inner Partition Wall Geometry (measured directly in CoppeliaSim)
# ==========================================
# The inner wall runs along a CONSTANT X-LINE (not a constant Y-line, which
# was the earlier -- incorrect -- assumption). It's a PARTIAL partition: it
# does not span the whole room, so it has no gap cut into its middle --
# instead, a robot must go around whichever END of the wall is closer.
#
# Measured wall: position X=2.550, spanning Y from -2.400 to +2.375
# (matches its stated size, X:0.2 (thickness) Y:5.0 (length) Z:0.5 (height)).
WALL_X = 2.550
WALL_Y_MIN = -2.400
WALL_Y_MAX = 2.475

# How far past each end of the wall a bypass waypoint should sit, so the
# robot clears the wall's corner with margin rather than clipping it.
GAP_MARGIN = 0.4
GAP_NORTH_Y = WALL_Y_MAX + GAP_MARGIN   # bypass route over the top end
GAP_SOUTH_Y = WALL_Y_MIN - GAP_MARGIN   # bypass route under the bottom end


def crosses_partition(current_xy, target_xy, wall_x=WALL_X):
    """True if current and target are on opposite sides of the wall's X line."""
    return (current_xy[0] < wall_x) != (target_xy[0] < wall_x)


def choose_bypass_y(current_xy, target_xy):
    """
    Pick whichever end of the wall (north or south) gives the shorter
    detour, based on the average Y of the current position and target.
    """
    avg_y = (current_xy[1] + target_xy[1]) / 2.0
    if abs(avg_y - GAP_NORTH_Y) <= abs(avg_y - GAP_SOUTH_Y):
        return GAP_NORTH_Y
    return GAP_SOUTH_Y

# ==========================================
# 3. LLM Planner (Task Decomposition)
# ==========================================
client_llm = genai.Client()

SYSTEM_PROMPT = """You convert multi-robot natural language instructions into a JSON task plan.

Available robots: Robot_A, Robot_B
Available locations: charging_station, left_corridor, loading_zone
Available actions: goto, patrol, avoid

Rules:
- Assign each sub-task to exactly one robot based on what the instruction implies.
- If the instruction doesn't specify which robot, split tasks evenly.
- "avoid" constraints apply to a robot for its entire task sequence.
- Output ONLY valid JSON, no markdown, no explanation, in this exact format:

{
  "Robot_A": [{"action": "goto", "location": "charging_station"}],
  "Robot_B": [{"action": "goto", "location": "loading_zone"}]
}
"""

def get_task_plan(instruction: str, max_retries=3) -> dict:
    prompt = f"{SYSTEM_PROMPT}\n\nUser Instruction: {instruction}"
    models_to_try = ["gemini-3.6-flash", "gemini-3.1-pro-preview"]

    for model_name in models_to_try:
        for attempt in range(max_retries):
            try:
                chat = client_llm.chats.create(model=model_name)
                response = chat.send_message(
                    message=prompt,
                    config=types.GenerateContentConfig(
                        response_mime_type="application/json",
                        temperature=0.0
                    )
                )
                raw = response.text.strip()
                raw = raw.replace("```json", "").replace("```", "").strip()
                return json.loads(raw)
            except ServerError:
                print(f"[API BUSY] {model_name} returned 503 (attempt {attempt+1}/{max_retries}). Retrying in 2s...")
                time.sleep(2)
            except Exception as e:
                print(f"[API ERROR] {e}")
                break
    raise RuntimeError("Failed to obtain task plan after retries.")

# ==========================================
# 4. Grounding & Validation Layer
# ==========================================
def validate_plan(plan: dict, landmark_table: dict) -> dict:
    validated = {}
    for robot, steps in plan.items():
        clean_steps = []
        for step in steps:
            loc = step.get("location")
            if loc not in landmark_table:
                print(f"[REJECTED] {robot}: unknown landmark '{loc}'")
                continue
            clean_steps.append(step)
        validated[robot] = clean_steps
    return validated

# ==========================================
# 5. Motion Controller
# ==========================================
def move_to_target(sim, robot_handle, motor_pair, target_xy, sensor_handle=None,
                    sensor_lr=None, threshold=0.25, robot_name="?", speed_scale=1.0):
    """
    Drives robot_handle toward target_xy.
    Returns True if the target was reached.
    Returns False if the robot has made no meaningful progress toward the
    target for MAX_STALL_WINDOWS consecutive windows -- this is the trigger
    for reassignment.

    sensor_lr: optional {"left": handle, "right": handle} pair used to steer
    AWAY from whichever side is actually closer to an obstacle, instead of
    always pivoting the same fixed direction regardless of which side was
    triggered. A single fixed maneuver works for a robot approaching a wall
    dead-on, but fails in tighter spots like a doorway, where the robot may
    clip either side depending on its approach angle -- turning the same
    way every time can just push it into the opposite side of the frame.

    NOTE on why this uses distance-progress rather than counting sensor
    ticks: a naive "count consecutive blocked sensor ticks" approach fails
    against a wall, because the avoidance maneuver itself (reversing away
    from the obstacle) clears the sensor reading almost immediately, which
    resets any tick-based counter before it can accumulate -- the robot
    then turns straight back toward the target, re-triggers the sensor,
    and repeats forever without ever being flagged as stuck. Measuring
    actual distance-to-target closed over a time window catches this
    "oscillating in place" failure mode directly, regardless of what the
    sensor is doing tick-to-tick.
    """
    avoid_timer = 0

    PROGRESS_CHECK_INTERVAL = 20   # ticks (~1s @ 0.05s/tick)
    MIN_PROGRESS = 0.05            # must net-close at least 5cm per window
    MAX_STALL_WINDOWS = 8          # ~8s of no net progress => genuinely stuck
    OBSTACLE_RANGE = 0.5           # meters -- slightly wider than before so
                                    # the left/right comparison has margin
                                    # to tell sides apart before impact

    stall_windows = 0
    ticks_in_window = 0
    distance_at_window_start = None

    while True:
        pos = sim.getObjectPosition(robot_handle, -1)
        orientation = sim.getObjectOrientation(robot_handle, -1)
        dx = target_xy[0] - pos[0]
        dy = target_xy[1] - pos[1]
        distance = np.hypot(dx, dy)

        # 1. Target check
        if distance < threshold:
            sim.setJointTargetVelocity(motor_pair["left"], 0.0)
            sim.setJointTargetVelocity(motor_pair["right"], 0.0)
            return True

        if distance_at_window_start is None:
            distance_at_window_start = distance

        # 1b. Progress-based stall check -- runs independently of the
        # sensor/avoidance logic below, so it can't be reset by a maneuver
        # that briefly clears the sensor.
        ticks_in_window += 1
        if ticks_in_window >= PROGRESS_CHECK_INTERVAL:
            progress = distance_at_window_start - distance
            stall_windows = 0 if progress >= MIN_PROGRESS else stall_windows + 1
            distance_at_window_start = distance
            ticks_in_window = 0

            if stall_windows >= MAX_STALL_WINDOWS:
                sim.setJointTargetVelocity(motor_pair["left"], 0.0)
                sim.setJointTargetVelocity(motor_pair["right"], 0.0)
                log_event(robot_name, "stuck",
                          f"no progress toward {target_xy} for "
                          f"{MAX_STALL_WINDOWS * PROGRESS_CHECK_INTERVAL * 0.05:.0f}s")
                return False

        # 2. Check left/right sensors independently so we know WHICH side
        # is blocked, not just "something is close."
        left_dist = right_dist = None
        if sensor_lr is not None:
            if sensor_lr.get("left") is not None:
                res_l, d_l, *_ = sim.readProximitySensor(sensor_lr["left"])
                left_dist = d_l if res_l > 0 else None
            if sensor_lr.get("right") is not None:
                res_r, d_r, *_ = sim.readProximitySensor(sensor_lr["right"])
                right_dist = d_r if res_r > 0 else None
        elif sensor_handle is not None:
            res, dist, *_ = sim.readProximitySensor(sensor_handle)
            if res > 0 and dist < OBSTACLE_RANGE:
                left_dist = right_dist = dist  # no side info -- treat as centered

        left_blocked = left_dist is not None and left_dist < OBSTACLE_RANGE
        right_blocked = right_dist is not None and right_dist < OBSTACLE_RANGE

        if left_blocked or right_blocked:
            avoid_timer = 25  # Maintain turn for ~1.25s (25 ticks @ 0.05s)
            # Steer AWAY from whichever side is closer, instead of a fixed
            # direction. If only one side reports a distance, treat the
            # other as clear (safe to turn toward).
            l = left_dist if left_dist is not None else float("inf")
            r = right_dist if right_dist is not None else float("inf")
            turn_away_from_left = l <= r  # obstacle is on/nearer the left -> turn right

        if avoid_timer > 0:
            avoid_timer -= 1
            if turn_away_from_left:
                # Obstacle on the left: pivot right while backing off
                sim.setJointTargetVelocity(motor_pair["left"], 1.2)
                sim.setJointTargetVelocity(motor_pair["right"], -0.4)
            else:
                # Obstacle on the right: pivot left while backing off
                sim.setJointTargetVelocity(motor_pair["left"], -0.4)
                sim.setJointTargetVelocity(motor_pair["right"], 1.2)
            time.sleep(0.05)
            continue

        # 4. Normal proportional heading steering
        target_angle = np.arctan2(dy, dx)
        angle_diff = (target_angle - orientation[2] + np.pi) % (2 * np.pi) - np.pi

        base_speed = 2.0 * speed_scale
        turn_gain = 3.5
        left_speed = base_speed - turn_gain * angle_diff
        right_speed = base_speed + turn_gain * angle_diff

        sim.setJointTargetVelocity(motor_pair["left"], left_speed)
        sim.setJointTargetVelocity(motor_pair["right"], right_speed)

        time.sleep(0.05)

# ==========================================
# 6. Per-Robot Worker (runs concurrently, one thread per robot)
# ==========================================
def robot_worker(robot_name: str, plan: dict, other_robot_name: str,
                  busy: dict, stop_event: threading.Event):
    """
    Pulls steps off plan[robot_name] one at a time and executes them.
    If a step can't be completed (move_to_target returns False = stuck),
    the step is handed off to other_robot_name's queue instead of being
    dropped -- this is the feedback/reassignment loop (COHERENT-style),
    layered on top of the SMART-LLM-style plan this thread is executing.

    NOTE: this is a simple two-robot handoff, not a full negotiation
    protocol -- there's no check here that other_robot_name can actually
    reach the handed-off location either. Documented as a scoped
    simplification vs. COHERENT's full proposal/feedback cycle.

    IMPORTANT -- why this polls instead of exiting on an empty queue:
    an earlier version broke out of the loop the first time its queue
    was empty. That created a race condition: if Robot_B finished its
    own tasks and exited BEFORE Robot_A got stuck and reassigned a step
    to it, the reassigned step landed in Robot_B's queue with no thread
    left alive to read it -- the step silently never executed. Polling
    with `stop_event` lets a worker stay "listening" even after it runs
    dry, until the main thread confirms EVERY robot is simultaneously
    idle (see run_demo) and explicitly signals shutdown.

    IMPORTANT -- per-thread connection: each thread opens its OWN
    RemoteAPIClient connection, since the ZMQ remote API socket is not
    thread-safe. Object handles (plain integers) are valid across any
    connection to the same running simulation, so the existing handles
    in `robots`/`motors`/`sensors` can be reused with this thread's own
    `local_sim` -- only the transport is per-thread, not the scene state.
    """
    local_client = RemoteAPIClient()
    local_sim = local_client.getObject('sim')
    logged_idle = False

    while not stop_event.is_set():
        with plan_lock:
            if plan[robot_name]:
                step = plan[robot_name].pop(0)
                busy[robot_name] = True
                logged_idle = False
            else:
                busy[robot_name] = False
                step = None

        if step is None:
            if not logged_idle:
                log_event(robot_name, "idle", "queue empty -- waiting")
                logged_idle = True
            time.sleep(0.1)
            continue

        if step["action"] == "avoid":
            continue

        # Guard against infinite ping-pong reassignment if a location is
        # unreachable by both robots (e.g. sealed off by an obstacle).
        step.setdefault("_reassign_count", 0)
        if step["_reassign_count"] >= 2:
            print(f"[DROPPED] '{step['location']}' unreachable by either "
                  f"robot after {step['_reassign_count']} reassignments. "
                  f"Giving up on this step.")
            log_event(robot_name, "dropped", step["location"])
            continue

        loc_name = step["location"]
        target = landmarks[loc_name]
        bot_sensor = sensors.get(robot_name)
        bot_sensor_lr = sensors_lr.get(robot_name)
        current_pos = local_sim.getObjectPosition(robots[robot_name], -1)

        # Route around whichever end of the partition wall is closer,
        # whenever this move crosses the wall's X line -- based on the
        # robot's CURRENT position vs. the target, not a hardcoded
        # destination name.
        #
        # IMPORTANT: this wall is a PARTIAL partition (it doesn't span the
        # whole room), so there's no doorway cut into its middle to aim
        # for -- the only way across is around one of its two ends. An
        # earlier version assumed a full-width wall with a central gap,
        # which was a wrong model of this specific scene's geometry (and
        # was also checking the wrong axis entirely). The two waypoints
        # below move the robot to the chosen end's Y-value first (still on
        # its current side of the wall), then across to the target's side
        # at that same Y (now clear of the wall entirely), before the
        # final leg proceeds directly to the actual target.
        if crosses_partition(current_pos, target):
            bypass_y = choose_bypass_y(current_pos, target)
            log_event(robot_name, "routing", f"bypass wall end at y={bypass_y}")
            print(f">> {robot_name} routing around partition wall (y={bypass_y})...")
            move_to_target(
                local_sim, robots[robot_name], motors[robot_name],
                (current_pos[0], bypass_y), sensor_handle=bot_sensor,
                sensor_lr=bot_sensor_lr, robot_name=robot_name, speed_scale=0.7
            )
            move_to_target(
                local_sim, robots[robot_name], motors[robot_name],
                (target[0], bypass_y), sensor_handle=bot_sensor,
                sensor_lr=bot_sensor_lr, robot_name=robot_name, speed_scale=0.7
            )

        log_event(robot_name, "task_start", loc_name)
        print(f">> {robot_name} navigating to '{loc_name}' at coordinates {target}...")
        reached = move_to_target(
            local_sim, robots[robot_name], motors[robot_name], target,
            sensor_handle=bot_sensor, sensor_lr=bot_sensor_lr, robot_name=robot_name
        )

        if reached:
            log_event(robot_name, "task_complete", loc_name)
            print(f">> {robot_name} successfully arrived at {loc_name}!\n")
        else:
            # Genuinely blocked -- hand this step to the other robot rather
            # than abandoning it. This is the "context-aware, dynamic"
            # behavior the project claims: a failed assignment triggers
            # reassignment instead of silently failing.
            print(f"[REPLAN] {robot_name} could not reach '{loc_name}' "
                  f"(blocked). Reassigning to {other_robot_name}.\n")
            log_event(robot_name, "reassigned", f"{loc_name} -> {other_robot_name}")
            step["_reassign_count"] += 1
            if other_robot_name is not None:
                with plan_lock:
                    plan[other_robot_name].append(step)

    log_event(robot_name, "thread_exit", "stop signal received")


# ==========================================
# 7. Execution Loop (concurrent dispatch)
# ==========================================
def run_demo(instruction: str):
    print("\n==========================================")
    print(f"INPUT INSTRUCTION: '{instruction}'")
    print("==========================================")

    # Zero motors immediately while LLM computes
    for bot in motors.values():
        sim.setJointTargetVelocity(bot["left"], 0.0)
        sim.setJointTargetVelocity(bot["right"], 0.0)

    # 1. Query LLM for Task Decomposition
    print("\n[1] Querying LLM for Task Decomposition...")
    raw_plan = get_task_plan(instruction)
    print(f"Generated JSON Plan:\n{json.dumps(raw_plan, indent=2)}")
    log_event("system", "plan_generated", json.dumps(raw_plan))

    # 2. Validate against Physical Grounding Table
    print("\n[2] Validating against Physical Grounding Table...")
    plan = validate_plan(raw_plan, landmarks)
    for robot_name in robots:
        plan.setdefault(robot_name, [])  # ensure every robot has a queue, even if empty
    print("Validation passed!")

    # 3. Start simulation physics after planning is complete
    if sim.getSimulationState() == sim.simulation_stopped:
        sim.startSimulation()
        time.sleep(0.2)

    # 4. Execute commands -- one thread per robot, running concurrently.
    # Each thread only knows about "the other" robot for handoff purposes;
    # this assumes exactly two robots. Generalizing to N robots would need
    # a shared "which robots are idle/available" check instead of a fixed pair.
    #
    # Threads are NOT simply .join()'d as soon as they run out of queued
    # work -- see robot_worker's docstring for why: a robot that finishes
    # first and exits immediately can miss a step reassigned to it later
    # by a robot that took longer to get stuck. Instead, the main thread
    # polls until every robot is simultaneously idle (no queue, not mid-
    # move) for a sustained window, THEN signals all workers to stop.
    print("\n[3] Executing Robot Commands in CoppeliaSim (concurrent)...")
    robot_names = list(robots.keys())
    busy = {name: False for name in robot_names}
    stop_event = threading.Event()

    threads = []
    for i, robot_name in enumerate(robot_names):
        other = robot_names[1 - i] if len(robot_names) == 2 else None
        t = threading.Thread(
            target=robot_worker, args=(robot_name, plan, other, busy, stop_event),
            daemon=True
        )
        threads.append(t)
        t.start()

    # Require ~1s (10 ticks @ 0.1s) of EVERY robot being simultaneously
    # idle before declaring the run finished -- this window is what
    # prevents the race where a step gets reassigned into a queue whose
    # worker already decided it was done.
    CONFIRM_TICKS_NEEDED = 10
    idle_confirm_ticks = 0
    while True:
        with plan_lock:
            all_idle = all(not busy[r] and not plan[r] for r in robot_names)
        idle_confirm_ticks = idle_confirm_ticks + 1 if all_idle else 0
        if idle_confirm_ticks >= CONFIRM_TICKS_NEEDED:
            break
        time.sleep(0.1)

    stop_event.set()
    for t in threads:
        t.join(timeout=2)

    print("All tasks completed successfully.\n")

if __name__ == "__main__":
    print("=" * 50)
    print(" Multi-Robot Natural Language Navigation CLI")
    print("=" * 50)

    while True:
        try:
            user_input = input("\nEnter multi-robot instruction (or 'q' to quit): ").strip()
            if user_input.lower() in ["q", "quit", "exit"]:
                print("Exiting...")
                break
            if not user_input:
                continue
            run_demo(user_input)
        except KeyboardInterrupt:
            print("\nExecution stopped.")
            break
