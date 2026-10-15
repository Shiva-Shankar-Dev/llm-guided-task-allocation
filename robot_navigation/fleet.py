"""Central fleet loop with explicit task queues and conservative separation.

This prototype yields/stops for conflicts; passage reservations and deadlock
recovery remain future work. One process owns simulation stepping.
"""
import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import numpy as np

from .controller import follow_path
from .map_environment import read_scan
from .navigate_map import Map, astar, localize, odometry, transform
from .paths import DEFAULT_MAP, PROJECT_ROOT
from .robot import DEFAULT_ROBOTS, RobotHandles, load_robot, wheel_speeds


def vector(value, length, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{label} must contain {length} finite numbers')
    return result


def read_mission(filename, robots_file=DEFAULT_ROBOTS):
    data = json.loads(Path(filename).read_text())
    robots = {}
    for name, spec in data['robots'].items():
        config = load_robot(name, robots_file)
        pose = vector(spec['initial_pose'], 3, f'{name} initial_pose').copy()
        pose[2] = math.radians(pose[2])
        robots[name] = (config, pose)
    if not robots or not data['tasks']:
        raise ValueError('Mission needs robots and tasks')
    ids = set()
    for task in data['tasks']:
        if not isinstance(task['id'], str) or not task['id'] or task['id'] in ids:
            raise ValueError('Task IDs must be nonempty unique strings')
        ids.add(task['id'])
        if task['robot'] not in robots:
            raise ValueError(f'Unknown assigned robot: {task["robot"]}')
        task['goal'] = vector(task['goal'], 2, 'goal').tolist()
    for i, task in enumerate(data['tasks']):
        for other in data['tasks'][i+1:]:
            if task['robot'] != other['robot']:
                minimum = robots[task['robot']][0].clearance + robots[other['robot']][0].clearance + .15
                if np.linalg.norm(np.array(task['goal'])-other['goal']) < minimum:
                    raise ValueError('Different robots need separated goals in this prototype')
    return robots, data['tasks']


def trajectory(pose, command, horizon=1.5):
    result = [pose[:2].copy()]
    current = pose.copy()
    v, omega = command
    for _ in range(30):
        dt = horizon/30
        heading = current[2]+omega*dt/2
        current[:2] += v*dt*np.sinc(omega*dt/(2*np.pi))*np.array([math.cos(heading), math.sin(heading)])
        current[2] += omega*dt
        result.append(current[:2].copy())
    return np.asarray(result)


def separate_commands(states, commands, explain=False):
    """Stop both robots if predicted separation is inadequate.

    No winner is selected here: deterministic yielding and reservations are
    deliberately left for the traffic-coordination milestone.
    """
    result = dict(commands)
    reasons = {}
    names = list(states)
    changed = True
    while changed:
        changed = False
        for i, name in enumerate(names):
            for other in names[i+1:]:
                a, b = states[name], states[other]
                distance = np.linalg.norm(trajectory(a.pose, result[name])-trajectory(b.pose, result[other]), axis=1)
                minimum = a.config.clearance+b.config.clearance+.15
                if np.min(distance) < minimum:
                    for robot in (name,other):
                        if result[robot] != (0.0,0.0):
                            result[robot] = (0.0,0.0)
                            peer = other if robot == name else name
                            reasons[robot] = f'predicted separation from {peer} below {minimum:.2f} m'
                            changed = True
    return (result, reasons) if explain else result


def robot_obstacles(map_, own_name, states):
    occupied = np.zeros(map_.free.shape, dtype=bool)
    for name, state in states.items():
        if name == own_name:
            continue
        cy,cx = map_.cell(state.pose[:2])
        radius = int(math.ceil((state.config.clearance+.1)/map_.resolution))
        y0,y1 = max(0,cy-radius), min(occupied.shape[0],cy+radius+1)
        x0,x1 = max(0,cx-radius), min(occupied.shape[1],cx+radius+1)
        if y0 < y1 and x0 < x1:
            yy,xx = np.ogrid[y0:y1,x0:x1]
            occupied[y0:y1,x0:x1] |= (yy-cy)**2+(xx-cx)**2 <= radius**2
    return occupied


def margin_escape(map_, occupied, pose, margin, radius=.3):
    """Recover only from the extra planning margin, without reducing current clearance.

    The short escape stays within 30 cm of the estimate and never admits a
    cell closer to any static/live/peer obstacle than the starting cell.
    """
    from scipy.ndimage import distance_transform_edt
    start = map_.cell(pose[:2])
    if not map_.valid(start):
        return None
    distance = distance_transform_edt(np.pad(~occupied, 1, constant_values=False))[1:-1, 1:-1]*map_.resolution
    clearance = distance[start]
    if clearance <= map_.clearance + .1 or clearance > map_.clearance + margin:
        return None
    full = map_.inflate(occupied, margin)
    yy, xx = np.indices(full.shape)
    nearby = np.hypot(yy-start[0], xx-start[1])*map_.resolution <= radius
    # Restrict the whole recovery path, including its endpoint, to this neighborhood.
    blocked = (~nearby) | (distance < clearance-1e-9)
    candidates = np.argwhere(nearby & ~full)
    candidates = sorted(candidates, key=lambda c: np.linalg.norm(c-np.asarray(start)))
    for candidate in candidates[:40]:
        try:
            path = astar(blocked, start, tuple(candidate))
            return blocked, path
        except RuntimeError:
            continue
    return None


@dataclass
class Agent:
    config: object
    pose: np.ndarray
    map: Map
    handles: object = None
    angles: object = None
    scans: list = field(default_factory=list)
    score: float = 0.0
    last_match: float = -10.0
    dynamic_until: object = None
    last_plan: float = -10.0
    blocked: object = None
    path: list = field(default_factory=list)
    active: object = None
    progress_position: object = None
    progress_time: float = 0.0
    control_reason: str = ''
    path_reason: str = ''
    unavailable: bool = False
    escaping_margin: bool = False
    cruise_speed: float = .12

    def observe(self, sim, now, states):
        angles = np.array([sim.getJointPosition(h) for h in self.handles.motors])
        self.pose = odometry(self.pose, self.angles, angles, self.config.wheel_radius, self.config.track)
        self.angles = angles
        self.scans = read_scan(sim, self.handles.sensors, relative_to=self.handles.model)
        endpoints = np.concatenate([pts[hits,:2] for _,pts,hits in self.scans])
        if now-self.last_match >= .2:
            # Other robot returns are useful obstacles but poor static-map matches.
            world = transform(endpoints, self.pose)
            valid = np.ones(len(world), dtype=bool)
            for state in states.values():
                if state is not self:
                    valid &= np.linalg.norm(world-state.pose[:2], axis=1) > state.config.clearance+.15
            self.pose, self.score = localize(self.map, endpoints[valid], self.pose)
            self.last_match = now
        for point in transform(endpoints[np.linalg.norm(endpoints,axis=1)<3], self.pose):
            # Peers already have a moving footprint layer. Do not leave their
            # observed bodies behind as five-second ghost obstacles.
            if any(np.linalg.norm(point-state.pose[:2]) <= state.config.clearance+.15
                   for state in states.values() if state is not self):
                continue
            cell = self.map.cell(point)
            if self.map.valid(cell) and self.map.distance[cell] > .15:
                self.dynamic_until[cell] = now+5

    def propose(self, now, states, margin):
        self.control_reason = ''
        if self.active is None or self.unavailable:
            return (0.0,0.0)
        if now-self.last_plan >= 1:
            occupied = (~self.map.free) | (self.dynamic_until>now) | robot_obstacles(self.map, self.config.name, states)
            self.blocked = self.map.inflate(occupied, margin)
            self.last_plan = now
            self.escaping_margin = False
            try:
                self.path = astar(self.blocked, self.map.cell(self.pose[:2]), self.map.cell(self.active['goal']))
                self.path_reason = ''
            except RuntimeError as exc:
                escape = margin_escape(self.map, occupied, self.pose, margin)
                if escape is not None:
                    self.blocked, self.path = escape
                    self.escaping_margin = True
                    self.path_reason = ''
                else:
                    self.path = []
                # Compare the actual planning layers rather than guessing the cause.
                start, goal = self.map.cell(self.pose[:2]), self.map.cell(self.active['goal'])
                try:
                    static = self.map.inflate(~self.map.free, margin)
                    astar(static, start, goal)
                except RuntimeError:
                    self.path_reason = f'static map/unknown-space clearance: {exc}'
                else:
                    try:
                        live = self.map.inflate((~self.map.free) | (self.dynamic_until>now), margin)
                        astar(live, start, goal)
                    except RuntimeError:
                        self.path_reason = 'live LiDAR obstacle blocks route (recent hits retained up to 5 seconds)'
                    else:
                        peers = ', '.join(name for name in states if name != self.config.name)
                        self.path_reason = f'robot footprint blocks route: {peers}'
        if not self.path:
            self.control_reason = self.path_reason or 'no safe route available'
            return (0.0,0.0)
        if not self.map.valid(self.map.cell(self.pose[:2])) or self.blocked[self.map.cell(self.pose[:2])]:
            self.control_reason = 'estimated pose is inside the current planning safety margin'
            return (0.0,0.0)
        velocity, omega, self.control_reason = follow_path(
            self.map,self.blocked,self.pose,self.path,self.scans,
            speed=min(.04,self.cruise_speed) if self.escaping_margin else self.cruise_speed,explain=True)
        if self.escaping_margin:
            self.control_reason = 'recovering extra planning margin' + (': '+self.control_reason if self.control_reason else '')
        return velocity, omega


class Journal:
    def __init__(self, output, tasks):
        self.output, self.tasks = output, tasks
        self.events = (output/'events.jsonl').open('w')

    def event(self, now, task, status, reason=''):
        task['status'], task['reason'] = status, reason
        record = dict(time=round(now,3), task=task['id'], robot=task['robot'], status=status, reason=reason)
        for key in ('station', 'slot', 'arrival', 'diagnostic'):
            if key in task:
                record[key] = task[key]
        self.events.write(json.dumps(record)+'\n')
        self.events.flush()
        print(f'{now:.1f}s {task["robot"]} {task["id"]}: {status} {reason}')
        self.save()

    def save(self):
        (self.output/'tasks.json').write_text(json.dumps(self.tasks,indent=2)+'\n')

    def close(self):
        self.save()
        self.events.close()


def recover_robot(name, states, tasks, now, journal, margin=.2, max_reassignments=1):
    """Transfer unfinished queue as one ordered bundle; failed robot stays parked.

    This handles lack of motion with valid sensing/localization, not loss of
    position knowledge. Original robot-specific assignments may be overridden
    on failure and are retained in the audit history.
    """
    source=states[name]
    source.unavailable=True
    active=source.active
    source.active=None
    source.path=[]
    affected=([active] if active is not None else []) + [
        t for t in tasks if t['robot']==name and t['status']=='pending' and t is not active]
    if not affected:
        return None
    reason=f'{name} made no position progress; robot parked for this run'
    for task in affected:
        task.setdefault('original_robot',name)
        task.setdefault('attempt_history',[]).append({'robot':name,'time':round(now,3),'reason':reason})
        journal.event(now,task,'attempt_failed',reason)
    candidates={}
    for other,state in sorted(states.items()):
        if state.unavailable or other==name:
            continue
        if any(t.get('reassignments',0)>=max_reassignments for t in affected):
            continue
        try:
            occupied=(~state.map.free) | (state.dynamic_until>now) | robot_obstacles(state.map,other,states)
            blocked=state.map.inflate(occupied,margin)
            # Include active and queued commitments before the replacement work.
            queue=([state.active] if state.active is not None else []) + [
                t for t in tasks if t['robot']==other and t['status']=='pending' and t is not state.active]
            current=state.pose[:2].copy()
            distance=0.
            for task in queue+affected:
                path=astar(blocked,state.map.cell(current),state.map.cell(task['goal']))
                points=np.asarray([state.map.xy(c) for c in path])
                distance+=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
                distance+=float(np.linalg.norm(current-points[0])+np.linalg.norm(np.asarray(task['goal'])-points[-1]))
                current=np.asarray(task['goal'])
            candidates[other]=distance
        except RuntimeError:
            continue
    if not candidates:
        for task in affected:
            journal.event(now,task,'failed','no reachable available replacement or reassignment limit reached')
        return None
    selected=min(candidates,key=lambda robot:(candidates[robot],robot))
    # Put transferred active task ahead of its original pending successors.
    # All transferred tasks remain behind replacement robot's prior commitments.
    for task in affected:
        tasks.remove(task)
        tasks.append(task)
        task['robot']=selected
        task['reassignments']=task.get('reassignments',0)+1
        task['replacement_candidates_m']={robot:round(cost,3) for robot,cost in candidates.items()}
        journal.event(now,task,'pending',f'reassigned from {name} to {selected}; projected queued route {candidates[selected]:.2f} m')
    return selected


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interactive',action='store_true',help='accept successive instructions while retaining robot poses')
    parser.add_argument('--model',default=None,help='LLM model for interactive instructions')
    parser.add_argument('--stations',type=Path,default=PROJECT_ROOT/'config/stations.json')
    parser.add_argument('--mission',type=Path,default=PROJECT_ROOT/'config/fleet_demo.json')
    parser.add_argument('--robots-config',type=Path,default=DEFAULT_ROBOTS)
    parser.add_argument('--map',type=Path,default=DEFAULT_MAP)
    parser.add_argument('--output',type=Path,default=Path('results/fleet'))
    parser.add_argument('--plan-only',action='store_true')
    parser.add_argument('--initialize-from-sim',action='store_true',help='read actual robot poses ONCE while stopped, then use wheels/LiDAR')
    parser.add_argument('--duration',type=float,default=300)
    parser.add_argument('--speed',type=float,default=.12,help='cruise speed in m/s (0 < speed <= 0.18); turns and safety checks may reduce it')
    parser.add_argument('--planning-margin',type=float,default=.2)
    parser.add_argument('--stall-timeout',type=float,default=30,help='simulation seconds without position progress before reassignment')
    parser.add_argument('--max-reassignments',type=int,default=1,help='replacement attempts per task; 0 disables replacement')
    parser.add_argument('--simulate-stall',help='demo only: hold this mission robot stationary to test task replacement')
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or not 0 < args.speed <= .18:
        parser.error('--speed must be finite, positive, and no greater than 0.18 m/s')
    if not math.isfinite(args.duration) or args.duration<=0 or not math.isfinite(args.planning_margin) or args.planning_margin<0:
        parser.error('Duration must be positive and planning margin nonnegative')
    if not math.isfinite(args.stall_timeout) or args.stall_timeout<=0 or args.max_reassignments<0:
        parser.error('stall timeout must be positive and reassignment limit nonnegative')
    if args.plan_only and args.initialize_from_sim:
        parser.error('--plan-only does not connect to the simulator; use mission initial poses')
    configs,tasks = read_mission(args.mission,args.robots_config)
    if args.simulate_stall and args.simulate_stall not in configs:
        parser.error('--simulate-stall must name a robot in the mission')
    if args.interactive and args.plan_only:
        parser.error('--interactive cannot be combined with --plan-only')
    if args.interactive:
        tasks=[]  # Mission file selects fleet membership; console supplies work.
    states = {name: Agent(config,pose,Map(args.map,config.clearance),cruise_speed=args.speed) for name,(config,pose) in configs.items()}
    args.output.mkdir(parents=True,exist_ok=True)
    if args.plan_only:
        for name,state in states.items():
            start = state.pose[:2]
            for number,task in enumerate(t for t in tasks if t['robot']==name):
                blocked = state.map.inflate(~state.map.free,args.planning_margin)
                path = astar(blocked,state.map.cell(start),state.map.cell(task['goal']))
                output = args.output/f'{name}_task_{number+1}.png'
                state.map.preview(path,output)
                print(f'{name}: {task["id"]}, {len(path)} cells -> {output}')
                start = np.asarray(task['goal'])
        print('Static previews only: online robot positions and obstacles may change the routes.')
        return
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client = RemoteAPIClient()
    sim = client.require('sim')
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError('Stop simulation and all other controllers first')
    used_models,used_motors = set(),set()
    for name,state in states.items():
        state.handles = RobotHandles.resolve(sim,state.config)
        if state.handles.model in used_models or used_motors.intersection(state.handles.motors):
            raise ValueError('Fleet robots must resolve to distinct models and motors')
        used_models.add(state.handles.model)
        used_motors.update(state.handles.motors)
        if args.initialize_from_sim:
            pos=sim.getObjectPosition(state.handles.model,sim.handle_world)
            matrix=sim.getObjectMatrix(state.handles.model,sim.handle_world)
            state.pose=np.array([pos[0],pos[1],math.atan2(matrix[4],matrix[0])])
        state.dynamic_until=np.zeros(state.map.free.shape)
        state.progress_position=state.pose[:2].copy()
        if not state.map.valid(state.map.cell(state.pose[:2])) or state.map.base_blocked[state.map.cell(state.pose[:2])]:
            raise ValueError(f'{name}: initial pose is not in safe mapped free space')
    for i,a in enumerate(list(states.values())):
        for b in list(states.values())[i+1:]:
            if np.linalg.norm(a.pose[:2]-b.pose[:2]) < a.config.clearance+b.config.clearance+.15:
                raise ValueError('Robots start too close together')
    journal = Journal(args.output,tasks)
    for task in tasks:
        journal.event(0,task,'pending')
    session = None
    if args.interactive:
        from .session import ConsoleSession
        session = ConsoleSession(args)
    started,now = False,0.0
    batch_start=0.0
    failure = None

    def command_all(commands):
        for name,state in states.items():
            for motor,speed in zip(state.handles.motors,wheel_speeds(state.config,*commands[name])):
                sim.setJointTargetVelocity(motor,float(speed))

    try:
        sim.setStepping(True)
        command_all({name:(0,0) for name in states})
        for state in states.values():
            state.angles=np.array([sim.getJointPosition(h) for h in state.handles.motors])
        sim.startSimulation()
        started=True
        sim.step()
        start=sim.getSimulationTime()
        while sim.getSimulationState()!=sim.simulation_stopped:
            now=sim.getSimulationTime()-start
            if now-batch_start>args.duration:
                raise RuntimeError('Fleet timeout')
            # Freeze peer poses for this observation phase to avoid name-order bias.
            snapshot={name:Agent(s.config,s.pose.copy(),s.map) for name,s in states.items()}
            for name,state in states.items():
                peers={n:s for n,s in snapshot.items() if n!=name}
                state.observe(sim,now,peers)
            for name,state in states.items():
                if state.active is not None and np.linalg.norm(state.pose[:2]-state.active['goal'])<.15:
                    state.active['arrival'] = {
                        'estimated_pose': state.pose.tolist(),
                        'distance_to_slot_m': float(np.linalg.norm(state.pose[:2]-state.active['goal'])),
                        'tolerance_m': .15, 'scan_score': float(state.score)}
                    journal.event(now,state.active,'completed')
                    state.active=None
                if state.active is None and not state.unavailable:
                    state.active=next((t for t in tasks if t['robot']==name and t['status']=='pending'),None)
                    if state.active is not None:
                        journal.event(now,state.active,'running')
                        state.last_plan=-10
                        state.progress_time=now
                        state.progress_position=state.pose[:2].copy()
            if all(t['status'] in ('completed','failed','cancelled') for t in tasks):
                command_all({name:(0,0) for name in states})
                if session is not None:
                    # Stepping remains owned here: no physics advances while
                    # input/API parsing waits, and odometry is never reset.
                    incoming=session.next_tasks(states,now)
                    if incoming is None:
                        print('Session ended by user')
                        break
                    tasks.extend(incoming)
                    for task in incoming:
                        journal.event(now,task,'pending')
                    batch_start=now
                    continue
                failed=sum(t['status']!='completed' for t in tasks)
                if failed:
                    failure=f'{failed} task(s) could not be recovered'
                else:
                    print('All fleet tasks completed')
                break
            proposed={name:s.propose(now,states,args.planning_margin) for name,s in states.items()}
            if args.simulate_stall:
                proposed[args.simulate_stall]=(0.0,0.0)
                states[args.simulate_stall].control_reason='simulated stall for feedback demonstration'
            commands, separation_reasons=separate_commands(states,proposed,explain=True)
            for name,state in states.items():
                if state.active is None:
                    continue
                status='waiting' if commands[name]==(0.0,0.0) else 'running'
                reason=separation_reasons.get(name,state.control_reason)
                if status!=state.active['status'] or reason!=state.active.get('reason',''):
                    state.active['diagnostic'] = {'estimated_pose':state.pose.tolist(),
                                                  'scan_score':float(state.score)}
                    journal.event(now,state.active,status,reason)
                if name!=args.simulate_stall and np.linalg.norm(state.pose[:2]-state.progress_position)>.1:
                    state.progress_time=now
                    state.progress_position=state.pose[:2].copy()
                elif now-state.progress_time>args.stall_timeout:
                    recover_robot(name,states,tasks,now,journal,args.planning_margin,args.max_reassignments)
                    commands[name]=(0.0,0.0)
            command_all(commands)
            sim.step()  # Exactly one step for the entire fleet.
        else:
            raise RuntimeError('Simulation stopped before all tasks finished')
    except KeyboardInterrupt:
        failure='Cancelled by user'
    except Exception as exc:
        failure=str(exc)
    finally:
        # Attempt every stop even if one simulator/motor command fails.
        for state in states.values():
            for motor in state.handles.motors:
                try:
                    sim.setJointTargetVelocity(motor,0)
                except Exception as exc:
                    failure=failure or f'Motor stop failed: {exc}'
        try:
            if started:
                sim.stopSimulation()
        except Exception as exc:
            failure = failure or f'Simulation stop failed: {exc}'
        finally:
            try:
                sim.setStepping(False)
            except Exception as exc:
                failure = failure or f'Stepping reset failed: {exc}'
            if failure:
                for task in tasks:
                    if task['status'] not in ('completed','failed','cancelled'):
                        journal.event(now,task,'cancelled' if failure=='Cancelled by user' else 'failed',failure)
            journal.close()
    if failure:
        raise SystemExit(f'Fleet stopped: {failure}. See {args.output}/tasks.json')


if __name__=='__main__':
    main()
