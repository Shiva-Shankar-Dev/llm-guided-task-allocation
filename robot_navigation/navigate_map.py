"""Saved-map navigation with wheel odometry and local LiDAR scan matching.

No runtime simulator pose feedback. Initial pose must be supplied in map metres.
This is local localization, not a globally convergent or loop-closing SLAM system.
"""
import argparse
from .paths import DEFAULT_MAP
import heapq
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import distance_transform_edt
import yaml

from .map_environment import Grid, read_scan, steering
from .robot import DEFAULT_ROBOTS, RobotHandles, load_robot, wheel_speeds


def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi


def transform(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points[:, :2] @ np.array([[c, s], [-s, c]]) + pose[:2]


class Map:
    def __init__(self, filename, clearance=0.4):
        filename = Path(filename)
        meta = yaml.safe_load(filename.read_text())
        self.resolution = float(meta['resolution'])
        self.origin = np.array(meta['origin'][:2], dtype=float)
        if self.resolution <= 0 or abs(meta['origin'][2]) > 1e-8:
            raise ValueError('Map must have positive resolution and zero origin yaw')
        pixels = np.flipud(np.asarray(Image.open(filename.parent/meta['image']).convert('L')))
        probability = pixels/255 if meta.get('negate', 0) else 1-pixels/255
        self.occupied = probability > float(meta['occupied_thresh'])
        self.free = probability < float(meta['free_thresh'])
        self.distance = distance_transform_edt(~self.occupied)*self.resolution
        self.clearance = clearance
        self.base_blocked = self.inflate(~self.free)

    def cell(self, xy):
        x, y = np.floor((np.asarray(xy)-self.origin)/self.resolution).astype(int)
        return int(y), int(x)

    def xy(self, cell):
        y, x = cell
        return self.origin + (np.array([x, y])+0.5)*self.resolution

    def valid(self, cell):
        y, x = cell
        return 0 <= y < self.free.shape[0] and 0 <= x < self.free.shape[1]

    def inflate(self, blocked, margin=0.0):
        # Treat outside the image as blocked too.
        padded = np.pad(~blocked, 1, constant_values=False)
        distance = distance_transform_edt(padded)[1:-1, 1:-1]*self.resolution
        return distance <= self.clearance + margin

    def score(self, endpoints, pose):
        world = transform(endpoints, pose)
        cells = np.floor((world-self.origin)/self.resolution).astype(int)
        x, y = cells[:, 0], cells[:, 1]
        valid = (x >= 0) & (y >= 0) & (x < self.free.shape[1]) & (y < self.free.shape[0])
        distances = np.full(len(x), 1.0)
        distances[valid] = self.distance[y[valid], x[valid]]
        return float(np.mean(np.exp(-0.5*(distances/0.12)**2)))

    def preview(self, path, filename):
        pixels = np.where(self.occupied, 0, np.where(self.free, 254, 205)).astype('uint8')
        image = Image.fromarray(np.flipud(pixels)).convert('RGB')
        draw = ImageDraw.Draw(image)
        coordinates = [(x, len(pixels)-1-y) for y,x in path]
        if len(coordinates) > 1:
            draw.line(coordinates, fill=(0,150,50), width=3)
        for point, color in [(coordinates[0], 'blue'), (coordinates[-1], 'red')]:
            x,y = point
            draw.ellipse((x-4,y-4,x+4,y+4), fill=color)
        image.save(filename)


def astar(blocked, start, goal):
    h, w = blocked.shape
    for name, (y, x) in [('Start', start), ('Goal', goal)]:
        if not (0 <= y < h and 0 <= x < w) or blocked[y, x]:
            raise RuntimeError(f'{name} is outside known free space or too close to an obstacle/unknown area')
    queue = [(0, 0, start)]
    costs, parents = {start: 0}, {}
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost > costs[current]:
            continue
        if current == goal:
            path = [current]
            while current in parents:
                current = parents[current]
                path.append(current)
            return path[::-1]
        y, x = current
        for dy, dx in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            yy, xx = y+dy, x+dx
            if not (0 <= yy < h and 0 <= xx < w) or blocked[yy, xx]:
                continue
            if dy and dx and (blocked[y, xx] or blocked[yy, x]):
                continue
            neighbor = (yy, xx)
            new_cost = cost + math.hypot(dx, dy)
            if new_cost < costs.get(neighbor, float('inf')):
                costs[neighbor], parents[neighbor] = new_cost, current
                heuristic = math.hypot(yy-goal[0], xx-goal[1])
                heapq.heappush(queue, (new_cost+heuristic, new_cost, neighbor))
    raise RuntimeError('No safe path: scan more of the area or choose a reachable goal')


def odometry(pose, old_angles, angles, radius, track):
    dl, dr = wrap(np.asarray(angles)-old_angles)*radius
    distance, turn = (dl+dr)/2, (dr-dl)/track
    result = pose.copy()
    # Exact differential-drive arc, including straight motion limit.
    heading = pose[2] + turn/2
    travel = distance*np.sinc(turn/(2*np.pi))
    result[:2] += travel*np.array([math.cos(heading), math.sin(heading)])
    result[2] = wrap(pose[2]+turn)
    return result


def localize(map_, endpoints, predicted):
    if len(endpoints) < 15:
        raise RuntimeError('Too few LiDAR hits for localization; robot stopped')
    endpoints = endpoints[::max(1, len(endpoints)//180)]
    best = predicted.copy()
    # Local correlative search around wheel odometry, then fine refinement.
    for step, angle in [(0.06, math.radians(2)), (0.02, math.radians(0.7))]:
        center = best.copy()
        best_value = -float('inf')
        for dx in [-step, 0, step]:
            for dy in [-step, 0, step]:
                for da in [-angle, 0, angle]:
                    candidate = center + [dx,dy,da]
                    raw_score = map_.score(endpoints, candidate)
                    penalty = 0.03*(np.linalg.norm(candidate[:2]-predicted[:2])/0.1)**2
                    penalty += 0.01*(wrap(candidate[2]-predicted[2])/0.07)**2
                    if raw_score-penalty > best_value:
                        best, best_value = candidate, raw_score-penalty
    score = map_.score(endpoints, best)
    if score < 0.45:
        raise RuntimeError(f'Localization unreliable (score {score:.2f}); verify initial pose and wheel dimensions')
    best[2] = wrap(best[2])
    return best, score


def visible(blocked, a, b):
    count = max(abs(b[0]-a[0]), abs(b[1]-a[1]))*2+1
    cells = np.rint(np.linspace(a,b,count)).astype(int)
    return not blocked[cells[:,0], cells[:,1]].any()


def motion_is_safe(map_, blocked, pose, velocity, omega, horizon=1.0):
    """Check the commanded curved motion, rather than just its target chord."""
    predicted = pose.copy()
    for _ in range(20):
        dt = horizon/20
        heading = predicted[2] + omega*dt/2
        predicted[:2] += velocity*dt*np.sinc(omega*dt/(2*np.pi))*np.array([math.cos(heading), math.sin(heading)])
        predicted[2] += omega*dt
        cell = map_.cell(predicted[:2])
        if not map_.valid(cell) or blocked[cell]:
            return False
    return True


def main():
    from .controller import follow_path
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map', type=Path, default=DEFAULT_MAP)
    p.add_argument('--robot', default='Robot_A', help='robot name from the configuration')
    p.add_argument('--robots-config', type=Path, default=DEFAULT_ROBOTS)
    p.add_argument('--initial-pose', nargs=3, type=float, required=True, metavar=('X','Y','YAW_DEG'))
    p.add_argument('--goal', nargs=2, type=float, required=True, metavar=('X','Y'))
    p.add_argument('--wheel-radius', type=float, default=None)
    p.add_argument('--track', type=float, default=None, help='distance between wheel centres, metres')
    p.add_argument('--clearance', type=float, default=None, help='robot bounding radius plus margin, metres')
    p.add_argument('--planning-margin', type=float, default=0.20, help='extra path clearance for tracking/localization error, metres')
    p.add_argument('--duration', type=float, default=300)
    p.add_argument('--plan-only', action='store_true', help='save planned path without connecting to simulator')
    p.add_argument('--slam', action='store_true', help='experimental: also extend occupancy map using estimated poses; no loop closure')
    p.add_argument('--output', type=Path, default=Path('results/navigation'))
    args = p.parse_args()
    try:
        config = load_robot(args.robot, args.robots_config, wheel_radius=args.wheel_radius,
                            track=args.track, clearance=args.clearance)
    except (ValueError, TypeError, OSError) as exc:
        p.error(str(exc))
    args.wheel_radius, args.track, args.clearance = config.wheel_radius, config.track, config.clearance
    if min(args.wheel_radius, args.track, args.clearance, args.duration) <= 0:
        p.error('wheel dimensions, clearance and duration must be positive')
    if args.planning_margin < 0:
        p.error('planning margin must be nonnegative')
    map_ = Map(args.map, args.clearance)
    pose = np.array(args.initial_pose)
    pose[2] = math.radians(pose[2])
    goal = np.array(args.goal)
    planning_blocked = map_.inflate(~map_.free, args.planning_margin)
    path = astar(planning_blocked, map_.cell(pose[:2]), map_.cell(goal))
    args.output.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output/'path.csv', [map_.xy(c) for c in path], delimiter=',', header='x,y', comments='')
    map_.preview(path, args.output/'path.png')
    print(f'{config.name}: planned {len(path)} cells. Path saved to {args.output}/path.csv')
    if args.plan_only:
        return
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client = RemoteAPIClient()
    sim = client.require('sim')
    handles = RobotHandles.resolve(sim, config)
    robot, motors, sensors = handles.model, handles.motors, handles.sensors
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError('Stop simulation and other controllers first')

    def drive(v, omega):
        speeds = wheel_speeds(config, v, omega)
        for handle, speed in zip(motors, speeds):
            sim.setJointTargetVelocity(handle, float(speed))

    started = False
    dynamic_until = np.zeros(map_.free.shape)
    last_plan, last_match, last_progress = -10, -10, 0
    score, blocked = 0, map_.base_blocked
    progress_position = pose[:2].copy()
    slam_grid = None
    if args.slam:
        if map_.free.shape[0] != map_.free.shape[1]:
            raise ValueError('--slam currently requires a square map')
        slam_grid = Grid(map_.free.shape[0]*map_.resolution, map_.resolution)
        slam_grid.origin = map_.origin.copy()
        slam_grid.odds = np.where(map_.occupied, 3, np.where(map_.free, -3, 0)).astype(np.float32)
        slam_grid.seen = map_.occupied | map_.free
    try:
        sim.setStepping(True)
        drive(0,0)
        sim.startSimulation()
        started = True
        angles = np.array([sim.getJointPosition(h) for h in motors])
        sim.step()
        start = sim.getSimulationTime()
        while sim.getSimulationState() != sim.simulation_stopped:
            now = sim.getSimulationTime()-start
            if now > args.duration:
                raise RuntimeError('Navigation timed out')
            new_angles = np.array([sim.getJointPosition(h) for h in motors])
            pose = odometry(pose, angles, new_angles, args.wheel_radius, args.track)
            angles = new_angles
            # Relative sensor transforms only: no world robot pose is queried.
            scans = read_scan(sim, sensors, relative_to=robot)
            endpoints = np.concatenate([pts[hits,:2] for _,pts,hits in scans])
            if now-last_match >= 0.2:
                pose, score = localize(map_, endpoints, pose)
                last_match = now
            if np.linalg.norm(pose[:2]-goal) < 0.15:
                print('Goal reached')
                break
            if np.linalg.norm(pose[:2]-progress_position) > 0.1:
                last_progress, progress_position = now, pose[:2].copy()
            elif now-last_progress > 20:
                raise RuntimeError('No progress for 20 seconds; robot stopped')
            # Add nearby hits inconsistent with the static map as temporary obstacles.
            for point in transform(endpoints[np.linalg.norm(endpoints, axis=1)<3], pose):
                cell = map_.cell(point)
                if map_.valid(cell) and map_.distance[cell] > 0.15:
                    dynamic_until[cell] = now+5
            if now-last_plan >= 1:
                if slam_grid is not None:
                    # Scan-to-map matching above uses the previous map, then this
                    # accepted pose inserts the new scan (local occupancy-grid SLAM).
                    for origin, points, hits in scans:
                        world_origin = transform(np.array([origin]), pose)[0]
                        slam_grid.update(world_origin, transform(points[::3], pose), hits[::3])
                    map_.occupied = slam_grid.seen & (slam_grid.odds > 0.5)
                    map_.free = slam_grid.seen & (slam_grid.odds < -0.5)
                    map_.distance = distance_transform_edt(~map_.occupied)*map_.resolution
                    slam_grid.save(args.output/'slam_map')
                blocked = map_.inflate((~map_.free) | (dynamic_until > now))
                planning_blocked = map_.inflate((~map_.free) | (dynamic_until > now), args.planning_margin)
                path = astar(planning_blocked, map_.cell(pose[:2]), map_.cell(goal))
                last_plan = now
                print(f'{now:.1f}s pose=({pose[0]:.2f},{pose[1]:.2f}) scan score={score:.2f}')
            current = map_.cell(pose[:2])
            if not map_.valid(current) or blocked[current]:
                raise RuntimeError('Estimated robot pose is outside safe map space')
            velocity, omega = follow_path(map_, planning_blocked, pose, path, scans)
            drive(velocity, omega)
            sim.step()
    except KeyboardInterrupt:
        print('Stopped by user')
    finally:
        if started:
            try:
                drive(0,0)
                sim.stopSimulation()
            finally:
                sim.setStepping(False)


if __name__ == '__main__':
    main()
