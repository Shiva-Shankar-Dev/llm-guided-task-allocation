"""Drive Robot A and map standard fastHokuyo auxiliary packets (no ROS).

Uses simulator poses, not SLAM. See docs/MAPPING.md. Never run with main.py.
"""
import argparse
import math
from pathlib import Path
import select
import sys
import termios
import time
import tty

import numpy as np
from .robot import load_robot, resolve_lidar


def decode_packet(packet):
    """Hokuyo vision callback packet: width, height, then x,y,z,d per ray."""
    if packet is None or len(packet) < 6:
        return None
    a = np.asarray(packet, dtype=float)
    if not np.all(np.isfinite(a[:2])):
        return None
    w, h = int(a[0]), int(a[1])
    if w <= 0 or h <= 0 or w != a[0] or h != a[1] or len(a) != 2 + 4*w*h:
        return None
    return a[2:].reshape(-1, 4)


def world_points(matrix, xyz):
    m = np.asarray(matrix).reshape(3, 4)
    return xyz @ m[:, :3].T + m[:, 3]


class Grid:
    @classmethod
    def load(cls, filename):
        with np.load(filename, allow_pickle=False) as data:
            grid = cls(1, float(data['resolution']))
            grid.odds = data['log_odds'].copy()
            grid.seen = data['seen'].copy()
            grid.origin = data['origin'].copy()
        if grid.odds.shape != grid.seen.shape or grid.odds.ndim != 2 or grid.odds.shape[0] != grid.odds.shape[1]:
            raise ValueError('Resume file must contain a square mapping grid')
        return grid

    def __init__(self, size=30.0, resolution=0.05):
        self.resolution = resolution
        n = int(math.ceil(size / resolution))
        self.origin = np.array([-n*resolution/2]*2)
        self.odds = np.zeros((n, n), dtype=np.float32)
        self.seen = np.zeros((n, n), dtype=bool)
        self.clipped = False

    def cell(self, xy):
        return np.floor((np.asarray(xy)[:2]-self.origin)/self.resolution).astype(int)

    def update(self, origin, endpoints, hits):
        start = self.cell(origin)
        n = self.odds.shape[0]
        free, occupied = set(), set()
        for point, hit in zip(endpoints, hits):
            end = self.cell(point)
            count = int(np.max(np.abs(end-start))) + 1
            cells = np.rint(np.linspace(start, end, count)).astype(int)
            valid = np.all((cells >= 0) & (cells < n), axis=1)
            if not np.all(valid):
                self.clipped = True
            # Do not label an obstacle endpoint as free.
            for x, y in cells[:-1] if hit else cells:
                if 0 <= x < n and 0 <= y < n:
                    free.add((y, x))
            if hit and valid[-1]:
                occupied.add((end[1], end[0]))
        for y, x in free - occupied:
            self.odds[y, x] -= 0.4
            self.seen[y, x] = True
        for y, x in occupied:
            self.odds[y, x] += 0.85
            self.seen[y, x] = True
        np.clip(self.odds, -5, 5, out=self.odds)

    def save(self, directory):
        directory.mkdir(parents=True, exist_ok=True)
        pixels = np.full(self.odds.shape, 205, dtype=np.uint8)
        pixels[self.seen & (self.odds < -0.5)] = 254
        pixels[self.seen & (self.odds > 0.5)] = 0
        n = len(pixels)
        (directory/'map.pgm').write_bytes(
            f'P5\n{n} {n}\n255\n'.encode() + np.flipud(pixels).tobytes())
        (directory/'map.yaml').write_text(
            f'image: map.pgm\nresolution: {self.resolution}\n'
            f'origin: [{self.origin[0]}, {self.origin[1]}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        np.savez_compressed(directory/'map_data.npz', log_odds=self.odds,
                            seen=self.seen, origin=self.origin, resolution=self.resolution)


def read_scan(sim, sensors, relative_to=None):
    scans = []
    for sensor in sensors:
        result = sim.readVisionSensor(sensor)
        if not result or result[0] < 0:
            raise RuntimeError('Hokuyo has not produced a scan. Check its simulation/vision scripts.')
        points = None
        for packet in result[2:]:
            points = decode_packet(packet)
            if points is not None:
                break
        if points is None:
            raise RuntimeError('No fastHokuyo coordinate packet found. See MAPPING.md.')
        far = sim.getObjectFloatParam(sensor, sim.visionfloatparam_far_clipping)
        valid = np.all(np.isfinite(points), axis=1) & (points[:, 3] > 0.05)
        points = points[valid]
        if not len(points):
            raise RuntimeError('Hokuyo scan is empty; robot stopped.')
        matrix = sim.getObjectMatrix(sensor, sim.handle_world if relative_to is None else relative_to)
        # fastHokuyo's vision callback caps the RADIAL distance (packet's
        # fourth value) at maxScanDistance, also used as far clipping.
        # Sensor Z alone makes oblique no-return rays look like obstacles.
        hits = points[:, 3] < far * 0.9999
        scans.append((np.asarray(matrix).reshape(3, 4)[:, 3],
                      world_points(matrix, points[:, :3]), hits))
    return scans


class Keyboard:
    """Terminal key repeats refresh a short deadman timer; q exits."""
    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError('Keyboard driving needs a normal interactive terminal.')
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.key, self.until = ' ', 0
        return self

    def command(self):
        while select.select([sys.stdin], [], [], 0)[0]:
            self.key = sys.stdin.read(1).lower()
            self.until = time.monotonic() + 0.3
            if self.key == 'q':
                raise KeyboardInterrupt
        return self.key if time.monotonic() < self.until else ' '

    def __exit__(self, *args):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def steering(scans, position, matrix, heading_offset):
    m = np.asarray(matrix).reshape(3, 4)
    yaw = math.atan2(m[1, 0], m[0, 0]) + heading_offset
    endpoints = np.concatenate([p for _, p, _ in scans])
    delta = endpoints[:, :2] - position[:2]
    distance = np.linalg.norm(delta, axis=1)
    angles = (np.arctan2(delta[:, 1], delta[:, 0])-yaw+np.pi) % (2*np.pi)-np.pi

    def clearance(lo, hi):
        values = distance[(angles > lo) & (angles < hi)]
        if len(values) < 3:
            raise RuntimeError('Missing scan coverage ahead. Check LiDAR orientation or --heading-offset.')
        return float(np.min(values))

    front = clearance(-0.55, 0.55)
    left = clearance(0.15, 1.35)
    right = clearance(-1.35, -0.15)
    return front, left, right


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=180, help='simulation seconds')
    parser.add_argument('--lidar', default=None, help='scanner path; defaults to configured Robot A lidar')
    parser.add_argument('--output', type=Path, default=Path('results/mapping'))
    parser.add_argument('--size', type=float, default=30, help='map width in metres, centred on world zero')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--heading-offset', type=float, default=0, help='robot forward direction relative to local X, degrees')
    parser.add_argument('--scan-only', action='store_true', help='map without commanding movement')
    parser.add_argument('--manual', action='store_true', help='terminal keys: W forward, A/D turn, space stop, Q save/quit')
    parser.add_argument('--resume', type=Path, help='previous map_data.npz to extend in the same scene/world frame')
    args = parser.parse_args()
    if args.duration <= 0 or args.size <= 0 or args.resolution <= 0:
        parser.error('duration, size and resolution must be positive')
    if args.manual and args.scan_only:
        parser.error('--manual and --scan-only cannot be combined')
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client = RemoteAPIClient()
    sim = client.require('sim')
    robot = sim.getObject('/Robot_A')
    motors = [sim.getObject('/Robot_A/leftMotor'), sim.getObject('/Robot_A/rightMotor')]
    config = load_robot()
    lidar = sim.getObject(args.lidar) if args.lidar else resolve_lidar(sim, config)
    sensors = sim.getObjectsInTree(lidar, sim.object_visionsensor_type, 0)
    if len(sensors) != 2:
        raise RuntimeError(f'Expected two fastHokuyo vision sensors under {args.lidar or config.lidar_path}; found {len(sensors)}.')
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError('Stop the simulation first, then run this script without main.py.')
    grid = Grid.load(args.resume) if args.resume else Grid(args.size, args.resolution)
    keyboard = None
    started = False
    next_save = 0
    turn_direction = 1
    turn_until = 0
    samples = 0

    def drive(left, right):
        for handle, velocity in zip(motors, (left, right)):
            sim.setJointTargetVelocity(handle, velocity)

    try:
        if args.manual:
            keyboard = Keyboard().__enter__()
            print('W forward | A left | D right | SPACE stop | Q save and quit. Tap/hold keys; no reverse.')
        sim.setStepping(True)
        drive(0, 0)
        # Keep the second robot still during this mapping session.
        for name in ('leftMotor', 'rightMotor'):
            try:
                handle = sim.getObject('/Robot_B/' + name)
            except Exception:
                continue
            sim.setJointTargetVelocity(handle, 0)
        sim.startSimulation()
        started = True
        sim.step()
        start = sim.getSimulationTime()
        print('Mapping started. Ctrl+C stops and saves. White=free, black=occupied, gray=unknown.')
        while sim.getSimulationState() != sim.simulation_stopped:
            now = sim.getSimulationTime() - start
            if now >= args.duration:
                break
            scans = read_scan(sim, sensors)
            position = np.asarray(sim.getObjectPosition(robot, sim.handle_world))
            matrix = sim.getObjectMatrix(robot, sim.handle_world)
            for origin, points, hits in scans:
                # Thin rays for a responsive remote-API mapping loop.
                grid.update(origin, points[::3], hits[::3])
            samples += 1
            if args.manual:
                key = keyboard.command()
                front, left, right = steering(scans, position, matrix, math.radians(args.heading_offset))
                command = {'w': (1.0, 1.0), 'a': (-0.5, 0.5), 'd': (0.5, -0.5)}.get(key, (0, 0))
                if key == 'w' and front < 0.75:
                    command = (0, 0)
                if min(front, left, right) < 0.4:
                    command = (0, 0)
                drive(*command)
            elif not args.scan_only:
                front, left, right = steering(scans, position, matrix, math.radians(args.heading_offset))
                if front < 0.75 and now >= turn_until:
                    turn_direction = 1 if left >= right else -1
                    turn_until = now + 1.2
                if now < turn_until or front < 0.85:
                    drive(-0.65*turn_direction, 0.65*turn_direction)
                else:
                    # Slow forward motion with a slight exploratory curve.
                    bias = 0.12*math.sin(now / 8)
                    drive(1.2-bias, 1.2+bias)
            if now >= next_save:
                grid.save(args.output)
                print(f'{now:5.1f}s | scans: {samples} | observed cells: {grid.seen.sum()} | {args.output}/map.pgm')
                next_save = now + 5
            sim.step()
            if args.manual:
                time.sleep(0.03)
    except KeyboardInterrupt:
        print('\nStopping and saving...')
    finally:
        if keyboard is not None:
            keyboard.__exit__()
        if started:
            try:
                drive(0, 0)
                sim.stopSimulation()
            finally:
                sim.setStepping(False)
        if samples:
            grid.save(args.output)
            print(f'Map saved in {args.output.resolve()}')
            if grid.clipped:
                print('Some rays exceeded map bounds. Repeat with a larger --size.')


if __name__ == '__main__':
    main()
