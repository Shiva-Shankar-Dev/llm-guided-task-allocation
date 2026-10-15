"""Read a configured robot's starting map coordinates without moving it."""
import argparse
import math
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from .robot import DEFAULT_ROBOTS, load_robot, resolve_model, scene_paths

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='Robot_A')
    parser.add_argument('--robots-config', type=Path, default=DEFAULT_ROBOTS)
    parser.add_argument('--list', action='store_true', help='list robot/scanner and root paths in the open scene')
    args = parser.parse_args()
    config = load_robot(args.robot, args.robots_config)
    client = RemoteAPIClient()
    sim = client.require('sim')
    if args.list:
        print('\n'.join(scene_paths(sim)))
        return
    try:
        robot = resolve_model(sim, config)
    except RuntimeError as exc:
        parser.exit(1, f'{exc}\n')
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError('Stop the simulation first')
    position = sim.getObjectPosition(robot, sim.handle_world)
    matrix = sim.getObjectMatrix(robot, sim.handle_world)
    yaw = math.degrees(math.atan2(matrix[4], matrix[0]))
    print(f'--initial-pose {position[0]:.4f} {position[1]:.4f} {yaw:.3f}')
    print('Use these only for initialization; navigation estimates subsequent poses from wheels and LiDAR.')
    print(f'Robot: {config.name}; navigate with --robot {config.name}')


if __name__ == "__main__":
    main()
