"""Read Robot A's starting map coordinates without moving it."""
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    robot = sim.getObject('/Robot_A')
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError('Stop the simulation first')
    position = sim.getObjectPosition(robot, sim.handle_world)
    matrix = sim.getObjectMatrix(robot, sim.handle_world)
    yaw = math.degrees(math.atan2(matrix[4], matrix[0]))
    print(f'--initial-pose {position[0]:.4f} {position[1]:.4f} {yaw:.3f}')
    print('Use these only for initialization; navigation estimates subsequent poses from wheels and LiDAR.')


if __name__ == "__main__":
    main()
