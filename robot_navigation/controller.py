"""Path following shared by standalone and fleet controllers."""
import math

import numpy as np

from .map_environment import steering
from .navigate_map import motion_is_safe, visible, wrap


def follow_path(map_, blocked, pose, path, scans, speed=0.12, explain=False):
    current = map_.cell(pose[:2])
    if not map_.valid(current) or blocked[current]:
        raise RuntimeError('Estimated robot pose is outside safe map space')
    nearest = min(range(len(path)), key=lambda i: np.linalg.norm(map_.xy(path[i])-pose[:2]))
    target = map_.xy(path[nearest])
    for cell in path[nearest:]:
        if np.linalg.norm(map_.xy(cell)-pose[:2]) > 0.25 or not visible(blocked, current, cell):
            break
        target = map_.xy(cell)
    error = wrap(math.atan2(*(target-pose[:2])[::-1])-pose[2])
    front, left, right = steering(scans, np.zeros(3), [1,0,0,0,0,1,0,0,0,0,1,0], 0)
    omega = float(np.clip(2*error,-0.6,0.6))
    velocity = 0 if abs(error)>0.4 else speed*max(0, math.cos(error))
    reason = 'turning to align with path' if abs(error)>0.4 else ''
    if front < map_.clearance+0.25:
        velocity = 0
        reason = 'LiDAR obstacle ahead: forward clearance below stopping threshold'
    if min(front,left,right) < map_.clearance:
        velocity, omega = 0, 0
        reason = 'LiDAR obstacle too close: full safety stop'
    if velocity and not motion_is_safe(map_, blocked, pose, velocity, omega):
        velocity = 0
        reason = 'predicted turning motion enters map safety margin'
    return (velocity, omega, reason) if explain else (velocity, omega)
