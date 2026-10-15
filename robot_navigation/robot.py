"""Robot configuration and simulator access shared by CLI tools."""
from dataclasses import dataclass, replace
import json
import math
import warnings
from pathlib import Path

from .paths import PROJECT_ROOT

DEFAULT_ROBOTS = PROJECT_ROOT / 'config' / 'robots.json'


def scene_paths(sim):
    """List roots and robot/scanner paths without modifying the scene."""
    paths = []
    for handle in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        alias = sim.getObjectAlias(handle)
        if sim.getObjectParent(handle) == -1 or any(s in alias.lower() for s in ('robot', 'pioneer', 'lidar', 'hokuyo')):
            paths.append(sim.getObjectAlias(handle, 2))
    return sorted(paths)


def resolve_model(sim, config):
    try:
        return sim.getObject(config.model_path)
    except Exception as exc:
        paths = '\n  '.join(scene_paths(sim))
        raise RuntimeError(f'{config.name}: robot {config.model_path} is missing from the OPEN scene. '
                           'Open the scene containing that robot, add/name the robot in CoppeliaSim, '
                           'or correct model_path and child paths in config/robots.json. '
                           f'Renaming the LiDAR cannot fix a missing robot.\nScene paths:\n  {paths}') from exc


def resolve_lidar(sim, config):
    try:
        return sim.getObject(config.lidar_path)
    except Exception as exc:
        # Support both the newly requested name and the original model alias.
        if config.lidar_path == config.model_path + '/lidar':
            old_path = config.model_path + '/fastHokuyo'
            try:
                handle = sim.getObject(old_path)
            except Exception:
                pass
            else:
                warnings.warn(f'{config.name}: {config.lidar_path} not found; using existing {old_path}. '
                              'Rename this model to lidar in CoppeliaSim and save the scene.', stacklevel=2)
                return handle
        raise RuntimeError(f'{config.name}: no LiDAR at {config.lidar_path}. Check its alias and parent in CoppeliaSim.') from exc


@dataclass(frozen=True)
class RobotConfig:
    name: str
    model_path: str
    left_motor: str
    right_motor: str
    lidar_path: str
    wheel_radius: float
    track: float
    clearance: float

    def __post_init__(self):
        for field in ('model_path', 'left_motor', 'right_motor', 'lidar_path'):
            value = getattr(self, field)
            if not isinstance(value, str) or not value.startswith('/'):
                raise ValueError(f'{self.name}: {field} must be an absolute scene path')
        if self.left_motor == self.right_motor:
            raise ValueError(f'{self.name}: motor paths must be distinct')
        for field in ('wheel_radius', 'track', 'clearance'):
            value = getattr(self, field)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{self.name}: {field} must be a positive finite number')


def load_robot(name='Robot_A', filename=DEFAULT_ROBOTS, **overrides):
    data = json.loads(Path(filename).read_text())
    if name not in data:
        raise ValueError(f'Unknown robot {name!r}. Configured robots: {", ".join(data)}')
    config = RobotConfig(name=name, **data[name])
    return replace(config, **{k:v for k,v in overrides.items() if v is not None})


@dataclass
class RobotHandles:
    model: int
    motors: list
    sensors: list

    @classmethod
    def resolve(cls, sim, config):
        try:
            model = resolve_model(sim, config)
            motors = [sim.getObject(config.left_motor), sim.getObject(config.right_motor)]
            lidar = resolve_lidar(sim, config)
            sensors = sim.getObjectsInTree(lidar, sim.object_visionsensor_type, 0)
        except Exception as exc:
            if isinstance(exc, RuntimeError):
                raise
            raise RuntimeError(f'{config.name}: check model/motor/LiDAR paths in config/robots.json. '
                               f'Expected scanner at {config.lidar_path}.') from exc
        if len(sensors) != 2:
            raise RuntimeError(f'{config.name}: expected two fastHokuyo vision sensors, found {len(sensors)}')
        return cls(model, motors, sensors)


def wheel_speeds(config, velocity, omega, limit=2.0):
    speeds = [(velocity-omega*config.track/2)/config.wheel_radius,
              (velocity+omega*config.track/2)/config.wheel_radius]
    scale = min(1.0, limit/max(max(abs(v) for v in speeds), 1e-6))
    return [v*scale for v in speeds]
