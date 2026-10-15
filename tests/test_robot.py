import unittest
from dataclasses import replace

from robot_navigation.robot import RobotHandles, load_robot, resolve_lidar, resolve_model, wheel_speeds


class RobotTests(unittest.TestCase):
    def test_missing_robot_is_not_replaced_with_another_robot(self):
        class Sim:
            handle_scene, handle_all = -1, -2
            def getObject(self, path):
                if path == '/Robot_A':
                    return 1
                raise Exception('missing')
            def getObjectsInTree(self, *args):
                return [1]
            def getObjectAlias(self, handle, options=-1):
                return '/Robot_A' if options == 2 else 'Robot_A'
            def getObjectParent(self, handle):
                return -1
        with self.assertRaisesRegex(RuntimeError, 'missing from the OPEN scene'):
            resolve_model(Sim(), load_robot('Robot_B'))

    def test_lidar_rename_and_legacy_fallback(self):
        class Sim:
            def getObject(self, path):
                if path == '/Robot_B/fastHokuyo':
                    return 7
                raise Exception('missing')
        with self.assertWarnsRegex(UserWarning, 'using existing'):
            self.assertEqual(resolve_lidar(Sim(), load_robot('Robot_B')), 7)

    def test_robot_specific_paths_and_defaults(self):
        a, b = load_robot(), load_robot('Robot_B')
        self.assertEqual(a.model_path, '/Robot_A')
        self.assertEqual(b.lidar_path, '/Robot_B/lidar')
        self.assertEqual(a.wheel_radius, b.wheel_radius)
        self.assertEqual(load_robot('Robot_B', track=.5).track, .5)
        with self.assertRaises(ValueError):
            load_robot('Robot_Z')
        for value in [0, -1, float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                replace(a, wheel_radius=value)

    def test_wheel_commands_preserve_curvature_at_speed_limit(self):
        cfg = load_robot()
        left, right = wheel_speeds(cfg, .1, 0)
        self.assertAlmostEqual(left, right)
        left, right = wheel_speeds(cfg, 0, .5)
        self.assertAlmostEqual(left, -right)
        self.assertLess(left, 0)
        self.assertLessEqual(max(abs(v) for v in wheel_speeds(cfg, 5, 2)), 2)

    def test_handles_resolve_only_selected_robot(self):
        class Sim:
            object_visionsensor_type = 9
            def __init__(self):
                self.paths = []
            def getObject(self, path):
                self.paths.append(path)
                return len(self.paths)
            def getObjectsInTree(self, model, object_type, options):
                return [10,11]
        sim = Sim()
        handles = RobotHandles.resolve(sim, load_robot('Robot_B'))
        self.assertEqual(len(handles.sensors), 2)
        self.assertTrue(all(p.startswith('/Robot_B/') or p == '/Robot_B' for p in sim.paths))


if __name__ == '__main__':
    unittest.main()
