import tempfile
from pathlib import Path
import unittest

import numpy as np

from robot_navigation.map_environment import Grid
from robot_navigation.navigate_map import Map, astar, localize, motion_is_safe, odometry, transform


class NavigationTests(unittest.TestCase):
    def test_predicted_motion_stops_before_blocked_cells(self):
        with tempfile.TemporaryDirectory() as d:
            grid = Grid(4, .05)
            grid.seen[:] = True
            grid.odds[:] = -3
            grid.save(Path(d))
            map_ = Map(Path(d)/'map.yaml', .1)
            blocked = map_.base_blocked.copy()
            blocked[map_.cell([.10, 0])] = True
            self.assertFalse(motion_is_safe(map_, blocked, np.zeros(3), .2, 0))
            self.assertTrue(motion_is_safe(map_, blocked, np.zeros(3), 0, .5))

    def test_resume_retains_world_frame_and_evidence(self):
        with tempfile.TemporaryDirectory() as d:
            original = Grid(10, 0.1)
            original.update([0,0], [[2,0]], [True])
            original.save(Path(d))
            resumed = Grid.load(Path(d)/'map_data.npz')
            np.testing.assert_array_equal(resumed.odds, original.odds)
            np.testing.assert_array_equal(resumed.origin, original.origin)
            self.assertEqual(resumed.resolution, original.resolution)

    def test_astar_routes_through_gap_and_rejects_corner_cutting(self):
        blocked = np.zeros((12,12), dtype=bool)
        blocked[:,6] = True
        blocked[9,6] = False
        path = astar(blocked, (2,2), (2,10))
        self.assertIn((9,6), path)
        self.assertTrue(all(not blocked[c] for c in path))
        with self.assertRaises(RuntimeError):
            astar(np.array([[False,True],[True,False]]), (0,0), (1,1))
        blocked[9,6] = True
        with self.assertRaises(RuntimeError):
            astar(blocked, (2,2), (2,10))

    def test_odometry_and_transform(self):
        np.testing.assert_allclose(odometry(np.zeros(3), [0,0], [1,1], .1,.4), [.1,0,0])
        np.testing.assert_allclose(odometry(np.zeros(3), [0,0], [-1,1], .1,.4), [0,0,.5])
        np.testing.assert_allclose(transform(np.array([[1,0]]), [2,3,np.pi/2]), [[2,4]], atol=1e-8)

    def test_pgm_orientation_inflation_and_scan_matching(self):
        with tempfile.TemporaryDirectory() as d:
            grid = Grid(10, .05)
            grid.seen[:] = True
            grid.odds[:] = -3
            points = np.array([[2,y] for y in np.linspace(-2,2,90)] + [[x,2] for x in np.linspace(-2,2,90)])
            for pt in points:
                x,y = grid.cell(pt)
                grid.odds[y,x] = 3
            grid.save(Path(d))
            map_ = Map(Path(d)/'map.yaml', .25)
            self.assertTrue(map_.occupied[map_.cell([2,1])])
            self.assertTrue(map_.base_blocked[map_.cell([1.9,1])])
            self.assertFalse(map_.base_blocked[map_.cell([0,0])])
            predicted = np.array([.07,.07,0])
            pose, score = localize(map_, points, predicted)
            self.assertLess(np.linalg.norm(pose[:2]), np.linalg.norm(predicted[:2]))
            self.assertGreater(score, .8)
            with self.assertRaises(RuntimeError):
                localize(map_, points, np.array([-3,-3,0]))


if __name__ == '__main__':
    unittest.main()
