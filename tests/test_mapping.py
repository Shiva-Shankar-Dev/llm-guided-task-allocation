"""Offline regression checks: python -m unittest test_mapping."""
import unittest

import numpy as np

from robot_navigation.map_environment import Grid, read_scan


class HokuyoStub:
    visionfloatparam_far_clipping = 1
    handle_world = -1

    def readVisionSensor(self, sensor):
        # Angled no-return ray: Z < 5 although its radial range is 5.
        # Second ray is a real obstacle at radial range 2.
        return 0, [0]*15, [2, 1, 4, 0, 3, 5, 1.6, 0, 1.2, 2]

    def getObjectFloatParam(self, sensor, param):
        return 5

    def getObjectMatrix(self, sensor, relative):
        return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]


class MappingTests(unittest.TestCase):
    def test_max_range_ray_is_free_but_obstacle_is_occupied(self):
        origin, points, hits = read_scan(HokuyoStub(), [1])[0]
        np.testing.assert_array_equal(hits, [False, True])
        grid = Grid(12, 0.1)
        grid.update(origin, points, hits)
        x, y = grid.cell(points[0])
        self.assertLess(grid.odds[y, x], 0)
        x, y = grid.cell(points[1])
        self.assertGreater(grid.odds[y, x], 0)


if __name__ == '__main__':
    unittest.main()
