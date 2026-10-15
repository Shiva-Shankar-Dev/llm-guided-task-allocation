import unittest
import numpy as np
from scipy.ndimage import distance_transform_edt
from robot_navigation.fleet import margin_escape
from robot_navigation.navigate_map import Map
from robot_navigation.paths import DEFAULT_MAP


class MarginRecoveryTests(unittest.TestCase):
    def test_recorded_robot_b_pose_can_exit_margin_without_losing_clearance(self):
        m=Map(DEFAULT_MAP)
        pose=np.array([.5135292518819242,3.0200532731073433,1.5419802430751073])
        occupied=~m.free
        full=m.inflate(occupied,.2)
        self.assertTrue(full[m.cell(pose[:2])])
        result=margin_escape(m,occupied,pose,.2)
        self.assertIsNotNone(result)
        blocked,path=result
        distance=distance_transform_edt(np.pad(~occupied,1,constant_values=False))[1:-1,1:-1]*m.resolution
        self.assertFalse(full[path[-1]])
        for cell in path:
            self.assertFalse(blocked[cell])
            self.assertGreaterEqual(distance[cell]+1e-9,distance[path[0]])
            self.assertLessEqual(np.linalg.norm(np.array(cell)-path[0])*m.resolution,.3+1e-9)

    def test_occupied_start_and_already_safe_pose_do_not_escape(self):
        m=Map(DEFAULT_MAP)
        occupied=~m.free
        bad=tuple(np.argwhere(m.base_blocked)[0])
        self.assertIsNone(margin_escape(m,occupied,np.r_[m.xy(bad),0],.2))
        good=tuple(np.argwhere(~m.inflate(occupied,.2))[0])
        self.assertIsNone(margin_escape(m,occupied,np.r_[m.xy(good),0],.2))

    def test_live_obstacle_prevents_escape(self):
        m=Map(DEFAULT_MAP)
        pose=np.array([.5135292518819242,3.0200532731073433,0])
        occupied=(~m.free).copy()
        occupied[m.cell(pose[:2])]=True
        self.assertIsNone(margin_escape(m,occupied,pose,.2))
