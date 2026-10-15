import io
import unittest
from contextlib import redirect_stderr
from unittest.mock import patch
import numpy as np
from robot_navigation.fleet import Agent, main
from robot_navigation.navigate_map import Map
from robot_navigation.paths import DEFAULT_MAP
from robot_navigation.robot import load_robot, wheel_speeds


class SpeedTests(unittest.TestCase):
    def test_invalid_speed_rejected_before_simulator_connection(self):
        for speed in ('0','-1','nan','inf','.5'):
            with self.subTest(speed=speed), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(['--speed',speed])

    def test_cruise_setting_and_recovery_speed_remain_separate(self):
        m=Map(DEFAULT_MAP)
        a=Agent(load_robot('Robot_A'),np.array([0.,5.,0.]),m,cruise_speed=.18)
        a.active={'goal':[1,5]}
        a.last_plan=0
        a.blocked=m.inflate(~m.free,.2)
        a.path=[m.cell(a.pose[:2])]
        with patch('robot_navigation.fleet.follow_path',return_value=(.1,0,'')) as follow:
            a.propose(.1,{'Robot_A':a},.2)
            self.assertEqual(follow.call_args.kwargs['speed'],.18)
            a.escaping_margin=True
            a.propose(.1,{'Robot_A':a},.2)
            self.assertEqual(follow.call_args.kwargs['speed'],.04)
        self.assertLessEqual(max(abs(v) for v in wheel_speeds(a.config,.18,.6)),2.)
