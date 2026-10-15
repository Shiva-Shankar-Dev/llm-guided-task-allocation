from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from tests.test_fleet import FakeSim
from robot_navigation.fleet import Agent, main, read_mission
from robot_navigation.paths import PROJECT_ROOT, DEFAULT_MAP
from robot_navigation.allocation import allocate
from robot_navigation.navigate_map import Map


class FiveSim(FakeSim):
    def __init__(self, configs):
        super().__init__()
        self.configs=configs
        self.poses={n:p.copy() for n,(c,p) in configs.items()}
        self.concurrent=False
    def step(self):
        assert len(self.written)==10
        self.written.clear()
        moving=0
        for n,(c,_) in self.configs.items():
            left=self.speeds[self.paths[c.left_motor]]*c.wheel_radius
            right=self.speeds[self.paths[c.right_motor]]*c.wheel_radius
            v=(left+right)/2
            w=(right-left)/c.track
            p=self.poses[n]
            p[:2]+=.1*v*np.array([math.cos(p[2]+w*.05),math.sin(p[2]+w*.05)])
            p[2]+=w*.1
            moving+=abs(v)>.01
        self.concurrent |= moving==5
        self.time+=.1
        self.steps+=1


class FiveFleetTests(unittest.TestCase):
    def run_trial(self,speed=None,traffic=False):
        configs,_=read_mission(PROJECT_ROOT/'config/fleet_five.json')
        sim=FiveSim(configs)
        def observe(agent,sim,now,states):
            agent.pose=sim.poses[agent.config.name].copy()
            agent.score=1
        def stopped(agent,now,states,margin):
            agent.control_reason='robot footprint blocks route: test peer'
            return (0.,0.)
        with tempfile.TemporaryDirectory() as directory:
            args=['--mission',str(PROJECT_ROOT/'config/fleet_five.json'),'--output',directory]
            if speed: args+=['--speed',str(speed)]
            if traffic: args+=['--stall-timeout','.2','--traffic-timeout','.6']
            with patch('coppeliasim_zmqremoteapi_client.RemoteAPIClient',return_value=SimpleNamespace(require=lambda n:sim)), \
                 patch.object(Agent,'observe',observe), \
                 patch('robot_navigation.controller.steering',return_value=(10,10,10)), redirect_stdout(io.StringIO()):
                if traffic:
                    with patch.object(Agent,'propose',stopped),self.assertRaisesRegex(SystemExit,'Traffic deadlock'):
                        main(args)
                else:
                    main(args)
            tasks=json.loads((Path(directory)/'tasks.json').read_text())
            if traffic:
                self.assertTrue(all(not t.get('attempt_history') for t in tasks))
            else:
                self.assertTrue(all(t['status']=='completed' for t in tasks))
                self.assertTrue(sim.concurrent)
            self.assertTrue(all(v==0 for v in sim.speeds.values()))
            return sim.time

    def test_default_is_faster_and_all_five_move_concurrently(self):
        slow=self.run_trial(.12)
        fast=self.run_trial()
        self.assertLess(fast,slow)
        print(f'Five-robot kinematic smoke test: 0.12 m/s={slow:.1f}s; default 0.18 m/s={fast:.1f}s')

    def test_traffic_wait_does_not_quarantine_healthy_robots(self):
        self.run_trial(traffic=True)

    def test_five_loading_slots_allocate_distinct_goals(self):
        configs,_=read_mission(PROJECT_ROOT/'config/fleet_five.json')
        stations=json.loads((PROJECT_ROOT/'config/stations_five.json').read_text())
        poses={n:p.tolist() for n,(c,p) in configs.items()}
        maps={n:Map(DEFAULT_MAP,c.clearance) for n,(c,p) in configs.items()}
        intent={'clarification':'','jobs':[{'id':n,'robot':n,'locations':['loading dock']} for n in poses]}
        mission,_=allocate(intent,poses,stations,maps,occupied_poses=poses)
        self.assertEqual(len({t['slot'] for t in mission['tasks']}),5)
        self.assertEqual(len({t['robot'] for t in mission['tasks']}),5)
