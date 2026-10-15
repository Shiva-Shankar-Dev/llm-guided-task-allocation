from contextlib import redirect_stdout
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from robot_navigation.fleet import Agent, main, read_mission, robot_obstacles, separate_commands
from robot_navigation.map_environment import Grid
from robot_navigation.navigate_map import Map
from robot_navigation.robot import load_robot


class FakeSim:
    simulation_stopped=0
    object_visionsensor_type=9
    def __init__(self):
        self.state=0
        self.time=0
        self.steps=0
        self.paths={}
        self.speeds={}
        self.written=set()
        self.stepping=False
    def getSimulationState(self): return self.state
    def getSimulationTime(self): return self.time
    def getObject(self,path):
        if path not in self.paths:
            self.paths[path]=len(self.paths)+1
        return self.paths[path]
    def getObjectsInTree(self,*args): return [100,101]
    def setStepping(self,value): self.stepping=value
    def setJointTargetVelocity(self,motor,speed):
        self.speeds[motor]=speed
        self.written.add(motor)
    def getJointPosition(self,motor): return 0
    def startSimulation(self): self.state=1
    def stopSimulation(self): self.state=0
    def step(self):
        # Both robots must receive commands before the single shared step.
        assert len(self.written)==4
        self.written.clear()
        self.time+=.1
        self.steps+=1


class FleetTests(unittest.TestCase):
    def test_predicted_collision_stops_both_and_open_paths_continue(self):
        a=SimpleNamespace(config=load_robot('Robot_A'),pose=np.array([-0.6,0.,0.]))
        b=SimpleNamespace(config=load_robot('Robot_B'),pose=np.array([0.6,0.,np.pi]))
        commands={'Robot_A':(.12,0),'Robot_B':(.12,0)}
        self.assertEqual(separate_commands({'Robot_A':a,'Robot_B':b},commands),
                         {'Robot_A':(0.,0.),'Robot_B':(0.,0.)})
        _, reasons=separate_commands({'Robot_A':a,'Robot_B':b},commands,explain=True)
        self.assertIn('Robot_B', reasons['Robot_A'])
        self.assertIn('Robot_A', reasons['Robot_B'])
        b.pose[1]=2
        self.assertEqual(separate_commands({'Robot_A':a,'Robot_B':b},commands),commands)

    def test_wait_reasons_distinguish_robot_and_lidar_planning_layers(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            grid=Grid(10,.1); grid.seen[:]=True; grid.odds[:]=-3; grid.save(root)
            map_=Map(root/'map.yaml')
            a=Agent(load_robot('Robot_A'),np.array([-1.,0.,0.]),map_)
            b=Agent(load_robot('Robot_B'),np.array([1.,0.,0.]),map_)
            a.active={'goal':[1.,0.]}
            a.dynamic_until=np.zeros(map_.free.shape)
            self.assertEqual(a.propose(0,{'Robot_A':a,'Robot_B':b},.2),(0.,0.))
            self.assertIn('robot footprint',a.control_reason)
            self.assertIn('Robot_B',a.control_reason)
            a.dynamic_until[map_.cell([1,0])]=10
            self.assertEqual(a.propose(1,{'Robot_A':a,'Robot_B':b},.2),(0.,0.))
            self.assertIn('live LiDAR',a.control_reason)

    def test_sequential_queues_complete_in_one_loop_and_failure_stops_every_motor(self):
        for inject_failure in (False,True):
            with self.subTest(failure=inject_failure), tempfile.TemporaryDirectory() as directory:
                root=Path(directory)
                grid=Grid(10,.1)
                grid.seen[:]=True
                grid.odds[:]=-3
                grid.save(root)
                mission={'robots':{'Robot_A':{'initial_pose':[-1,0,0]},'Robot_B':{'initial_pose':[1,0,0]}},
                         'tasks':[{'id':'a1','robot':'Robot_A','goal':[-1,1]},
                                  {'id':'a2','robot':'Robot_A','goal':[-1,2]},
                                  {'id':'b1','robot':'Robot_B','goal':[1,1]}]}
                (root/'mission.json').write_text(json.dumps(mission))
                sim=FakeSim()
                def observe(agent, sim, now, states):
                    if inject_failure and agent.config.name=='Robot_B':
                        raise RuntimeError('sensor failure')
                    agent.pose[1]=now
                argv=['run_fleet.py','--mission',str(root/'mission.json'),'--map',str(root/'map.yaml'),
                      '--output',str(root/'output')]
                with patch('sys.argv',argv), patch('coppeliasim_zmqremoteapi_client.RemoteAPIClient',return_value=SimpleNamespace(require=lambda name:sim)), \
                     patch.object(Agent,'observe',observe), patch.object(Agent,'propose',return_value=(.1,0)), redirect_stdout(io.StringIO()):
                    if inject_failure:
                        with self.assertRaisesRegex(SystemExit,'sensor failure'):
                            main()
                    else:
                        main()
                self.assertEqual(sim.state,0)
                self.assertFalse(sim.stepping)
                self.assertTrue(all(v==0 for v in sim.speeds.values()))
                tasks=json.loads((root/'output/tasks.json').read_text())
                self.assertTrue(all(t['status']==('failed' if inject_failure else 'completed') for t in tasks))
                events=[json.loads(line) for line in (root/'output/events.jsonl').read_text().splitlines()]
                if not inject_failure:
                    a1_done=next(i for i,e in enumerate(events) if e['task']=='a1' and e['status']=='completed')
                    a2_start=next(i for i,e in enumerate(events) if e['task']=='a2' and e['status']=='running')
                    self.assertLess(a1_done,a2_start)

    def test_robot_footprint_blocks_map_and_duplicate_tasks_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            grid=Grid(10,.1);grid.save(root)
            map_=Map(root/'map.yaml')
            state=SimpleNamespace(config=load_robot('Robot_B'),pose=np.array([1.,1.,0.]))
            mask=robot_obstacles(map_,'Robot_A',{'Robot_B':state})
            self.assertTrue(mask[map_.cell([1,1])])
            data={'robots':{'Robot_A':{'initial_pose':[0,0,0]}},
                  'tasks':[{'id':'same','robot':'Robot_A','goal':[1,1]}]*2}
            (root/'mission.json').write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'unique'):
                read_mission(root/'mission.json')


if __name__=='__main__':
    unittest.main()
