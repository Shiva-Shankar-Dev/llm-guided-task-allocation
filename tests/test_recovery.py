from contextlib import redirect_stdout
from dataclasses import replace
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from robot_navigation.fleet import Agent,Journal,main,recover_robot
from robot_navigation.map_environment import Grid
from robot_navigation.navigate_map import Map
from robot_navigation.robot import load_robot
from tests.test_fleet import FakeSim


class RecoveryTests(unittest.TestCase):
    def fixture(self,root):
        grid=Grid(20,.1);grid.seen[:]=True;grid.odds[:]=-3;grid.save(root)
        m=Map(root/'map.yaml')
        states={}
        for name,pose in [('Robot_A',[-3,0,0]),('Robot_B',[0,-3,0]),('Robot_C',[4,0,0])]:
            config=replace(load_robot('Robot_A'),name=name)
            states[name]=Agent(config,np.array(pose,dtype=float),m,dynamic_until=np.zeros(m.free.shape))
        return states

    def test_distance_selection_order_and_no_reassignment_loop(self):
        with tempfile.TemporaryDirectory() as directory,redirect_stdout(io.StringIO()):
            root=Path(directory);states=self.fixture(root)
            tasks=[{'id':'a1','robot':'Robot_A','goal':[3,0],'status':'running'},
                   {'id':'a2','robot':'Robot_A','goal':[3,2],'status':'pending'}]
            states['Robot_A'].active=tasks[0]
            journal=Journal(root,tasks)
            selected=recover_robot('Robot_A',states,tasks,1,journal)
            self.assertEqual(selected,'Robot_C')
            self.assertTrue(states['Robot_A'].unavailable)
            self.assertIsNone(states['Robot_A'].active)
            self.assertEqual([t['id'] for t in tasks],['a1','a2'])
            self.assertTrue(all(t['robot']=='Robot_C' and t['status']=='pending' for t in tasks))
            states['Robot_C'].active=tasks[0];tasks[0]['status']='running'
            self.assertIsNone(recover_robot('Robot_C',states,tasks,2,journal,max_reassignments=1))
            self.assertTrue(all(t['status']=='failed' for t in tasks))
            self.assertEqual(tasks[0]['original_robot'],'Robot_A')
            self.assertEqual(len(tasks[0]['attempt_history']),2)
            journal.close()

    def test_closed_loop_replacement_finishes_task_in_shared_loop(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            grid=Grid(10,.1);grid.seen[:]=True;grid.odds[:]=-3;grid.save(root)
            mission={'robots':{'Robot_A':{'initial_pose':[-1,0,0]},'Robot_B':{'initial_pose':[1,0,0]}},
                     'tasks':[{'id':'replace_me','robot':'Robot_A','goal':[-1,2]},
                              {'id':'b_first','robot':'Robot_B','goal':[1,1]}]}
            (root/'mission.json').write_text(json.dumps(mission))
            sim=FakeSim()
            def observe(agent,sim,now,states):
                if agent.config.name=='Robot_B' and agent.active:
                    delta=np.asarray(agent.active['goal'])-agent.pose[:2]
                    norm=np.linalg.norm(delta)
                    if norm:
                        agent.pose[:2]+=delta*min(1,.2/norm)
            argv=['--mission',str(root/'mission.json'),'--map',str(root/'map.yaml'),
                  '--output',str(root/'output'),'--simulate-stall','Robot_A','--stall-timeout','.5']
            with patch('google.genai.Client',side_effect=AssertionError('Recovery must not call an LLM')) as llm, \
                 patch('coppeliasim_zmqremoteapi_client.RemoteAPIClient',return_value=SimpleNamespace(require=lambda name:sim)), \
                 patch.object(Agent,'observe',observe),patch.object(Agent,'propose',return_value=(.1,0)),redirect_stdout(io.StringIO()):
                main(argv)
            llm.assert_not_called()
            tasks=json.loads((root/'output/tasks.json').read_text())
            self.assertTrue(all(t['status']=='completed' for t in tasks))
            recovered=next(t for t in tasks if t['id']=='replace_me')
            self.assertEqual(recovered['robot'],'Robot_B')
            self.assertEqual(recovered['reassignments'],1)
            self.assertTrue(all(v==0 for v in sim.speeds.values()))
            self.assertFalse(sim.stepping)


if __name__=='__main__':
    unittest.main()
