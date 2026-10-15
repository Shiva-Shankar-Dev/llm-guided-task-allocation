import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from robot_navigation.allocation import allocate, interpret, validate_intent
from robot_navigation.map_environment import Grid
from robot_navigation.navigate_map import Map


def intent(*jobs):
    return {'clarification':'','jobs':list(jobs)}


def job(name, locations, robot=None):
    return {'id':name,'locations':locations,'robot':robot}


class AllocationTests(unittest.TestCase):
    def test_single_interpretation_invocation_and_usage_reporting(self):
        import json
        from types import SimpleNamespace
        data=intent(job('visit',['goal']))
        response=SimpleNamespace(text=json.dumps(data),usage_metadata=SimpleNamespace(
            prompt_token_count=100,candidates_token_count=20,total_token_count=120))
        with patch.dict('os.environ',{'GOOGLE_API_KEY':'offline-test-key'}),patch('google.genai.Client') as client:
            api=client.return_value.__enter__.return_value.models.generate_content
            api.return_value=response
            metrics={}
            result=interpret('Visit goal',{'Robot_A'},{'goal':[0,0]},'test-model',metrics)
            api.assert_called_once()
            self.assertEqual(result,data)
            self.assertEqual(metrics['interpretation_sdk_invocations'],1)
            self.assertEqual(metrics['total_tokens'],120)

    def make_map(self, root, wall=False):
        grid=Grid(20,.2);grid.seen[:]=True;grid.odds[:]=-3
        if wall:
            for y in np.arange(-9.8,3.2,.2):
                xcell,ycell=grid.cell([0,y]);grid.odds[ycell,xcell]=3
        grid.save(root)
        return Map(root/'map.yaml')

    def test_route_distance_beats_euclidean_nearest_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            m=self.make_map(Path(directory),wall=True)
            mission,decisions=allocate(intent(job('visit',['goal'])),
                                      {'Robot_A':[-.8,0,0],'Robot_B':[5,0,0]},
                                      {'goal':[1,0]}, {'Robot_A':m,'Robot_B':m})
            self.assertEqual(mission['tasks'][0]['robot'],'Robot_B')
            costs=decisions[0]['candidates']
            self.assertGreater(costs['Robot_A']['travel_m'],costs['Robot_B']['travel_m'])

    def test_queue_cost_explicit_request_and_sequence(self):
        with tempfile.TemporaryDirectory() as directory:
            m=self.make_map(Path(directory))
            maps={'Robot_A':m,'Robot_B':m}
            poses={'Robot_A':[0,0,0],'Robot_B':[8,0,0]}
            stations={'first':[5,0],'second':[3,0]}
            mission,decisions=allocate(intent(job('first',['first']),job('second',['second'])),poses,stations,maps)
            self.assertEqual([t['robot'] for t in mission['tasks']],['Robot_B','Robot_A'])
            self.assertGreater(decisions[1]['candidates']['Robot_B']['queued_m'],0)
            mission,_=allocate(intent(job('ordered',['first','second'],'Robot_A')),poses,stations,maps)
            self.assertEqual([t['station'] for t in mission['tasks']],['first','second'])
            self.assertTrue(all(t['robot']=='Robot_A' for t in mission['tasks']))

    def test_reject_unknown_and_ambiguous_and_unreachable(self):
        for data in [intent(job('x',['missing'])),intent(job('x',['goal'],'Robot_Z')),
                     {'jobs':[],'clarification':'Which station?'},
                     intent(job('same',['goal']),job('same',['goal']))]:
            with self.assertRaises(ValueError):
                validate_intent(data,{'Robot_A'},{'goal':[0,0]})
        with tempfile.TemporaryDirectory() as directory:
            m=self.make_map(Path(directory))
            with self.assertRaisesRegex(ValueError,'cannot be allocated'):
                allocate(intent(job('bad',['outside'])),{'Robot_A':[0,0,0]},
                         {'outside':[100,100]},{'Robot_A':m})

    def test_recognized_peer_return_is_not_cached_as_ghost_obstacle(self):
        from robot_navigation.fleet import Agent
        from robot_navigation.robot import RobotHandles,load_robot
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            m=self.make_map(Path(directory))
            a=Agent(load_robot('Robot_A'),np.array([-1.,0.,0.]),m,
                    handles=RobotHandles(1,[2,3],[4,5]),angles=np.zeros(2),dynamic_until=np.zeros(m.free.shape))
            b=Agent(load_robot('Robot_B'),np.array([1.,0.,0.]),m)
            scans=[(np.zeros(3),np.array([[2,0,0],[1,2,0]]),np.array([True,True]))]
            sim=SimpleNamespace(getJointPosition=lambda handle:0)
            with patch('robot_navigation.fleet.read_scan',return_value=scans),patch('robot_navigation.fleet.localize',return_value=(a.pose.copy(),1.)):
                a.observe(sim,0,{'Robot_B':b})
            self.assertEqual(a.dynamic_until[m.cell([1,0])],0)
            self.assertGreater(a.dynamic_until[m.cell([0,2])],0)


if __name__=='__main__':
    unittest.main()
