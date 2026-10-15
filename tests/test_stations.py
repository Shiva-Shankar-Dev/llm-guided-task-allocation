import tempfile
from pathlib import Path
import unittest
import numpy as np

from robot_navigation.allocation import allocate
from robot_navigation.stations import canonical_instruction, normalize_stations
from tests import test_allocation as allocation_tests
from tests.test_allocation import intent, job


class StationTests(unittest.TestCase):
    def test_explicit_aliases_and_unknown_names(self):
        stations={'loading_zone': {'aliases':['loading dock','loading bay'], 'slots':{'one':[1,1]}}}
        self.assertEqual(canonical_instruction('Both robots to Loading Dock.', stations),
                         'Both robots to loading_zone.')
        self.assertEqual(canonical_instruction('Visit airport', stations), 'Visit airport')
        with self.assertRaisesRegex(ValueError, 'Ambiguous'):
            normalize_stations({**stations, 'other':{'aliases':['loading dock'],'slots':{'x':[2,2]}}})

    def test_two_robots_reserve_separate_slots_and_capacity_is_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            m=allocation_tests.AllocationTests().make_map(Path(directory))
            poses={'Robot_A':[-2,0,0],'Robot_B':[2,0,0]}
            maps={n:m for n in poses}
            stations={'loading_zone':{'aliases':['loading dock'],
                                      'slots':{'west':[-1,3],'east':[1,3]}}}
            request=intent(job('a',['loading dock'],'Robot_A'),job('b',['loading_zone'],'Robot_B'))
            mission,_=allocate(request,poses,stations,maps)
            a,b=mission['tasks']
            self.assertNotEqual(a['slot'],b['slot'])
            self.assertEqual(a['station'],b['station'])
            self.assertGreater(np.linalg.norm(np.asarray(a['goal'])-b['goal']), 2*m.clearance+.45)
            stations['loading_zone']['slots'].pop('east')
            with self.assertRaisesRegex(ValueError, 'capacity'):
                allocate(request,poses,stations,maps)

    def test_parked_robot_slot_is_not_reused_next_batch(self):
        with tempfile.TemporaryDirectory() as directory:
            m=allocation_tests.AllocationTests().make_map(Path(directory))
            poses={'Robot_A':[-1,3,0],'Robot_B':[2,0,0]}
            maps={n:m for n in poses}
            stations={'dock':{'slots':{'west':[-1,3],'east':[1,3]}}}
            mission,_=allocate(intent(job('b',['dock'],'Robot_B')),poses,stations,maps,occupied_poses=poses)
            self.assertEqual(mission['tasks'][0]['slot'],'east')
