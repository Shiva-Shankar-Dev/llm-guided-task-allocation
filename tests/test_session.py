from contextlib import redirect_stdout
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from robot_navigation.fleet import Agent, main
from robot_navigation.map_environment import Grid
from tests.test_fleet import FakeSim


class SessionTests(unittest.TestCase):
    def run_session(self, fail=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            grid = Grid(10, .1)
            grid.seen[:] = True
            grid.odds[:] = -3
            grid.save(root)
            mission = {'robots': {'Robot_A': {'initial_pose': [-1, 0, 0]},
                                  'Robot_B': {'initial_pose': [1, 0, 0]}},
                       'tasks': [{'id': 'ignored', 'robot': 'Robot_A', 'goal': [-1, 3]}]}
            (root/'mission.json').write_text(json.dumps(mission))
            (root/'stations.json').write_text(json.dumps({'first': [-1, 1], 'second': [-1, 2]}))
            intent = {'clarification': '', 'jobs': [{'id': 'visit', 'robot': 'Robot_A', 'locations': ['first']}]}
            (root/'intent.json').write_text(json.dumps(intent))
            sim = FakeSim()
            starts, stops, prompts = [], [], []
            original_start, original_stop = sim.startSimulation, sim.stopSimulation
            def start():
                starts.append(sim.time)
                original_start()
            def stop():
                stops.append(sim.time)
                original_stop()
            sim.startSimulation, sim.stopSimulation = start, stop
            commands = iter(['malformed instruction', f'/intent {root}/intent.json',
                             'Visit second', '/quit'])
            def input_(prompt):
                self.assertEqual(sim.state, 1)
                self.assertTrue(sim.stepping)
                self.assertTrue(all(v == 0 for v in sim.speeds.values()))
                prompts.append(sim.time)
                return next(commands)
            def observe(agent, sim, now, states):
                if fail:
                    raise RuntimeError('lost lidar')
                if agent.active is not None:
                    agent.pose[:2] = agent.active['goal']
            def interpret(text, names, stations, model, metrics):
                metrics['interpretation_sdk_invocations'] = 1
                if text == 'malformed instruction':
                    raise ValueError('ambiguous instruction')
                return {'clarification': '', 'jobs': [{'id': 'visit', 'robot': 'Robot_A', 'locations': ['second']}]}
            args = ['--interactive', '--mission', str(root/'mission.json'), '--map', str(root/'map.yaml'),
                    '--stations', str(root/'stations.json'), '--output', str(root/'out')]
            with patch('builtins.input', input_), \
                 patch('robot_navigation.allocation.interpret', side_effect=interpret) as llm, \
                 patch('coppeliasim_zmqremoteapi_client.RemoteAPIClient', return_value=SimpleNamespace(require=lambda n: sim)), \
                 patch.object(Agent, 'observe', observe), patch.object(Agent, 'propose', return_value=(.1, 0)), \
                 redirect_stdout(io.StringIO()):
                if fail:
                    with self.assertRaisesRegex(SystemExit, 'lost lidar'):
                        main(args)
                else:
                    main(args)
                    self.assertEqual(llm.call_count, 2)  # one rejected, one valid; offline batch is zero
            self.assertEqual(len(starts), 1)
            self.assertEqual(len(stops), 1)  # only exit, never between batches
            self.assertEqual(sim.state, 0)
            self.assertFalse(sim.stepping)
            self.assertTrue(all(v == 0 for v in sim.speeds.values()))
            if not fail:
                tasks = json.loads((root/'out/tasks.json').read_text())
                self.assertEqual([t['status'] for t in tasks], ['completed', 'completed'])
                self.assertEqual(len({t['id'] for t in tasks}), 2)
                second = json.loads((root/'out/batch_003/mission.json').read_text())
                self.assertEqual(second['robots']['Robot_A']['initial_pose'][:2], [-1, 1])
                self.assertEqual(prompts[0], prompts[1])  # API rejection advances no physics
                usage = json.loads((root/'out/batch_002/llm_usage.json').read_text())
                self.assertEqual(usage['interpretation_sdk_invocations'], 0)

    def test_repeated_batches_keep_pose_and_reject_bad_input_without_reset(self):
        self.run_session()

    def test_sensing_failure_stops_session(self):
        self.run_session(fail=True)
