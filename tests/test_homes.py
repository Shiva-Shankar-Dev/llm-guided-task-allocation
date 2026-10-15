from contextlib import redirect_stdout
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from robot_navigation.session import ConsoleSession
from robot_navigation.stations import canonical_instruction, canonical_robots
from robot_navigation.allocation import interpret
from robot_navigation.paths import PROJECT_ROOT


class HomeTests(unittest.TestCase):
    def test_aliases_are_stable_when_robot_a_is_unavailable(self):
        self.assertEqual(canonical_robots('Robot 1 and Robot 2', ['Robot_B']), 'Robot 1 and Robot_B')
        stations=json.loads((PROJECT_ROOT/'config/stations_five.json').read_text())
        text=canonical_robots(canonical_instruction('Robot 1 to charging and Robot 2 to Loading',stations),['Robot_A','Robot_B'])
        self.assertEqual(text,'Robot_A to charging_station and Robot_B to loading_zone')

    def test_homes_are_captured_once_and_supplied_to_interpreter(self):
        with tempfile.TemporaryDirectory() as directory,redirect_stdout(io.StringIO()):
            root=Path(directory)
            args=SimpleNamespace(model='test',stations=PROJECT_ROOT/'config/stations.json',output=root)
            state=SimpleNamespace(pose=np.array([1.,2.,.3]))
            session=ConsoleSession(args,{'Robot_A':state})
            state.pose[:2]=[5,6]
            session.register_homes({'Robot_A':state})
            self.assertEqual(session.stations['Robot_A_home']['slots']['home'],[1.,2.])
            self.assertEqual(json.loads((root/'home_positions.json').read_text()),{'Robot_A':[1,2]})
            response=SimpleNamespace(text=json.dumps({'clarification':'','jobs':[{'id':'home','robot':'Robot_A','locations':['Robot_A_home']}]}),usage_metadata=None)
            with patch.dict('os.environ',{'GOOGLE_API_KEY':'test'}),patch('google.genai.Client') as client:
                api=client.return_value.__enter__.return_value.models.generate_content
                api.return_value=response
                interpret('Robot 1 return to its starting point',['Robot_A'],session.stations,'test')
                prompt=api.call_args.kwargs['contents']
                self.assertIn('Robot_A_home',prompt)
                self.assertIn('Robot_A return to its starting point',prompt)
                self.assertNotIn('[1.0, 2.0]',prompt)  # Coordinates remain local.
