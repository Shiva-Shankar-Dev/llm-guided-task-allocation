"""Interactive batches sharing one simulation and continuous pose estimates."""
import json
import math
import os
from pathlib import Path

import numpy as np

from .fleet import read_mission, vector
from .stations import normalize_stations


class ConsoleSession:
    def __init__(self, args, initial_states=None):
        self.args = args
        self.model = args.model or os.environ.get('GEMINI_MODEL')
        self.stations = json.loads(args.stations.read_text())
        if not isinstance(self.stations, dict) or not self.stations:
            raise ValueError('Stations must be a nonempty object')
        self.stations = normalize_stations(self.stations)
        self.homes_registered = False
        if initial_states is not None:
            self.register_homes(initial_states)
        self.number = 0
        print('\nPersistent fleet session. Robots hold position between batches.')
        print('Type an instruction, /status, /stations, /intent FILE (offline), or /quit.')
        print('/quit or Ctrl+C ends the session and stops/resets the simulation.')

    def register_homes(self, states):
        if self.homes_registered:
            return
        for name, state in states.items():
            home=name+'_home'
            if home in self.stations:
                raise ValueError(f'{home} is reserved for the session starting position')
            self.stations[home]={'aliases': [name.replace('_',' ')+' home'],
                                 'slots': {'home': state.pose[:2].tolist()}}
        self.homes_registered=True
        (self.args.output/'home_positions.json').write_text(json.dumps(
            {name:self.stations[name+'_home']['slots']['home'] for name in states},indent=2)+'\n')

    def next_tasks(self, states, now):
        from .allocation import allocate, interpret
        self.register_homes(states)
        poses = {name: [float(s.pose[0]), float(s.pose[1]), math.degrees(float(s.pose[2]))]
                 for name, s in states.items() if not s.unavailable}
        (self.args.output/'session_state.json').write_text(json.dumps({
            'time': now, 'robots': {name: {'pose_m_rad': s.pose.tolist(),
                                        'available': not s.unavailable}
                                   for name, s in states.items()}}, indent=2)+'\n')
        print('\nReady for the next instruction. Simulation is held at the current positions.')
        while True:
            try:
                instruction = input('fleet> ').strip()
            except EOFError:
                return None
            if instruction == '/quit':
                return None
            if not instruction:
                continue
            if instruction == '/status':
                for name, s in states.items():
                    print(f'{name}: ({s.pose[0]:.2f}, {s.pose[1]:.2f}), '
                          + ('unavailable after failure' if s.unavailable else 'ready'))
                continue
            if instruction == '/stations':
                print(json.dumps(self.stations, indent=2))
                continue
            offline = instruction.startswith('/intent ')
            if instruction.startswith('/') and not offline:
                print('Commands: /status, /stations, /intent FILE, /quit')
                continue
            if not poses:
                print('No available robots remain. End the session and resolve the failures.')
                continue
            self.number += 1
            output = self.args.output/f'batch_{self.number:03d}'
            output.mkdir(parents=True, exist_ok=False)
            metrics = {'mode': 'offline_intent' if offline else 'llm_interpretation',
                       'interpretation_sdk_invocations': 0,
                       'execution_llm_invocations': 0, 'recovery_llm_invocations': 0,
                       'note': 'Application invocations; SDK/network retries are not measured.'}
            try:
                if offline:
                    intent = json.loads(Path(instruction[len('/intent '):].strip()).read_text())
                else:
                    # Count attempted requests even if the provider fails. No automatic repair call.
                    if self.model and (os.environ.get('GOOGLE_API_KEY') or os.environ.get('GEMINI_API_KEY')):
                        metrics['interpretation_sdk_invocations'] = 1
                    intent = interpret(instruction, poses, self.stations, self.model, metrics)
                (output/'intent.json').write_text(json.dumps(intent, indent=2)+'\n')
                mission, decisions = allocate(intent, poses, self.stations,
                                              {n: states[n].map for n in poses},
                                              self.args.planning_margin,
                                              occupied_poses={n:s.pose.tolist() for n,s in states.items()})
                moving = {t['robot'] for t in mission['tasks']}
                # A robot that has no work will not vacate its station. Reject
                # an occupied destination instead of waiting until a false stall.
                for task in mission['tasks']:
                    own = states[task['robot']]
                    for name, other in states.items():
                        if name != task['robot'] and name not in moving:
                            minimum = own.config.clearance + other.config.clearance + .15
                            if np.linalg.norm(np.asarray(task['goal'])-other.pose[:2]) < minimum:
                                raise ValueError(f'Destination occupied by idle/unavailable {name}; give it a departure task first')
                # Retain parked failed robots in the audit; executor already tracks them.
                for task in mission['tasks']:
                    task['id'] = f'b{self.number}:{task["id"]}'
                filename = output/'mission.json'
                filename.write_text(json.dumps(mission, indent=2)+'\n')
                read_mission(filename, self.args.robots_config)
                (output/'allocation.json').write_text(json.dumps(decisions, indent=2)+'\n')
                for decision in decisions:
                    print(f'{decision["job"]} -> {decision["robot"]} ({decision["basis"]})')
                print(f'Executing batch {self.number}; records: {output}')
                return mission['tasks']
            except Exception as exc:
                print(f'Instruction rejected; robots remain still: {exc}')
                (output/'error.txt').write_text(str(exc)+'\n')
            finally:
                (output/'llm_usage.json').write_text(json.dumps(metrics, indent=2)+'\n')


def main():
    import sys
    from datetime import datetime
    from .fleet import main as run
    # A fresh output folder prevents accidentally overwriting review evidence.
    output = Path('results/sessions')/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run(['--interactive', '--initialize-from-sim', '--output', str(output), *sys.argv[1:]])
