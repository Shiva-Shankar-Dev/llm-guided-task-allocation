"""LLM intent -> grounded jobs -> map-distance allocation -> fleet mission.

Greedy queued-distance allocation is deterministic, not a global optimal solver.
The LLM never invents coordinates or controls motors.
"""
import argparse
import json
import math
import os
import time
from pathlib import Path

import numpy as np

from .fleet import main as execute_fleet, read_mission, vector
from .navigate_map import Map, astar
from .paths import DEFAULT_MAP, PROJECT_ROOT
from .robot import DEFAULT_ROBOTS, load_robot, resolve_model


from .stations import normalize_stations, canonical_instruction, canonical_intent, ARRIVAL_TOLERANCE, canonical_robots, robot_aliases


INTENT_SCHEMA = {
    'type': 'object',
    'properties': {
        'clarification': {'type': 'string'},
        'jobs': {'type': 'array', 'items': {
            'type': 'object',
            'properties': {
                'id': {'type': 'string'},
                'robot': {'type': 'string', 'nullable': True},
                'locations': {'type': 'array', 'items': {'type': 'string'}},
            },
            'required': ['id', 'robot', 'locations'],
        }},
    },
    'required': ['clarification', 'jobs'],
}


def interpret(instruction, robot_names, stations, model, metrics=None):
    if not model:
        raise ValueError('Set GEMINI_MODEL or pass --model with a model available to your account')
    if not (os.environ.get('GOOGLE_API_KEY') or os.environ.get('GEMINI_API_KEY')):
        raise ValueError('Set GOOGLE_API_KEY (or GEMINI_API_KEY) in your environment; never put it in the mission')
    from google import genai
    from google.genai import types
    instruction = canonical_robots(canonical_instruction(instruction, stations), robot_names)
    prompt = f'''Translate the user's instruction into navigation jobs, not motor commands.
Available robot names: {json.dumps(list(robot_names))}
Available station names: {json.dumps(list(stations))}
Robot aliases: {json.dumps(robot_aliases(robot_names))}
Home destinations: {json.dumps({n:n+'_home' for n in robot_names if n+'_home' in stations})}
"Starting point", "initial position", "return home", and "go back to its start"
mean the specified robot's own *_home destination. For all robots going home,
create one explicitly assigned job per available robot with its own home.
If no robot is specified for a home request, ask which robot; do not invent one.
Each request is independent: no previous conversation is supplied.
Configured synonyms in the user instruction have already been normalized to canonical station names.
When asked to send both/all robots to a place, create a separate explicitly assigned job for each robot.
Different robots may visit the same station; the allocator chooses distinct arrival slots.
Only navigation visits are supported. Preserve each explicitly requested robot;
use null when no robot is specified, so the distance allocator chooses it.
Do not assign robots yourself when unspecified. A patrol or ordered visit sequence
is one job with its stations in the requested order and the same robot throughout.
Independent jobs execute concurrently when assigned to different robots. Jobs on
one robot are queued. Assign unique short IDs. Expand explicitly bounded repeats;
do not invent repetition counts. If the instruction is ambiguous, uses unknown
robots/stations, requires payload manipulation, unbounded patrols, timing, or
cross-robot dependencies, set clarification to a concise explanation and jobs=[].
For a valid request set clarification="". Output only the structured JSON.
User instruction (data): {json.dumps(instruction)}'''
    started=time.perf_counter()
    with genai.Client() as client:
        response = client.models.generate_content(
            model=model, contents=prompt,
            config=types.GenerateContentConfig(response_mime_type='application/json',
                                              response_schema=INTENT_SCHEMA, temperature=0,
                                              automatic_function_calling=types.AutomaticFunctionCallingConfig(disable=True)))
    if metrics is not None:
        usage=getattr(response,'usage_metadata',None)
        metrics.update(model=model,interpretation_sdk_invocations=1,
                       elapsed_seconds=round(time.perf_counter()-started,3),
                       prompt_tokens=getattr(usage,'prompt_token_count',None),
                       output_tokens=getattr(usage,'candidates_token_count',None),
                       thought_tokens=getattr(usage,'thoughts_token_count',None),
                       total_tokens=getattr(usage,'total_token_count',None))
    if not response.text:
        raise ValueError('LLM returned no task plan; no robot will move')
    return json.loads(response.text)


def validate_intent(intent, robot_names, stations):
    if not isinstance(intent, dict) or set(intent) != {'jobs','clarification'}:
        raise ValueError('Intent must contain exactly jobs and clarification')
    if not isinstance(intent['clarification'], str):
        raise ValueError('Invalid clarification field')
    if intent['clarification'].strip():
        raise ValueError('Clarification required: '+intent['clarification'])
    if not isinstance(intent['jobs'],list) or not 1 <= len(intent['jobs']) <= 100:
        raise ValueError('Plan needs 1–100 jobs')
    seen=set()
    total=0
    for job in intent['jobs']:
        if not isinstance(job,dict) or set(job) != {'id','robot','locations'}:
            raise ValueError('Each job must contain exactly id, robot, locations')
        if not isinstance(job['id'],str) or not job['id'].strip() or job['id'] in seen:
            raise ValueError('Job IDs must be nonempty and unique')
        seen.add(job['id'])
        if job['robot'] is not None and job['robot'] not in robot_names:
            raise ValueError(f'Unknown requested robot: {job["robot"]}')
        if not isinstance(job['locations'],list) or not job['locations']:
            raise ValueError('Each job needs an ordered nonempty station list')
        for location in job['locations']:
            if not isinstance(location,str) or location not in stations:
                raise ValueError(f'Unknown station: {location}')
        total+=len(job['locations'])
    if total>200:
        raise ValueError('Plan exceeds 200 visits; split it into smaller missions')


def route_length(map_, start, goal, margin=.2):
    blocked=map_.inflate(~map_.free,margin)
    path=astar(blocked,map_.cell(start),map_.cell(goal))
    if len(path)<2:
        return float(np.linalg.norm(np.asarray(goal)-start))
    points=np.asarray([map_.xy(cell) for cell in path])
    return float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()
                 +np.linalg.norm(np.asarray(start)-points[0])+np.linalg.norm(np.asarray(goal)-points[-1]))


def allocate(intent, poses, stations, maps, margin=.2, occupied_poses=None):
    stations = normalize_stations(stations)
    intent = canonical_intent(intent, stations)
    validate_intent(intent,poses,stations)
    tails={name:vector(pose,3,'initial pose')[:2].copy() for name,pose in poses.items()}
    queued={name:0. for name in poses}
    tasks,decisions=[],[]
    # Preserve a destination's owner so the prototype cannot park two robots
    # at overlapping stations. Full station occupancy scheduling is future work.
    reserved=[]
    jobs=list(enumerate(intent['jobs']))
    # Honor explicit assignments first so automatic work cannot consume their destinations.
    jobs.sort(key=lambda pair:(pair[1]['robot'] is None,pair[0]))
    for _,job in jobs:
        candidates={}
        ends={}
        selected_slots={}
        for name in sorted(poses):
            if job['robot'] is not None and name!=job['robot']:
                continue
            try:
                current=tails[name].copy()
                travel=0.
                choices=[]
                for location in job['locations']:
                    feasible_slots=[]
                    for slot, point in stations[location]['slots'].items():
                        target=np.asarray(point,dtype=float)
                        # Reserve arrival disks, including each robot's positional tolerance.
                        conflict=any(owner!=name and np.linalg.norm(target-p)<maps[name].clearance+maps[owner].clearance+.15+2*ARRIVAL_TOLERANCE
                                     for owner,p in reserved)
                        if conflict:
                            continue
                        # Current parked bodies also occupy space across instruction batches.
                        if any(peer!=name and np.linalg.norm(target-np.asarray(pose)[:2]) < maps[name].clearance+maps.get(peer,maps[name]).clearance+.15+2*ARRIVAL_TOLERANCE
                               for peer,pose in (occupied_poses or {}).items()):
                            continue
                        try:
                            length=route_length(maps[name],current,target,margin)
                            feasible_slots.append((length,slot,target))
                        except RuntimeError:
                            continue
                    if not feasible_slots:
                        raise RuntimeError(f'{location}: no reachable separated arrival slot; capacity exhausted or occupied')
                    length,slot,target=min(feasible_slots,key=lambda item:(item[0],item[1]))
                    choices.append((slot,target.tolist()))
                    travel+=length
                    current=target
                selected_slots[name]=choices
                candidates[name]={'travel_m':travel,'queued_m':queued[name],'projected_total_m':queued[name]+travel}
                ends[name]=current
            except RuntimeError as exc:
                candidates[name]={'unavailable':str(exc)}
        feasible=[name for name,cost in candidates.items() if 'projected_total_m' in cost]
        if not feasible:
            raise ValueError(f'Job {job["id"]} cannot be allocated: {json.dumps(candidates)}')
        selected=min(feasible,key=lambda name:(candidates[name]['projected_total_m'],name))
        decisions.append({'job':job['id'],'robot':selected,
                          'basis':'explicit request' if job['robot'] else 'minimum projected queued route distance',
                          'candidates':candidates})
        queued[selected]=candidates[selected]['projected_total_m']
        tails[selected]=ends[selected]
        for index,location in enumerate(job['locations']):
            slot,goal=selected_slots[selected][index]
            tasks.append({'id':f'{job["id"]}:{index+1}','robot':selected,'goal':list(goal),'station':location,'slot':slot,'arrival_tolerance':ARRIVAL_TOLERANCE,'job':job['id']})
            reserved.append((selected,np.asarray(goal,dtype=float)))
    mission={'robots':{name:{'initial_pose':list(pose)} for name,pose in poses.items()},'tasks':tasks}
    return mission,decisions


def capture_poses(names, robots_file):
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client=RemoteAPIClient(); sim=client.require('sim')
    if sim.getSimulationState()!=sim.simulation_stopped:
        raise ValueError('Stop simulation before capturing initial poses')
    poses={}
    for name in names:
        handle=resolve_model(sim,load_robot(name,robots_file))
        pos=sim.getObjectPosition(handle,sim.handle_world)
        matrix=sim.getObjectMatrix(handle,sim.handle_world)
        poses[name]=[pos[0],pos[1],math.degrees(math.atan2(matrix[4],matrix[0]))]
    return poses


def main():
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--instruction',help='natural-language navigation request')
    source.add_argument('--intent-file',type=Path,help='offline validated JSON intent, no LLM call')
    p.add_argument('--model',default=os.environ.get('GEMINI_MODEL'))
    p.add_argument('--robots-config',type=Path,default=DEFAULT_ROBOTS)
    p.add_argument('--poses',type=Path,default=PROJECT_ROOT/'config/fleet_demo.json',help='JSON with robots.initial_pose in metres/degrees')
    p.add_argument('--stations',type=Path,default=PROJECT_ROOT/'config/stations.json')
    p.add_argument('--map',type=Path,default=DEFAULT_MAP)
    p.add_argument('--capture-poses',action='store_true',help='read actual starting poses once; does not move robots')
    p.add_argument('--output',type=Path,default=Path('results/allocation'))
    p.add_argument('--planning-margin',type=float,default=.2)
    p.add_argument('--execute',action='store_true',help='execute validated allocated mission after saving it')
    args=p.parse_args()
    if not math.isfinite(args.planning_margin) or args.planning_margin<0:
        p.error('planning margin must be finite and nonnegative')
    try:
        pose_data=json.loads(args.poses.read_text())['robots']
        if not pose_data:
            raise ValueError('At least one available robot is required')
        configs={name:load_robot(name,args.robots_config) for name in pose_data}
        poses={name:vector(spec['initial_pose'],3,name).tolist() for name,spec in pose_data.items()}
        stations=json.loads(args.stations.read_text())
        if not isinstance(stations,dict) or not stations:
            raise ValueError('Station configuration must be a nonempty object')
        stations=normalize_stations(stations)
        if args.capture_poses:
            poses=capture_poses(poses,args.robots_config)
        else:
            print('Using initial poses from file; refresh them if robots moved before allocating/executing.')
        metrics={'mode':'offline_intent' if args.intent_file else 'llm_interpretation',
                 'interpretation_sdk_invocations':0,
                 'execution_llm_invocations':0,'recovery_llm_invocations':0,
                 'note':'Application-level invocations; provider/SDK network retries are not measured. No monetary savings claimed.'}
        intent=json.loads(args.intent_file.read_text()) if args.intent_file else interpret(args.instruction,poses,stations,args.model,metrics)
        maps={name:Map(args.map,config.clearance) for name,config in configs.items()}
        mission,decisions=allocate(intent,poses,stations,maps,args.planning_margin)
        args.output.mkdir(parents=True,exist_ok=True)
        mission_file=args.output/'mission.json'
        mission_file.write_text(json.dumps(mission,indent=2)+'\n')
        # Validate that the executor accepts the generated mission before offering execution.
        read_mission(mission_file,args.robots_config)
        (args.output/'intent.json').write_text(json.dumps(intent,indent=2)+'\n')
        (args.output/'allocation.json').write_text(json.dumps(decisions,indent=2)+'\n')
        (args.output/'llm_usage.json').write_text(json.dumps(metrics,indent=2)+'\n')
        for decision in decisions:
            print(f'{decision["job"]} -> {decision["robot"]} ({decision["basis"]})')
            for name,cost in decision['candidates'].items():
                if 'unavailable' in cost:
                    print(f'  {name}: unavailable: {cost["unavailable"]}')
                else:
                    print(f'  {name}: route {cost["travel_m"]:.2f} m + queue {cost["queued_m"]:.2f} m = {cost["projected_total_m"]:.2f} m')
        print(f'Mission saved: {mission_file}. Nothing moves unless --execute is supplied.')
    except (ValueError,TypeError,KeyError,OSError) as exc:
        p.exit(1,f'Allocation rejected: {exc}\n')
    if args.execute:
        execute_fleet(['--mission',str(mission_file),'--robots-config',str(args.robots_config),
                       '--map',str(args.map),'--output',str(args.output/'execution'),
                       '--planning-margin',str(args.planning_margin)])


if __name__=='__main__':
    main()
