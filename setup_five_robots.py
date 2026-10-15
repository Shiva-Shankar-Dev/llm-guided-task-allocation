"""Build a separate five-robot scene from the stopped current two-robot scene."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import numpy as np
from robot_navigation.fleet import read_mission
from robot_navigation.navigate_map import Map
from robot_navigation.paths import DEFAULT_MAP, PROJECT_ROOT
from robot_navigation.robot import RobotHandles


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply',action='store_true',help='copy models and save a new scene; otherwise only validate map/config')
    args=parser.parse_args()
    configs,_=read_mission(PROJECT_ROOT/'config/fleet_five.json')
    for name,(config,pose) in configs.items():
        m=Map(DEFAULT_MAP,config.clearance)
        if not m.valid(m.cell(pose[:2])) or m.inflate(~m.free,.2)[m.cell(pose[:2])]:
            raise ValueError(f'{name}: proposed start is not clear')
        print(name,pose[:2].tolist())
    for name,(config,pose) in configs.items():
        for other,(peer,p) in configs.items():
            if other!=name and np.linalg.norm(pose[:2]-p[:2]) < config.clearance+peer.clearance+.15:
                raise ValueError('Proposed starts overlap')
    if not args.apply:
        print('Map/config checked. With original scene stopped, use --apply to create scenes/environment_five.ttt.')
        return
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    sim=RemoteAPIClient().require('sim')
    if sim.getSimulationState()!=sim.simulation_stopped:
        raise RuntimeError('Stop the simulation and exit other controllers before setup')
    target=PROJECT_ROOT/'scenes/environment_five.ttt'
    if target.exists():
        raise RuntimeError(f'{target} already exists; open it instead. Setup will not overwrite it.')
    handles={n:sim.getObject('/'+n,{'noError':True}) for n in configs}
    if handles['Robot_A']==-1 or handles['Robot_B']==-1 or any(handles[n]!=-1 for n in ('Robot_C','Robot_D','Robot_E')):
        raise RuntimeError('Open the original two-robot scene, containing A/B and no C/D/E')
    for n in ('Robot_A','Robot_B'):
        RobotHandles.resolve(sim,configs[n][0])
    backup=PROJECT_ROOT/'results/five_robot_setup'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    backup.mkdir(parents=True)
    sim.saveScene(str(backup/'before_setup.ttt'))
    original={n:sim.getObjectPose(h,sim.handle_world) for n,h in handles.items() if h!=-1}
    created=[]
    try:
        for name in ('Robot_C','Robot_D','Robot_E'):
            h=sim.copyPasteObjects([handles['Robot_A']],1)[0]
            created.append(h)
            sim.setObjectAlias(h,name)
            handles[name]=h
        for name,(config,pose) in configs.items():
            h=handles[name]
            position=sim.getObjectPosition(h,sim.handle_world)
            sim.setObjectPosition(h,sim.handle_world,[float(pose[0]),float(pose[1]),position[2]])
            sim.setObjectOrientation(h,sim.handle_world,[0,0,float(pose[2])])
            resolved=RobotHandles.resolve(sim,config)
            for motor in resolved.motors:
                sim.setJointTargetVelocity(motor,0)
        sim.saveScene(str(target))
    except Exception:
        for h in reversed(created):
            sim.removeModel(h)
        for n,pose in original.items():
            sim.setObjectPose(handles[n],sim.handle_world,pose)
        raise
    print(f'Saved {target}. Original scene file preserved; snapshot: {backup}')


if __name__=='__main__':
    main()
