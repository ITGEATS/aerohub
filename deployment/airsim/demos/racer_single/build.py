"""Build a new, immutable candidate with fixed images and an explicit dependency cache."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import uuid

DEMO=Path(__file__).resolve().parent
REPO=DEMO.parents[3]
NATIVE=REPO/'src/tool/navigation/planner/racer/ros2_adapter/native'
SOURCE_LOCK=REPO/'src/tool/navigation/planner/racer/upstream.lock.json'

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def git_output(repo,*args):
    return subprocess.check_output(
        ['git','-C',str(repo),*args],text=True,encoding='utf-8').strip()

def native_identity():
    lock=json.loads(SOURCE_LOCK.read_text(encoding='utf-8'))
    rel=NATIVE.relative_to(REPO).as_posix()
    staged=git_output(REPO,'ls-files','--stage','--',rel).split()
    if len(staged)<2 or staged[0]!='160000':
        raise RuntimeError('RACER native source is not a gitlink')
    head=git_output(NATIVE,'rev-parse','HEAD')
    status=git_output(NATIVE,'status','--porcelain','--untracked-files=all')
    section='submodule.'+rel
    url=subprocess.check_output(
        ['git','config','-f',str(REPO/'.gitmodules'),'--get',section+'.url'],
        text=True,encoding='utf-8').strip()
    branch=subprocess.check_output(
        ['git','config','-f',str(REPO/'.gitmodules'),'--get',section+'.branch'],
        text=True,encoding='utf-8').strip()
    expected=lock['fork']
    if staged[1]!=head or head!=expected['commit']:
        raise RuntimeError('RACER gitlink, checkout, and source lock differ')
    if url!=expected['repository'] or branch!=expected['branch']:
        raise RuntimeError('RACER submodule metadata differs from source lock')
    if status:
        raise RuntimeError('RACER submodule checkout is dirty')
    return {rel:{'url':url,'branch':branch,'gitlink_commit':staged[1],
                 'checkout_commit':head,'clean':True}}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--release',type=Path,required=True)
    ap.add_argument('--deps',type=Path,required=True,help='directory with nlopt-debs and nlopt-debs.sha256')
    args=ap.parse_args()
    release=args.release.resolve();release.mkdir(parents=True,exist_ok=False)
    images=json.loads((DEMO/'images.lock.json').read_text(encoding='utf-8'))
    manifest={'status':'building','images':images,'source_files':{},
              'source_gitlinks':native_identity(),'commands':[]}
    for folder in [REPO/'src/tool/navigation/planner/racer',DEMO,REPO/'simulator/airsim/sensors',REPO/'deployment/swarm']:
        for p in sorted(folder.rglob('*')):
            if (p.is_file() and '__pycache__' not in p.parts and
                    NATIVE not in p.parents):
                manifest['source_files'][p.relative_to(REPO).as_posix()]=digest(p)
    for image in set(images.values()):
        actual=subprocess.check_output(['docker','image','inspect',image,'--format','{{.Id}}'],text=True).strip()
        if actual!=image:raise RuntimeError('image identity mismatch')
    shutil.copytree(args.deps/'nlopt-debs',release/'deps/nlopt-debs')
    shutil.copyfile(args.deps/'nlopt-debs.sha256',release/'deps/nlopt-debs.sha256')
    dependency_lock=json.loads((DEMO/'dependencies.lock.json').read_text())
    for rel,expected in dependency_lock['files'].items():
        if digest(release/'deps'/rel)!=expected:raise RuntimeError('dependency differs: '+rel)
    manifest['dependency_lock']=dependency_lock
    shutil.copytree(REPO/'simulator/airsim/sensors/uav_as2_sensor_adapter',release/'sensors/uav_as2_sensor_adapter',ignore=shutil.ignore_patterns('__pycache__'))
    # Current dev already contains the sensor changes used by the previous flight.
    manifest['sensor_files'] = {
        p.name: digest(p) for p in (release/'sensors/uav_as2_sensor_adapter').glob('*.py')}
    for part,image,script,target in [('native',images['algorithm'],'build_native.sh','/racer-run'),
                                    ('interfaces',images['workstation'],'build_interfaces.sh','/interfaces')]:
        out=release/part;out.mkdir()
        cmd=['docker','run','--rm','--init','--network','none','--pull=never','--name','racer-build-'+uuid.uuid4().hex[:10],
             '--mount',f'type=bind,source={REPO.as_posix()},target=/repo,readonly',
             '--mount',f'type=bind,source={out.as_posix()},target={target}',
             '--mount',f'type=bind,source={(release/"deps").as_posix()},target=/deps,readonly',
             '--entrypoint','bash',image,'/repo/deployment/airsim/demos/racer_single/'+script]
        manifest['commands'].append(cmd)
        (release/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
        with (release/(part+'-build.log')).open('x',encoding='utf-8') as f:
            result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
        if result.returncode:
            manifest.update(status='failed',failed_part=part,exit_code=result.returncode)
            (release/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
            raise RuntimeError(f'{part} build failed; see {release}')
    manifest['status']='built'
    manifest['artifacts']={p.relative_to(release).as_posix():digest(p)
        for part in ('native/install','interfaces/install','sensors','deps') for p in (release/part).rglob('*') if p.is_file()}
    (release/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print('Built candidate:',release)

if __name__=='__main__':main()
