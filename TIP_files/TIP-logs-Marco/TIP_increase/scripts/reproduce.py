#!/usr/bin/python3
"""Run four fresh 100 s TIP scenarios, always with transmission desynchronization."""
from pathlib import Path
import collections
import csv
import datetime
import hashlib
import json
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET

OUT = Path(__file__).resolve().parent.parent
NS3 = next(p for p in OUT.parents if (p/'ns3').is_file() and (p/'CMakeLists.txt').is_file())
REF = NS3/'TIP_files/TIP-logs/desync'
SUMO_HOME = Path.home()/'.local/share/van3twin/sumo'
SUMO_BIN = Path.home()/'.local/bin'
ENV = os.environ.copy()
ENV['PATH'] = str(SUMO_BIN) + ':/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
ENV['SUMO_HOME'] = str(SUMO_HOME)
ENV['OPENBLAS_NUM_THREADS']='1'
ENV['OMP_NUM_THREADS']='1'
ENV.pop('NS_LOG',None)
CASES=[('0_no_TIP','none',1003),('1_TIP_ped1003','single',1003),('2_TIP_ped1014','single',1014),('3_TIP_all_VRUs','all',1003)]

def read_csv(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))

def require(cond, message):
    if not cond:
        raise AssertionError(message)

def timestamp():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

metadata=dict(started=timestamp(),root=str(NS3),output=str(OUT),git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=NS3,text=True).strip(),sumo_version=subprocess.check_output([str(SUMO_BIN/'sumo'),'--version'],env=ENV,text=True).splitlines()[0],duration_s=100,desync=True,source_diff_sha256=hashlib.sha256(subprocess.check_output(['git','diff'],cwd=NS3)).hexdigest(),runs=[])
metadata['sumo_executable'] = str(SUMO_BIN/'sumo')
metadata['sumo_home'] = ENV['SUMO_HOME']
require('1.26.0' in metadata['sumo_version'], 'Wrong SUMO version selected')
(OUT/'source-changes.patch').write_bytes(subprocess.check_output(['git','diff','--binary'],cwd=NS3))
(OUT/'run_metadata.json').write_text(json.dumps(metadata,indent=2))
for name,mode,ped in CASES:
    folder=OUT/name
    require(not folder.exists(),f'Refusing to overwrite existing run {folder}')
    for sub in ['VAM_triggering','CAM_triggering','pcap','sumo']:
        (folder/sub).mkdir(parents=True,exist_ok=True)
    scenario=f'v2p-TIP-80211p --sumo-gui=false --sim-time=100 --met-sup=true --desync=true --tip-mode={mode} --tip-ped={ped} --log-dir={folder} --csv-log={folder}/VAM_metrics --csv-log-cumulative={folder}/PRR_latency_CBR_summary'
    argv=['/usr/bin/python3','./ns3','run','--no-build',scenario]
    (folder/'command.txt').write_text(scenario+'\n')
    start=time.monotonic()
    with (folder/'ns3_stdout.log').open('w') as stdout,(folder/'ns3_stderr.log').open('w') as stderr:
        r=subprocess.run(argv,cwd=NS3,env=ENV,stdout=stdout,stderr=stderr,timeout=240)
    elapsed=round(time.monotonic()-start,3)
    with (folder/'command.txt').open('a') as f:
        f.write(f'exit_code={r.returncode} wall_clock_s={elapsed}\n')
    require(r.returncode==0,f'{name}: simulation failed; see stderr log')
    vam=read_csv(folder/'VAM_metrics.csv')
    require(vam and all(None not in x for x in vam),f'{name}: malformed VAM CSV')
    ids={int(x['station_ID']) for x in vam}
    require(ids==set(range(1000,1025)),f'{name}: missing VRU VAM records')
    triggered=[x for x in vam if x['triggering_condition']!='0']
    causes=dict(collections.Counter(x['triggering_condition'] for x in triggered))
    if mode=='none':
        require(not (folder/'TIP_metrics.csv').exists(),f'{name}: TIP unexpectedly enabled')
        require(causes.get('6',0)==0,f'{name}: unexpected TIP-triggered VAM')
        tip=[]
    else:
        tip=read_csv(folder/'TIP_metrics.csv')
        require(tip,f'{name}: no TIP evaluations logged')
        owners={int(x['station_ID']) for x in tip}
        require(owners==({ped} if mode=='single' else set(range(1000,1025))),f'{name}: unexpected TIP-enabled stations')
        require(all(float(x['tip'])==-1 or 0<=float(x['tip'])<=1 for x in tip),f'{name}: invalid TIP values')
    cbr=read_csv(folder/'CBR.csv')
    require(len(cbr)==70000,f'{name}: expected 70 nodes x 1000 CBR windows')
    require(all(math.isfinite(float(x['CBR'])) and 0<=float(x['CBR'])<=1 for x in cbr),f'{name}: invalid CBR')
    with (folder/'sumo/fcd.xml').open() as handle:
        header = handle.read(1000)
    require('SUMO sumo 1.26.0' in header, f'{name}: FCD does not confirm SUMO 1.26.0')
    fcd=ET.parse(folder/'sumo/fcd.xml').getroot()
    timesteps=fcd.findall('timestep')
    last=max(float(x.attrib['time']) for x in timesteps)
    require(last>=99.8,f'{name}: SUMO stopped too early')
    require(len(list((folder/'pcap').glob('*.pcap')))==70,f'{name}: missing PCAP traces')
    require(len(list((folder/'VAM_triggering').glob('*.log')))==25,f'{name}: missing VRU trigger logs')
    collisions=[dict(c.attrib) for c in ET.parse(folder/'sumo/collisions.xml').getroot().findall('collision')]
    ET.parse(folder/'sumo/tripinfo.xml')
    record=dict(case=name,mode=mode,ped=ped,desync=True,returncode=r.returncode,wall_clock_s=elapsed,vams_including_start=len(vam),triggered_vams=len(triggered),trigger_counts=causes,tip_evaluations=len(tip),valid_tip_evaluations=sum(float(x['tip'])>=0 for x in tip),cbr_records=len(cbr),last_sumo_time=last,collisions=collisions)
    metadata['runs'].append(record)
    (OUT/'run_metadata.json').write_text(json.dumps(metadata,indent=2))
    print(json.dumps(record),flush=True)
metadata['completed']=timestamp()
# Identical SUMO mobility across cases is required for a controlled radio comparison.
def mobility_hash(path):
    root=ET.parse(path).getroot()
    payload=b''.join(ET.tostring(t) for t in root.findall('timestep'))
    return hashlib.sha256(payload).hexdigest()
metadata['mobility_sha256']={name:mobility_hash(OUT/name/'sumo/fcd.xml') for name,_,_ in CASES}
metadata['identical_mobility_across_cases']=len(set(metadata['mobility_sha256'].values()))==1
require(metadata['identical_mobility_across_cases'],'Mobility differs between radio scenarios')
(OUT/'run_metadata.json').write_text(json.dumps(metadata,indent=2))
print('PASS: four 100 s simulations; all use --desync=true; identical SUMO mobility.',flush=True)
