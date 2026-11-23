#!/usr/bin/python3
"""Integration checks for the supplied TIP_logs.diff; does not modify source."""
import csv
import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parent
ROOT = BASE.parent.parent
ENV = os.environ.copy()
ENV['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
ENV['OPENBLAS_NUM_THREADS'] = '1'
ENV.pop('NS_LOG', None)

def rows(p):
    with p.open(newline='') as f:
        reader=csv.DictReader(f)
        return list(reader),reader.fieldnames

def check(condition, message):
    if not condition:
        raise AssertionError(message)

cases = [
    ('none_desync', 'none', 1003, 'true', 12),
    ('single_1003_desync', 'single', 1003, 'true', 15),
    ('single_1014_sync', 'single', 1014, 'false', 60),
    ('all_desync', 'all', 1003, 'true', 60),
]
results=[]
for name, mode, ped, desync, duration in cases:
    folder=BASE/name
    check(not folder.exists(), f'Preserve existing run: {folder}')
    for sub in ('VAM_triggering','CAM_triggering','pcap','sumo'):
        (folder/sub).mkdir(parents=True, exist_ok=True)
    scenario=f'v2p-TIP-80211p --sumo-gui=false --sim-time={duration} --met-sup=true --desync={desync} --tip-mode={mode} --tip-ped={ped} --log-dir={folder} --csv-log={folder}/VAM_metrics --csv-log-cumulative={folder}/PRR_latency_CBR_summary'
    argv=['/usr/bin/python3','./ns3','run','--no-build',scenario]
    (folder/'command.json').write_text(json.dumps(argv,indent=2))
    start=time.monotonic()
    with (folder/'stdout.log').open('w') as stdout, (folder/'stderr.log').open('w') as stderr:
        r=subprocess.run(argv,cwd=ROOT,env=ENV,stdout=stdout,stderr=stderr,timeout=180)
    item=dict(case=name,returncode=r.returncode,seconds=round(time.monotonic()-start,3),mode=mode,ped=ped,desync=desync,duration=duration)
    check(r.returncode==0,f'{name}: simulator failed; inspect stderr.log')
    vam,header=rows(folder/'VAM_metrics.csv')
    check(len(vam)>0,f'{name}: no VAM records')
    check(all(None not in row for row in vam),f'{name}: malformed VAM rows')
    station_key='station_ID'
    stations={row[station_key] for row in vam}
    check(len(stations)>1,f'{name}: VAM logging still restricted to one pedestrian')
    causes={}
    for row in vam:
        cause=row['triggering_condition']
        causes[cause]=causes.get(cause,0)+1
    if mode=='none':
        check(not (folder/'TIP_metrics.csv').exists(),f'{name}: unexpected TIP evaluation log')
        check(causes.get('6',0)==0,f'{name}: TIP triggered despite being disabled')
    else:
        tip,tip_header=rows(folder/'TIP_metrics.csv')
        check(len(tip)>0,f'{name}: no TIP evaluations')
        tip_stations={row['station_ID'] for row in tip}
        check(tip_header==['timestamp_ms','station_ID','remote_station_ID','remote_is_vehicle','ttc_s','stc_m','tip'],f'{name}: unexpected TIP schema')
        if mode=='single':
            check(tip_stations=={str(ped)},f'{name}: wrong TIP-enabled pedestrian(s): {tip_stations}')
        else:
            check(len(tip_stations)>1,f'{name}: all mode did not enable multiple pedestrians')
        for row in tip:
            probability=float(row['tip'])
            check(probability==-1 or (math.isfinite(probability) and 0<=probability<=1),f'{name}: invalid TIP')
        item.update(tip_evaluations=len(tip),tip_stations=sorted(tip_stations),valid_tip_evaluations=sum(float(row['tip'])>=0 for row in tip))
    cbr,cbr_header=rows(folder/'CBR.csv')
    check(cbr_header==['window_start_ms','window_end_ms','node_id','station_ID','CBR'],f'{name}: unexpected CBR schema')
    check(len(cbr)>0,f'{name}: no CBR records')
    busy=[float(row['CBR']) for row in cbr]
    check(all(math.isfinite(x) and -1e-9<=x<=1+1e-9 for x in busy),f'{name}: CBR outside [0,1]')
    check(any(x>0 for x in busy),f'{name}: no radio activity measured')
    summary,summary_header=rows(folder/'PRR_latency_CBR_summary.csv')
    check(len(summary)==1 and None not in summary[0],f'{name}: summary header/data mismatch')
    check('avg_last_CBR_MetricSupervisor' in summary_header,f'{name}: missing CBR summary')
    fcd=ET.parse(folder/'sumo/fcd.xml').getroot()
    last_time=max(float(t.attrib['time']) for t in fcd.findall('timestep'))
    if duration>55:
        check(last_time>55,f'{name}: SUMO did not run beyond its configuration end time')
    ET.parse(folder/'sumo/collisions.xml')
    ET.parse(folder/'sumo/tripinfo.xml')
    check(any((folder/'VAM_triggering').glob('*.log')),f'{name}: no VAM trigger logs')
    check(any((folder/'CAM_triggering').glob('*.log')),f'{name}: no CAM trigger logs')
    check(any((folder/'pcap').glob('*.pcap')),f'{name}: no PCAP traces')
    item.update(vam_records=len(vam),vam_stations=sorted(stations),trigger_counts=causes,cbr_records=len(cbr),maximum_cbr=max(busy),last_sumo_time=last_time)
    results.append(item)
    (BASE/'results.json').write_text(json.dumps(dict(date=datetime.datetime.now(datetime.timezone.utc).isoformat(),results=results),indent=2))
    print(json.dumps(item),flush=True)
print('PASS: all four TIP/logging integration scenarios',flush=True)
