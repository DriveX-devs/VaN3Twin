#!/home/marco/anaconda3/bin/python
"""Validate optional TIP decrease triggering and compare with TIP_increase."""
from pathlib import Path
from collections import Counter
import csv,difflib,hashlib,json,math,os,re,subprocess,textwrap
import xml.etree.ElementTree as ET
OUT=Path(__file__).resolve().parent.parent
BASE=OUT.parent/'TIP_increase';VAL=OUT/'validation';CMP=OUT/'comparison'
NS3=next(p for p in OUT.parents if (p/'ns3').is_file())
os.environ.setdefault('MPLCONFIGDIR',str(OUT/'.matplotlib'))
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from PIL import Image
CASES=['0_no_TIP','1_TIP_ped1003','2_TIP_ped1014','3_TIP_all_VRUs']
LABELS=['No TIP','TIP on ped1003','TIP on ped1014','TIP on all VRUs']
def read(path):
    with path.open(newline='') as stream:
        reader=csv.DictReader(stream);return list(reader),reader.fieldnames
def save(path,rows):
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def mobility(path):
    root=ET.parse(path).getroot()
    return hashlib.sha256(b''.join(ET.tostring(t) for t in root.findall('timestep'))).hexdigest()
old={r['scenario']:r for r in read(BASE/'comparison/CBR_summary.csv')[0]}
new={r['scenario']:r for r in read(CMP/'CBR_summary.csv')[0]}
metrics=[]
for label in LABELS:
    assert old[label].keys()==new[label].keys()
    for metric in old[label]:
        if metric=='scenario':continue
        a,b=float(old[label][metric]),float(new[label][metric])
        metrics.append(dict(scenario=label,metric=metric,TIP_increase=a,TIP_increase_decrease=b,difference=b-a,
                            relative_change_percent=(b-a)/a*100 if a else 0 if b==0 else '',
                            unchanged=math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-10)))
save(CMP/'TIP_increase_metrics_comparison.csv',metrics)
raw=[];motion=[];collisions=[];stations=[];transitions=[]
for case in CASES:
    for filename in ('VAM_metrics.csv','TIP_metrics.csv','CBR.csv','PRR_latency_CBR_summary.csv'):
        a,b=BASE/case/filename,OUT/case/filename
        if not a.exists() and not b.exists():continue
        assert a.exists() and b.exists(),(case,filename)
        x,hx=read(a);y,hy=read(b)
        raw.append(dict(case=case,file=filename,baseline_rows=len(x),current_rows=len(y),same_headers=hx==hy,exact_parsed_match=x==y and hx==hy))
    motion.append(dict(case=case,identical=mobility(BASE/case/'sumo/fcd.xml')==mobility(OUT/case/'sumo/fcd.xml')))
    def get_collisions(base):return [dict(c.attrib) for c in ET.parse(base/case/'sumo/collisions.xml').getroot().findall('collision')]
    a,b=get_collisions(BASE),get_collisions(OUT)
    collisions.append(dict(case=case,identical=a==b,baseline=a,current=b))
    a,b=read(BASE/case/'VAM_metrics.csv')[0],read(OUT/case/'VAM_metrics.csv')[0]
    for station in range(1000,1025):
        x=[r for r in a if int(r['station_ID'])==station];y=[r for r in b if int(r['station_ID'])==station]
        count=lambda rows:sum(r['triggering_condition']!='0' for r in rows)
        tips=lambda rows:sum(r['triggering_condition']=='6' for r in rows)
        stations.append(dict(case=case,station_ID=station,baseline_all_VAMs=len(x),current_all_VAMs=len(y),
                             baseline_triggered_VAMs=count(x),current_triggered_VAMs=count(y),VAM_difference=count(y)-count(x),
                             baseline_TIP_VAMs=tips(x),current_TIP_VAMs=tips(y),TIP_VAM_difference=tips(y)-tips(x)))
    for dataset in (BASE,OUT):
        path=dataset/case/'TIP_metrics.csv';data=read(path)[0] if path.exists() else []
        previous={};drops=0
        for row in data:
            key=(row['station_ID'],row['remote_station_ID']);tip=float(row['tip'])
            if tip<0 and previous.get(key,-1)>=0:drops+=1
            previous[key]=tip
        transitions.append(dict(dataset=dataset.name,case=case,evaluations=len(data),valid_evaluations=sum(float(r['tip'])>=0 for r in data),valid_to_invalid_transitions=drops))
save(CMP/'TIP_increase_raw_csv_comparison.csv',raw)
save(CMP/'TIP_increase_per_station_VAM_comparison.csv',stations)
save(CMP/'TIP_increase_prediction_transitions.csv',transitions)
assert all(r['identical'] for r in motion),'Mobility changed'
assert all(r['identical'] for r in collisions),'Collisions changed'
assert all(r['exact_parsed_match'] for r in raw if r['case']=='0_no_TIP'),'No-TIP metrics changed'

base_hashes=json.loads((VAL/'baseline-files-sha256.json').read_text())
assert {p.relative_to(BASE).as_posix() for p in BASE.rglob('*') if p.is_file()}==set(base_hashes)
assert all(sha(BASE/p)==digest for p,digest in base_hashes.items()),'TIP_increase was modified'
prior=json.loads((VAL/'source-before-sha256.json').read_text());expected=json.loads((VAL/'modified-files.json').read_text())
changed=[p for p,digest in prior.items() if sha(NS3/p)!=digest]
assert set(changed)==set(expected)
assert set(subprocess.check_output(['git','diff','--name-only'],cwd=NS3,text=True).splitlines())==set(expected)
patch=subprocess.check_output(['git','diff','--',*expected],cwd=NS3,text=True)
assert (OUT/'TIP_decrease.patch').read_text()==patch
(VAL/'source-after-sha256.json').write_text(json.dumps({p:sha(NS3/p) for p in prior},indent=2))
assert all(sha(NS3/p)==digest for p,digest in json.loads((BASE/'validation/source-after-sha256.json').read_text()).items() if p not in expected)
for p in expected:
    assert sha(VAL/'source-before'/p)==json.loads((BASE/'validation/source-after-sha256.json').read_text())[p]
    assert (NS3/'build/lib/libns3-dev-automotive-optimized.so').stat().st_mtime>=(NS3/p).stat().st_mtime
meta=json.loads((OUT/'run_metadata.json').read_text())
assert len(meta['runs'])==4 and 'completed' in meta and meta['TIP_decrease'] is True
assert meta['source_diff_sha256']==hashlib.sha256(subprocess.check_output(['git','diff'],cwd=NS3)).hexdigest()
assert all('--TIP-decrease=true' in (OUT/case/'command.txt').read_text() for case in CASES)
for name in ('plot_all.py','plot_results.py','plot_scenario.py','plot_vam_map.py'):
    assert (OUT/'scripts'/name).read_bytes()==(BASE/'scripts'/name).read_bytes()
tests=json.loads((VAL/'test-summary.json').read_text())
assert sum(t['checks'] for t in tests.values())==531
assert all(t['failed']==0 and t['passed']==t['checks'] and t['exit_code']==0 for t in tests.values())
compatibility=[];control_motion=[];control_collisions=[]
for label,folder,cases in [('explicit-false',VAL/'controls/explicit-false',CASES),('omitted',VAL/'controls/omitted',['3_TIP_all_VRUs'])]:
    control_meta=json.loads((folder/'run_metadata.json').read_text())
    assert 'completed' in control_meta and control_meta['TIP_decrease'] is False
    assert control_meta['source_diff_sha256']==meta['source_diff_sha256']
    for case in cases:
        command=(folder/case/'command.txt').read_text()
        assert ('--TIP-decrease=false' in command) if label=='explicit-false' else ('--TIP-decrease=' not in command)
        for filename in ('VAM_metrics.csv','TIP_metrics.csv','CBR.csv','PRR_latency_CBR_summary.csv'):
            a,b=BASE/case/filename,folder/case/filename
            if not a.exists() and not b.exists():continue
            assert a.exists() and b.exists()
            x,hx=read(a);y,hy=read(b)
            compatibility.append(dict(control=label,case=case,file=filename,baseline_rows=len(x),control_rows=len(y),identical=x==y and hx==hy))
        control_motion.append(dict(control=label,case=case,identical=mobility(BASE/case/'sumo/fcd.xml')==mobility(folder/case/'sumo/fcd.xml')))
        a=[dict(c.attrib) for c in ET.parse(BASE/case/'sumo/collisions.xml').getroot().findall('collision')]
        b=[dict(c.attrib) for c in ET.parse(folder/case/'sumo/collisions.xml').getroot().findall('collision')]
        control_collisions.append(dict(control=label,case=case,identical=a==b))
assert len(compatibility)==19 and all(row['identical'] for row in compatibility),'Backward compatibility failed'
assert all(row['identical'] for row in control_motion+control_collisions)
save(CMP/'TIP_increase_compatibility_checks.csv',compatibility)
traces=json.loads((CMP/'TIP_increase_trace_comparison.json').read_text())
assert traces['native_project_decoder']['decode_failures']==0
manifest=json.loads((OUT/'plot_titles.json').read_text())
manifest=[e for e in manifest if e['file']!='comparison/TIP_increase_comparison.png']
for case,pedestrians in {'0_no_TIP':(1003,1014),'1_TIP_ped1003':(1003,),'2_TIP_ped1014':(1014,),'3_TIP_all_VRUs':(1003,1014)}.items():
    for ped in pedestrians:
        entry=next(e for e in manifest if e['file']==f'{case}/plots/VAM_map_ped{ped}.png')
        assert entry['pedestrians']=={case:ped}
entry=next(e for e in manifest if e['file']=='comparison/VAM_map_all_scenarios.png')
assert [(p['position'],p['scenario'],p['pedestrian'],p['title']) for p in entry['panels']]==[
    ('upper_left','0_no_TIP',1003,'No TIP: map of ped1003'),('upper_right','1_TIP_ped1003',1003,'TIP on ped1003: map of ped1003'),
    ('lower_left','0_no_TIP',1014,'No TIP: map of ped1014'),('lower_right','2_TIP_ped1014',1014,'TIP on ped1014: map of ped1014')]
for panel in entry['panels']:
    assert panel['vam_count']==sum(int(r['station_ID'])==panel['pedestrian'] for r in read(OUT/panel['scenario']/'VAM_metrics.csv')[0])
panels=[('total_VAMs','Triggered VAMs','VAM count'),('TIP_VAMs','TIP-triggered VAMs','VAM count'),('mean_CBR_all_stations_percent','Mean channel busy ratio','CBR [%]'),('avg_PRR_150m_baseline','Packet reception ratio','PRR')]
fig,axes=plt.subplots(2,2,figsize=(14,9));x=np.arange(4)
for ax,(metric,title,ylabel) in zip(axes.flat,panels):
    for offset,label,data,color in [(-.19,'Increases only',old,'#607d8b'),(.19,'Increases and decreases',new,'#df8536')]:
        values=[float(data[s][metric]) for s in LABELS];bars=ax.bar(x+offset,values,.35,label=label,color=color)
        ax.bar_label(bars,labels=[str(int(v)) if 'VAM' in metric else f'{v:.5f}' for v in values],padding=3,fontsize=8,rotation=0 if 'VAM' in metric else 90)
    ax.set_xticks(x,['No TIP','ped1003','ped1014','All VRUs']);ax.set_ylabel(ylabel);ax.set_title(title,fontweight='bold')
    ax.set_ylim(0,max(float(data[s][metric]) for data in (old,new) for s in LABELS)*1.28 or 1)
    ax.spines[['top','right']].set_visible(False);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
figure_title='TIP increases and decreases - comparison with TIP_increase'
fig.suptitle(figure_title,fontweight='bold');fig.legend(*axes[0,0].get_legend_handles_labels(),loc='lower center',ncol=2,frameon=False)
fig.tight_layout(rect=(0,.05,1,.96));fig.savefig(CMP/'TIP_increase_comparison.png',dpi=160);plt.close(fig)
manifest.append(dict(file='comparison/TIP_increase_comparison.png',titles=[figure_title]+[p[1] for p in panels]))
assert len({e['file'] for e in manifest})==len(manifest)==49
for entry in manifest:
    assert not any(re.search(r'desync|desynchron|SUMO version|\b1\.(?:26|28)\b',t,re.I) for t in entry['titles'])
    with Image.open(OUT/entry['file']) as im:im.verify()
(OUT/'plot_titles.json').write_text(json.dumps(manifest,indent=2))
result=dict(baseline=str(BASE),current=str(OUT),TIP_decrease=True,default_TIP_decrease=False,summary_metrics_tested=len(metrics),
            summary_metrics_unchanged=sum(r['unchanged'] for r in metrics),summary_metrics_changed=sum(not r['unchanged'] for r in metrics),
            raw_csv_comparisons=raw,mobility=motion,collisions=collisions,prediction_transitions=transitions,tests=tests,
            compatibility_checks=compatibility,control_mobility=control_motion,control_collisions=control_collisions,
            production_files_changed=changed,baseline_files_preserved=len(base_hashes),validated_figures=len(manifest),
            source_provenance_note='Pre-task source hashes match the validated source saved in TIP_increase. The only production changes are the accepted three-file TIP-decrease patch.',trace_comparison=traces)
(CMP/'TIP_increase_comparison.json').write_text(json.dumps(result,indent=2))
lines=['Optional TIP decrease triggering - validation and comparison','',
 'The accepted patch adds --TIP-decrease, default false. Only three production files changed:',
 '- VRUBasicService.h: public setTIPDecrease(bool) setter and private bool initialized false.',
 '- VRUBasicService.cc: extend the existing selection predicate with an optional decrease comparison.',
 '- v2p-TIP-80211p.cc: false CLI default, register the option, pass it to each TIP-enabled pedestrian.',
 'The complete source patch has 10 added lines and 2 removed lines and is appended to this report.','',
 'When false or omitted, the current increase-only comparison is unchanged.',
 'When true, the selection also includes a valid current TIP at least 0.1 below that station\'s last reported TIP.',
 'The 0.1 threshold is an absolute probability difference (10 percentage points).',
 'A 1e-12 tolerance applies only to decreases, so decimal 0.70 -> 0.60 passes despite binary rounding.',
 'The existing increase comparison retains its original strict floating-point behavior.',
 'The reference remains the last actually submitted encoded probability (integer / 50).',
 'Unavailable (-1), NaN and infinite predictions are excluded. Invalid predictions are not converted to zero.',
 'Selection still sorts by current probability, limits the list to eight, and respects TIP mode ownership.',
 'Existing generation pacing, redundancy mitigation and DCC queue/timing controls remain active.',
 'Packet-specific confirmation updates reporting history after successful socket submission, including delayed',
 'packets. Socket submission does not guarantee actual PHY transmission or peer reception.','',
 'Validation results:']
for name,t in tests.items():lines.append(f"{name}: {t['passed']}/{t['checks']} checks passed; failed={t['failed']}.")
lines+=['Total: 531/531 focused checks passed. The v2p-TIP-80211p build passed.',
 'The new 108 checks cover omitted/false/true selection, valid decreases below/at/above 0.1, invalid data,',
 'both constructors, unchanged eight-entry sorting, actual TIP-triggered generation, motion-triggered VAMs,',
 'quantized payloads and reactive/adaptive queued decrease snapshots, drops and expiry.',
 'The prior 423 DCC, invalidation, heading and STC boundary checks were recompiled and passed.',
 'The deliberately closed-socket regression produces the expected GeoNetworking send-error log.',
 'All probes use the production implementation; test-only -fno-access-control allows state inspection.',
 'CLI help confirms --TIP-decrease defaults to false.','',
 'Simulation runs:',
 'Four fresh 100-second main runs use --TIP-decrease=true and --desync=true.',
 'Four explicit-false control runs reproduce all four TIP_increase scenarios.',
 'An additional all-VRU run omits --TIP-decrease and reproduces the same baseline.',
 'All nine runs exited successfully and reached 99.9 seconds in their mobility traces.',
 'All 19 raw CSV comparisons for the five compatibility controls are exactly equal to the baseline.',
 'Each main run contains the expected 25 pedestrians, 70 packet captures, and 70,000 CBR records.',
 'Mobility and collision records match TIP_increase in all main/control runs.',
 f"Recorded simulator: {meta['sumo_version']}",
 f"All {len(base_hashes)} files in TIP_increase retain their original hashes.",'',
 'Scenario | Triggered VAMs: increases -> increases/decreases | TIP-triggered VAMs: increases -> increases/decreases']
for label in LABELS:lines.append(f"{label} | {old[label]['total_VAMs']} -> {new[label]['total_VAMs']} | {old[label]['TIP_VAMs']} -> {new[label]['TIP_VAMs']}")
lines+=['','Counts above exclude one initial VAM per pedestrian (25 initial VAMs per case). Maps include initial VAMs.',
 'The main no-TIP CSV results remain identical because the new parameter only affects TIP-enabled services.',
 f"Changed summary metrics: {result['summary_metrics_changed']}/{len(metrics)}; unchanged: {result['summary_metrics_unchanged']}/{len(metrics)}.",
 f"Identical parsed main CSV pairs: {sum(r['exact_parsed_match'] for r in raw)}/{len(raw)}.",'',
 'Scenario | Mean CBR [%]: increases -> increases/decreases | PRR: increases -> increases/decreases']
for label in LABELS:lines.append(f"{label} | {old[label]['mean_CBR_all_stations_percent']} -> {new[label]['mean_CBR_all_stations_percent']} | {old[label]['avg_PRR_150m_baseline']} -> {new[label]['avg_PRR_150m_baseline']}")
lines+=['','Native packet validation:',
 'All captured VAMs decode with the project ASN.1 definitions. Encoded TIP probabilities remain in [0,50],',
 'lists contain at most eight entries, and unique generated message counts match the VAM CSVs.']
for row in traces['native_project_decoder']['comparisons']:
    lines.append(f"{row['case']}: generated {row['baseline_generated_VAMs']} -> {row['current_generated_VAMs']}; "
                 f"with TIP {row['baseline_VAMs_with_TIP']} -> {row['current_VAMs_with_TIP']}; "
                 f"changed TIP fields on common generation identities={row['changed_TIP_messages_on_common_identities']}.")
lines+=['Raw GeoNetworking packets also contain wall-clock timestamps; raw byte differences alone are not treated',
 'as changes to TIP behavior. Native semantic comparisons cover station, generation identity and TIP fields.','',
 'Saved outputs:',
 '- 49 validated figures, including the required six pedestrian maps and the ordered comparison map.',
 '- comparison/TIP_increase_comparison.png: VAM totals, TIP-triggered totals, mean CBR and PRR.',
 '- comparison/TIP_increase_metrics_comparison.csv: all 44 summary metrics and differences.',
 '- comparison/TIP_increase_per_station_VAM_comparison.csv: all 25 pedestrian counts in each scenario.',
 '- comparison/TIP_increase_raw_csv_comparison.csv: all 15 main raw CSV pairs.',
 '- comparison/TIP_increase_compatibility_checks.csv: all 19 default/false raw CSV control checks.',
 '- comparison/TIP_increase_trace_comparison.csv/json: trace hashes and native payload comparisons.',
 '- comparison/TIP_increase_comparison.json: complete comparison and verification metadata.',
 '- validation/: test sources, build commands/logs, passing checks, CLI help and compatibility runs.',
 '- TIP_decrease.patch: complete isolated three-file source changes.',
 '- run_metadata.json: exact settings, timings, source diff hash, counts and mobility hashes.',
 'All existing map selections and comparison panel positions are preserved.',
 'Synchronization and simulator-version wording are omitted from plot titles.','',
 'The executable default stays false. This reproduction script explicitly enables the parameter for the new results.',
 'Reproduce main runs: /usr/bin/python3 scripts/reproduce.py (use a fresh --output folder).',
 'Regenerate plots: /home/marco/anaconda3/bin/python scripts/plot_all.py',
 'Compare traces: /usr/bin/python3 scripts/compare_traces.py',
 'Regenerate comparison/report: /home/marco/anaconda3/bin/python scripts/compare_increase.py','',
 'Results: '+str(OUT),'Baseline: '+str(BASE),'','Complete source diff:',patch]
report='\n'.join(lines)+'\n';(OUT/'TIP_decrease_validation_report.txt').write_text(report)
with PdfPages(OUT/'TIP_decrease_validation_report.pdf') as pdf:
    wrapped=[]
    for line in lines:
        for part in line.splitlines() or ['']:
            wrapped.extend(textwrap.wrap(part,width=112,replace_whitespace=False,drop_whitespace=False) or [''])
    for start in range(0,len(wrapped),65):
        fig=plt.figure(figsize=(8.27,11.69));fig.text(.06,.96,'\n'.join(wrapped[start:start+65]),va='top',family='monospace',fontsize=7.5,linespacing=1.35)
        fig.text(.94,.025,f'Report page {start//65+1}',ha='right',fontsize=8,color='#666666');pdf.savefig(fig);plt.close(fig)
    fig,ax=plt.subplots(figsize=(11.69,8.27));ax.imshow(plt.imread(CMP/'TIP_increase_comparison.png'));ax.axis('off');fig.tight_layout();pdf.savefig(fig);plt.close(fig)
for label in LABELS:print(label,'baseline',old[label]['total_VAMs'],old[label]['TIP_VAMs'],'new',new[label]['total_VAMs'],new[label]['TIP_VAMs'])
print(f"PASS: {result['summary_metrics_changed']}/{len(metrics)} metrics changed; 531 tests; 19/19 compatibility CSVs; 49 figures; baseline preserved.")
print('Report:',OUT/'TIP_decrease_validation_report.pdf')
