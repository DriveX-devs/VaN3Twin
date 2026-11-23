#!/home/marco/anaconda3/bin/python
"""Validate the finding-2 patch, compare with TIP_current, and generate the report."""
from pathlib import Path
from collections import Counter
import csv,difflib,hashlib,json,math,os,re,subprocess,textwrap
import xml.etree.ElementTree as ET
OUT=Path(__file__).resolve().parent.parent
BASE=OUT.parent/'TIP_current';VAL=OUT/'validation';CMP=OUT/'comparison'
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
        metrics.append(dict(scenario=label,metric=metric,TIP_current=a,TIP_finding2_addressed=b,difference=b-a,
                            relative_change_percent=(b-a)/a*100 if a else 0 if b==0 else '',
                            unchanged=math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-10)))
save(CMP/'TIP_current_metrics_comparison.csv',metrics)
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
save(CMP/'TIP_current_raw_csv_comparison.csv',raw)
save(CMP/'TIP_current_per_station_VAM_comparison.csv',stations)
save(CMP/'TIP_current_prediction_transitions.csv',transitions)
assert all(r['identical'] for r in motion),'Mobility changed'
assert all(r['identical'] for r in collisions),'Collisions changed'
assert all(r['exact_parsed_match'] for r in raw if r['case']=='0_no_TIP'),'No-TIP metrics changed'
base_hashes=json.loads((VAL/'baseline-files-sha256.json').read_text())
assert {p.relative_to(BASE).as_posix() for p in BASE.rglob('*') if p.is_file()}==set(base_hashes),'Baseline file set changed'
assert all(sha(BASE/p)==digest for p,digest in base_hashes.items()),'Baseline modified'
prior=json.loads((VAL/'source-before-sha256.json').read_text())
expected=json.loads((VAL/'modified-files.json').read_text())
changed=[p for p,digest in prior.items() if sha(NS3/p)!=digest]
assert set(changed)==set(expected),(changed,expected)
assert set(subprocess.check_output(['git','diff','--name-only'],cwd=NS3,text=True).splitlines())==set(prior)
patch=''.join(''.join(difflib.unified_diff((VAL/'source-before'/p).read_text().splitlines(keepends=True),(NS3/p).read_text().splitlines(keepends=True),fromfile='a/'+p,tofile='b/'+p)) for p in expected)
assert (OUT/'finding2.patch').read_text()==patch,'Isolated patch differs from final source'
(VAL/'source-after-sha256.json').write_text(json.dumps({p:sha(NS3/p) for p in prior},indent=2))
base_meta=json.loads((BASE/'run_metadata.json').read_text());meta=json.loads((OUT/'run_metadata.json').read_text())
assert sha(VAL/'worktree-before.patch')==base_meta['source_diff_sha256'],'Unexpected baseline source provenance'
assert len(meta['runs'])==4 and 'completed' in meta
assert meta['source_diff_sha256']==hashlib.sha256(subprocess.check_output(['git','diff'],cwd=NS3)).hexdigest()
for name in ('reproduce.py','plot_all.py','plot_results.py','plot_scenario.py','plot_vam_map.py'):
    assert (OUT/'scripts'/name).read_bytes()==(BASE/'scripts'/name).read_bytes(),name
for relative in expected:
    assert (NS3/'build/lib/libns3-dev-automotive-optimized.so').stat().st_mtime>=(NS3/relative).stat().st_mtime
traces=json.loads((CMP/'TIP_current_trace_comparison.json').read_text())
assert traces['native_project_decoder']['decode_failures']==0
tests=json.loads((VAL/'test-summary.json').read_text())
assert set(tests)=={'tip_dcc_checks','finding4_boundary','heading_boundary','stc_boundary'}
assert all(t['failed']==0 and t['passed']==t['checks'] and t['exit_code']==0 for t in tests.values())
manifest=json.loads((OUT/'plot_titles.json').read_text())
manifest=[e for e in manifest if e['file']!='comparison/TIP_current_comparison.png']
expected_maps={'0_no_TIP':(1003,1014),'1_TIP_ped1003':(1003,),'2_TIP_ped1014':(1014,),'3_TIP_all_VRUs':(1003,1014)}
for case,pedestrians in expected_maps.items():
    for ped in pedestrians:
        entry=next(e for e in manifest if e['file']==f'{case}/plots/VAM_map_ped{ped}.png')
        assert entry['pedestrians']=={case:ped}
entry=next(e for e in manifest if e['file']=='comparison/VAM_map_all_scenarios.png')
expected_panels=[('upper_left','0_no_TIP',1003,'No TIP: map of ped1003'),('upper_right','1_TIP_ped1003',1003,'TIP on ped1003: map of ped1003'),('lower_left','0_no_TIP',1014,'No TIP: map of ped1014'),('lower_right','2_TIP_ped1014',1014,'TIP on ped1014: map of ped1014')]
assert [(p['position'],p['scenario'],p['pedestrian'],p['title']) for p in entry['panels']]==expected_panels
for panel in entry['panels']:
    data=read(OUT/panel['scenario']/'VAM_metrics.csv')[0]
    assert panel['vam_count']==sum(int(r['station_ID'])==panel['pedestrian'] for r in data)
panels=[('total_VAMs','Triggered VAMs','VAM count'),('TIP_VAMs','TIP-triggered VAMs','VAM count'),('mean_CBR_all_stations_percent','Mean channel busy ratio','CBR [%]'),('avg_PRR_150m_baseline','Packet reception ratio','PRR')]
fig,axes=plt.subplots(2,2,figsize=(14,9));x=np.arange(4)
for ax,(metric,title,ylabel) in zip(axes.flat,panels):
    for offset,label,data,color in [(-.19,'TIP_current',old,'#607d8b'),(.19,'Finding 2 addressed',new,'#df8536')]:
        values=[float(data[s][metric]) for s in LABELS]
        bars=ax.bar(x+offset,values,.35,label=label,color=color)
        ax.bar_label(bars,labels=[str(int(v)) if 'VAM' in metric else f'{v:.5f}' for v in values],padding=3,fontsize=8,rotation=0 if 'VAM' in metric else 90)
    ax.set_xticks(x,['No TIP','ped1003','ped1014','All VRUs']);ax.set_ylabel(ylabel);ax.set_title(title,fontweight='bold')
    ax.set_ylim(0,max(float(data[s][metric]) for data in (old,new) for s in LABELS)*1.28 or 1)
    ax.spines[['top','right']].set_visible(False);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
figure_title='TIP reporting history - comparison with TIP_current'
fig.suptitle(figure_title,fontweight='bold');fig.legend(*axes[0,0].get_legend_handles_labels(),loc='lower center',ncol=2,frameon=False)
fig.tight_layout(rect=(0,.05,1,.96));fig.savefig(CMP/'TIP_current_comparison.png',dpi=160);plt.close(fig)
manifest.append(dict(file='comparison/TIP_current_comparison.png',titles=[figure_title]+[p[1] for p in panels]))
assert len({e['file'] for e in manifest})==len(manifest)==49
for entry in manifest:
    assert not any(re.search(r'desync|desynchron|SUMO version|\b1\.(?:26|28)\b',t,re.I) for t in entry['titles'])
    with Image.open(OUT/entry['file']) as im:im.verify()
(OUT/'plot_titles.json').write_text(json.dumps(manifest,indent=2))
result=dict(baseline=str(BASE),current=str(OUT),summary_metrics_tested=len(metrics),summary_metrics_unchanged=sum(r['unchanged'] for r in metrics),summary_metrics_changed=sum(not r['unchanged'] for r in metrics),
            raw_csv_comparisons=raw,mobility=motion,collisions=collisions,prediction_transitions=transitions,tests=tests,
            production_files_changed=changed,baseline_files_preserved=len(base_hashes),validated_figures=len(manifest),
            source_provenance_note='The pre-task source diff hash matches TIP_current. finding2.patch is the complete source difference from that baseline; source-changes.patch includes all prior local changes relative to Git HEAD.',trace_comparison=traces)
(CMP/'TIP_current_comparison.json').write_text(json.dumps(result,indent=2))
lines=['TIP reporting-history correction and DCC validation','',
 'The accepted patch is installed in five production files. Four fresh 100-second scenarios completed.',
 'TIP_current remains unchanged. The saved baseline source matches the pre-task source exactly.','',
 'Corrected behavior:',
 '1. Every VAM construction refreshes the eligible TIP selection, including VAMs triggered by motion checks',
 '   that occur before the TIP-trigger branch. Previously these VAMs could use an older cached selection.',
 '2. The last-reported probability is the encoded integer divided by 50, rather than the full-precision estimate.',
 '   Example: estimate 0.409 is encoded as 20 and recorded as 0.40. The current estimate remains 0.409.',
 '3. A packet-specific callback crosses BTP and GeoNetworking, is copied through DCC queues, and runs only',
 '   after Socket::Send succeeds. Queueing, queue drops, expiry and encoding/socket errors do not commit history.',
 '4. Deferred callbacks retain the TIP values and trigger of their own packet. A newer prediction cannot be',
 '   mistakenly recorded as the payload of an older queued VAM. Weak session tokens prevent use after termination',
 '   or service destruction. An expired map entry can retain confirmed history with current TIP unavailable (-1).',
 '5. Generation timestamps and motion snapshots initialize pacing after successful encoding, independently of',
 '   later transmission. Transmission counters, trigger counters, CSV metrics and transmission time advance only',
 '   on confirmation. This also initializes the previously uninitialized TIP counter.',
 '6. GeoNetworking returns the identity of the dequeued packet. The new request is queued only after that send',
 '   and DCC timing updates, preventing synchronous enqueue/checkQueue reentry from sending twice in one gate.',
 '   DCC last-transmit/AoI/adaptive updates follow successful sends, using the selected packet and updated airtime.',
 '7. MetricSupervisor accounting occurs once in each shared SHB/GBC sender, after successful socket submission.',
 '   The duplicate queue-wrapper accounting and its leaked allocation are removed.','',
 'Changed production files:',
 '- src/automotive/model/Facilities/VRUBasicService.cc: fresh selection, immutable encoded snapshots, guarded',
 '  completion handling, separated generation pacing and confirmed transmission accounting/history.',
 '- src/automotive/model/Facilities/VRUBasicService.h: initialized TIP counter, weak-session token ownership,',
 '  and an actual-transmission timestamp for CSV intervals.',
 '- src/automotive/model/BTP/btpdatarequest.h: optional local onTxSuccess callback in BTP and GN request structures.',
 '- src/automotive/model/BTP/btp.cc: forwards the callback without changing the encoded packet format.',
 '- src/automotive/model/GeoNet/geonet.cc: packet-specific confirmation in shared senders, correct dequeued',
 '  message identity, queue insertion order and DCC timing/accounting.','',
 'Validation:',
 'The v2p-TIP-80211p target rebuilt successfully. Existing ASN_DEBUG unused-function warnings remain.',
 'The focused DCC harness uses real 802.11p packet sockets, the real DCC queues and native ASN.1 decoding.',
 'It sets gate timings directly for deterministic coverage; it does not replace queue/sender implementations.',
 'Both reactive and adaptive DCC cover snapshots, disabled/full queues, expiry, termination, service destruction,',
 'expired current predictions, initial-generation pacing, packet substitution/gate spacing and queued GBC sends.',
 'Immediate sends, fresh motion-triggered payloads, encoding failure and socket failure are also checked.',
 'DCC currently increments dropped-full only when queuing is disabled, not on an already full queue. The full',
 'queue check therefore inspects retained queue occupancy and which packet is actually transmitted.',
 'These focused checks are separate from the four standard scenario runs, which retain their original configuration.']
for name,test in tests.items():lines.append(f"{name}: {test['passed']}/{test['checks']} checks passed; failed={test['failed']}.")
lines+=['',f"Total: {sum(t['passed'] for t in tests.values())}/{sum(t['checks'] for t in tests.values())} checks passed.",
        'Expected failure injection: a deliberately closed packet socket logs',
        "'Error! GeoNetworking could not send any packet.' The check verifies that history/counts remain unchanged.",
        'The invalid latitude fixture deliberately causes ASN.1 encoding failure and verifies unchanged history.',
        'All four full simulations exit successfully, reach 99.9 seconds in the mobility trace, and retain the same',
        'mobility and collision records as TIP_current. Each case includes all 25 VRUs and 70,000 CBR records.',
        f"Recorded simulator: {meta['sumo_version']}",
        f"Changed summary metrics: {result['summary_metrics_changed']}/{len(metrics)}; unchanged: {result['summary_metrics_unchanged']}/{len(metrics)}.",
        f"Identical parsed raw CSV datasets: {sum(r['exact_parsed_match'] for r in raw)}/{len(raw)}.",
        'The no-TIP raw CSV results are identical.','',
        'Scenario | Triggered VAMs: baseline -> new | TIP-triggered VAMs: baseline -> new']
for label in LABELS:
    lines.append(f"{label} | {old[label]['total_VAMs']} -> {new[label]['total_VAMs']} | {old[label]['TIP_VAMs']} -> {new[label]['TIP_VAMs']}")
lines+=['','Counts above exclude the initial dissemination-start VAM from each of the 25 pedestrians.',
        'Maps include initial VAMs. Numerical changes reflect the combined reporting-reference and selection fixes;',
        'these runs do not isolate the individual contribution of each subchange.','',
        'Native trace validation: all captured VAMs decode using the project ASN.1 definitions; integer TIP values',
        'are in [0,50] and each list contains at most eight entries. Unique generated-message counts match VAM CSVs.']
for row in traces['native_project_decoder']['comparisons']:
    lines.append(f"{row['case']}: generated {row['baseline_generated_VAMs']} -> {row['current_generated_VAMs']}; "
                 f"with TIP {row['baseline_VAMs_with_TIP']} -> {row['current_VAMs_with_TIP']}; "
                 f"changed TIP fields on common identities={row['changed_TIP_messages_on_common_identities']}; "
                 f"removed/added identities={row['removed_generation_identities']}/{row['added_generation_identities']}.")
lines+=['Raw packet equality also depends on existing wall-clock GeoNetworking timestamps. The semantic comparison',
        'covers TIP fields and generation identity; it is not a complete comparison of every payload field.','',
        'Scope and interpretation:',
        'Confirmation means successful socket submission, not guaranteed PHY transmission or reception by a peer.',
        'Existing DCC packet lifetimes and queue cancellation policy remain in force. A queued packet may still be',
        'sent after the corresponding prediction changes; history then records the values that packet actually carries.',
        'Callbacks from a terminated session do not update its service history or counters.',
        'The existing one-sided DELTA_TIP comparison is unchanged. This patch does not claim complete ETSI compliance.',
        'The accepted STC>=0 boundary, heading handling, prediction invalidation and ideal-positioning model remain.','',
        f"All {len(base_hashes)} baseline files retain their original hashes. Exactly five production files changed in this task.",
        'The five simulation/plot scripts were copied byte-for-byte. The complete task diff is finding2.patch.',
        'The full accumulated local diff relative to Git HEAD is source-changes.patch.',
        f"Validated figures: {len(manifest)}. All requested pedestrian maps and comparison panel positions are retained.",
        'No synchronization or simulator-version wording appears in figure titles.','',
        'Artifacts:',
        'comparison/TIP_current_comparison.png: visual comparison of VAM counts, TIP counts, mean CBR and PRR.',
        'comparison/TIP_current_metrics_comparison.csv: all 44 summary values and numerical changes.',
        'comparison/TIP_current_raw_csv_comparison.csv: all 15 raw CSV pairs, row counts and parsed equality.',
        'comparison/TIP_current_per_station_VAM_comparison.csv: every pedestrian, including initial/triggered counts.',
        'comparison/TIP_current_native_TIP_comparison.csv: native decoded packet TIP comparisons.',
        'comparison/TIP_current_trace_comparison.csv/json: trace hashes and semantic comparison details.',
        'comparison/TIP_current_comparison.json: complete validation/comparison metadata.',
        'validation/test-summary.json: all focused checks; associated test sources and build commands are saved.',
        'run_metadata.json: exact commands/settings, source hash, run durations, mobility hashes and collisions.','',
        'Results: '+str(OUT),'Baseline: '+str(BASE),'',
        'Regenerate plots: /home/marco/anaconda3/bin/python '+str(OUT/'scripts/plot_all.py'),
        'Compare traces: /usr/bin/python3 '+str(OUT/'scripts/compare_traces.py'),
        'Regenerate report: /home/marco/anaconda3/bin/python '+str(OUT/'scripts/compare_current.py'),'',
        'Complete source differences introduced by this task:',patch]
report='\n'.join(lines)+'\n';(OUT/'finding2_validation_report.txt').write_text(report)
with PdfPages(OUT/'finding2_validation_report.pdf') as pdf:
    wrapped=[]
    for line in lines:
        for part in line.splitlines() or ['']:
            wrapped.extend(textwrap.wrap(part,width=112,replace_whitespace=False,drop_whitespace=False) or [''])
    for start in range(0,len(wrapped),65):
        fig=plt.figure(figsize=(8.27,11.69));fig.text(.06,.96,'\n'.join(wrapped[start:start+65]),va='top',family='monospace',fontsize=7.5,linespacing=1.35)
        fig.text(.94,.025,f'Report page {start//65+1}',ha='right',fontsize=8,color='#666666');pdf.savefig(fig);plt.close(fig)
    fig,ax=plt.subplots(figsize=(11.69,8.27));ax.imshow(plt.imread(CMP/'TIP_current_comparison.png'));ax.axis('off');fig.tight_layout();pdf.savefig(fig);plt.close(fig)
for label in LABELS:print(label,'baseline',old[label]['total_VAMs'],old[label]['TIP_VAMs'],'new',new[label]['total_VAMs'],new[label]['TIP_VAMs'])
print(f"PASS: {result['summary_metrics_changed']}/{len(metrics)} metrics changed; {len(manifest)} figures; baseline preserved.")
print('Report:',OUT/'finding2_validation_report.pdf')
