#!/usr/bin/python3
"""Compare saved traces and TIP fields using the project's native ASN.1 decoder."""
from pathlib import Path
from collections import Counter,defaultdict
from decimal import Decimal
import csv,hashlib,json,struct,subprocess
OUT=Path(__file__).resolve().parent.parent
BASE=OUT.parent/'TIP_increase'
VAL=OUT/'validation';CMP=OUT/'comparison'
CASES=['0_no_TIP','1_TIP_ped1003','2_TIP_ped1014','3_TIP_all_VRUs']
CMP.mkdir(exist_ok=True)
def save(path,rows):
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
raw=[]
for case in CASES:
    for category in ('pcap','VAM_triggering','CAM_triggering'):
        old={p.name:p for p in (BASE/case/category).iterdir() if p.is_file()}
        new={p.name:p for p in (OUT/case/category).iterdir() if p.is_file()}
        assert set(old)==set(new),(case,category)
        for name in sorted(old):
            a,b=old[name].read_bytes(),new[name].read_bytes()
            raw.append(dict(case=case,category=category,file=name,baseline_bytes=len(a),current_bytes=len(b),
                            exact_byte_match=a==b,baseline_sha256=hashlib.sha256(a).hexdigest(),current_sha256=hashlib.sha256(b).hexdigest()))
save(CMP/'TIP_increase_trace_comparison.csv',raw)
llc=bytes.fromhex('aaaa030000008947')
input_path=VAL/'native-vam-payloads.tsv'
with input_path.open('w',newline='') as stream:
    writer=csv.writer(stream,delimiter='\t',lineterminator='\n');writer.writerow(['dataset','case','capture_time','payload_hex'])
    for dataset in (BASE,OUT):
        for case in CASES:
            seen=set()
            for path in sorted((dataset/case/'pcap').glob('*.pcap')):
                data=path.read_bytes();magic=data[:4]
                assert magic in (b'\xd4\xc3\xb2\xa1',b'\x4d\x3c\xb2\xa1'),path
                nanosecond=magic==b'\x4d\x3c\xb2\xa1'
                assert struct.unpack_from('<I',data,20)[0]==105,path
                offset=24
                while offset<len(data):
                    sec,subsec,length,_=struct.unpack_from('<IIII',data,offset);offset+=16
                    packet=data[offset:offset+length];offset+=length
                    position=packet.find(llc,0,64)
                    if position<0:continue
                    gn=position+8
                    if len(packet)<gn+44 or packet[gn+5]!=0x50:continue
                    payload_length=int.from_bytes(packet[gn+8:gn+10],'big')-4
                    if payload_length<2 or len(packet)<gn+44+payload_length:continue
                    payload=packet[gn+44:gn+44+payload_length]
                    if payload[:2]!=b'\x03\x10':continue
                    capture=f'{sec}.{subsec:09d}' if nanosecond else f'{sec}.{subsec:06d}000'
                    item=(capture,payload)
                    if item in seen:continue
                    seen.add(item);writer.writerow([dataset.name,case,capture,payload.hex()])
with (VAL/'native-decoded-TIP.tsv').open('w') as out,(VAL/'native-decode.stderr.log').open('w') as err:
    subprocess.run([str(VAL/'decode_vam'),str(input_path)],stdout=out,stderr=err,check=True)
rows=list(csv.DictReader((VAL/'native-decoded-TIP.tsv').open(),delimiter='\t'))
assert rows and all(row['decoded']=='true' for row in rows),'Native VAM decoding failed'
unique=defaultdict(dict);observations=defaultdict(Counter)
for row in rows:
    key=(row['dataset'],row['case']);capture_ms=int(Decimal(row['capture_time'])*1000)
    gen_ms=capture_ms-((capture_ms-int(row['generationDeltaTime']))%65536)
    identity=(int(row['station']),gen_ms);value=(row['subjects'],row['probabilities'],row['confidences'])
    assert identity not in unique[key] or unique[key][identity]==value,(key,identity)
    unique[key][identity]=value
    observations[key][(row['capture_time'],row['station'],row['generationDeltaTime'],*value)]+=1
    probabilities=[int(p) for p in row['probabilities'].split(',') if p]
    assert len(probabilities)<=8 and all(0<=p<=50 for p in probabilities)
comparisons=[];examples=[]
for case in CASES:
    a,b=unique[BASE.name,case],unique[OUT.name,case]
    common=a.keys()&b.keys();changed=[key for key in sorted(common) if a[key]!=b[key]]
    for dataset,decoded in ((BASE,a),(OUT,b)):
        with (dataset/case/'VAM_metrics.csv').open() as stream:expected=len(list(csv.DictReader(stream)))
        assert len(decoded)==expected,(dataset.name,case,len(decoded),expected)
    comparisons.append(dict(case=case,baseline_generated_VAMs=len(a),current_generated_VAMs=len(b),
                            baseline_VAMs_with_TIP=sum(bool(v[1]) for v in a.values()),current_VAMs_with_TIP=sum(bool(v[1]) for v in b.values()),
                            common_generation_identities=len(common),changed_TIP_messages_on_common_identities=len(changed),
                            removed_generation_identities=len(a.keys()-b.keys()),added_generation_identities=len(b.keys()-a.keys()),
                            identical_TIP_messages=a==b,identical_decoded_observations=observations[BASE.name,case]==observations[OUT.name,case]))
    for key in changed[:10]:examples.append(dict(case=case,station=key[0],generation_ms=key[1],baseline=a[key],current=b[key]))
save(CMP/'TIP_increase_native_TIP_comparison.csv',comparisons)
summary={category:dict(files=sum(r['category']==category for r in raw),exact_matches=sum(r['category']==category and r['exact_byte_match'] for r in raw)) for category in ('pcap','VAM_triggering','CAM_triggering')}
report=dict(file_summary=summary,native_project_decoder=dict(comparisons=comparisons,decode_failures=0,
    baseline_generated_VAMs=sum(row['baseline_generated_VAMs'] for row in comparisons),current_generated_VAMs=sum(row['current_generated_VAMs'] for row in comparisons),
    deduplicated_capture_observations=len(rows),changed_TIP_examples=examples),
    note='Native ASN.1 decoding covers TIP subject/probability/confidence, station, generation time and capture time. It is not a complete payload comparison. Raw GeoNetworking timestamps also depend on wall-clock time. Duplicate observations at the same capture time and payload are removed before decoding.')
(CMP/'TIP_increase_trace_comparison.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
