#!/usr/bin/python3
from pathlib import Path
import csv,json,os,subprocess
VAL=Path(__file__).resolve().parent
ROOT=next(p for p in VAL.parents if (p/'ns3').is_file())
BASE=VAL.parent.parent/'TIP_increase'
template=json.loads((BASE/'validation/test-build-commands.json').read_text())
oldobj=template[0][template[0].index('-o')+1]
summary={}
for name in ('tip_decrease_checks','tip_dcc_checks','finding4_boundary','heading_boundary','stc_boundary','decode_vam'):
    commands=json.loads(json.dumps(template));obj=str(VAL/(name+'.o'));exe=str(VAL/name)
    commands[0][commands[0].index('-c')+1]=str(VAL/(name+'.cc'))
    commands[0][commands[0].index('-o')+1]=obj
    commands[1]=[obj if token==oldobj else token for token in commands[1]]
    commands[1][commands[1].index('-o')+1]=exe
    (VAL/(name+'-build-commands.json')).write_text(json.dumps(commands,indent=2))
    with (VAL/(name+'-build.log')).open('w') as log:
        for command in commands:
            result=subprocess.run(command,cwd=ROOT/'cmake-cache',stdout=log,stderr=subprocess.STDOUT)
            if result.returncode:print(name,'BUILD FAILED',flush=True);raise SystemExit(result.returncode)
    if name=='decode_vam':print('Native decoder rebuilt.',flush=True);continue
    records=VAL/(name+'-records');records.mkdir(exist_ok=True)
    with (VAL/(name+'-checks.csv')).open('w') as out,(VAL/(name+'-stderr.log')).open('w') as err:
        result=subprocess.run([exe,str(records)],cwd=ROOT,stdout=out,stderr=err,timeout=60)
    rows=list(csv.DictReader((VAL/(name+'-checks.csv')).open()))
    checks=sum(r.get('pass') in ('PASS','FAIL') for r in rows);passed=sum(r.get('pass')=='PASS' for r in rows)
    summary[name]=dict(checks=checks,passed=passed,failed=checks-passed,exit_code=result.returncode)
    (VAL/'test-summary.json').write_text(json.dumps(summary,indent=2))
    print(name,summary[name],flush=True)
    if result.returncode:raise SystemExit(result.returncode)
env=os.environ.copy();env['PATH']='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
result=subprocess.run(['/usr/bin/python3','./ns3','run','--no-build','v2p-TIP-80211p --PrintHelp'],cwd=ROOT,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
(VAL/'command-line-help.txt').write_text(result.stdout)
assert result.returncode==0 and '--TIP-decrease:' in result.stdout
line=next(line for line in result.stdout.splitlines() if '--TIP-decrease:' in line)
assert '[false]' in line or '[0]' in line,line
print('CLI help:',line.strip(),flush=True)
