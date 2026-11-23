#!/home/marco/anaconda3/bin/python
"""Regenerate all reference figures using fresh runs and omit synchronization from titles."""
from pathlib import Path
import json
import os
import re
import runpy
import sys

SCRIPTS=Path(__file__).resolve().parent
OUT=SCRIPTS.parent
os.environ.setdefault('MPLCONFIGDIR',str(OUT/'.matplotlib'))
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
sys.path.insert(0,str(SCRIPTS))
import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure

manifest=[]
original_savefig=Figure.savefig

def save_verified(fig,filename,*args,**kwargs):
    titles=[]
    if fig._suptitle:
        titles.append(fig._suptitle.get_text())
    for ax in fig.axes:
        titles.extend(ax.get_title(loc=loc) for loc in ('left','center','right'))
    titles=list(dict.fromkeys(t for t in titles if t))
    if any(re.search(r'desync|desynchron|\b1\.(?:26|28)\b|SUMO version',t,re.I) for t in titles):
        raise AssertionError(f'Unexpected synchronization wording in figure titles: {titles}')
    original_savefig(fig,filename,*args,**kwargs)
    entry=dict(file=str(Path(filename).resolve().relative_to(OUT)),titles=titles)
    if hasattr(fig,"vam_map_pedestrians"):
        entry["pedestrians"]=fig.vam_map_pedestrians
    if getattr(fig,"vam_map_panels",None) is not None:
        entry["panels"]=fig.vam_map_panels
    manifest.append(entry)
    (OUT/'plot_titles.json').write_text(json.dumps(manifest,indent=2))
Figure.savefig=save_verified

sys.argv=[str(SCRIPTS/'plot_results.py')]
runpy.run_path(str(SCRIPTS/'plot_results.py'),run_name='__main__')
sys.argv=[str(SCRIPTS/'plot_vam_map.py')]
runpy.run_path(str(SCRIPTS/'plot_vam_map.py'),run_name='__main__')
sys.argv=[str(SCRIPTS/'plot_scenario.py')]
runpy.run_path(str(SCRIPTS/'plot_scenario.py'),run_name='__main__')

ns3=next(p for p in OUT.parents if (p/'ns3').is_file())
ref=ns3/'TIP_files/TIP-logs/desync'
missing=[]
for expected in ref.rglob('*.png'):
    relative=expected.relative_to(ref)
    replacements = {
        "1_TIP_ped1003/plots/VAM_map_ped1014.png": "1_TIP_ped1003/plots/VAM_map_ped1003.png",
        "comparison/VAM_map_ped1014_all_scenarios.png": "comparison/VAM_map_all_scenarios.png",
    }
    relative=Path(replacements.get(relative.as_posix(),relative.as_posix()))
    if not (OUT/relative).is_file():
        missing.append(str(relative))
for relative in ("0_no_TIP/plots/VAM_map_ped1003.png", "3_TIP_all_VRUs/plots/VAM_map_ped1003.png"):
    if not (OUT/relative).is_file():
        missing.append(relative)
if missing:
    raise AssertionError(f'Missing reference plot counterparts: {missing}')
print(f'PASS: {len(manifest)} regenerated figures; all reference plots reproduced; titles omit synchronization details.')
