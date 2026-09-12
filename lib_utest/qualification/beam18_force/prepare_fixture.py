#!/usr/bin/env python3
"""Reuse the authenticated geometry generator; close the beam material cards."""
import argparse,importlib.util,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def prepare(fixture,output):
    path=ROOT.parent/'beam18_reference/prepare_fixture.py'
    spec=importlib.util.spec_from_file_location('beam_geometry',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.prepare(fixture,output)
    manifest=json.loads((fixture/'manifest.json').read_text())
    for part in manifest['declarations']['parts']:
        cards=part['material']['cards']
        row=cards[1]['text'];values=[row[n:n+10].strip() for n in range(0,50,10)]
        if [float(values[0]),float(values[1]),int(values[2]),values[3],float(values[4])]!=[8000.,8.,2100270,'',0.]:
            raise ValueError('Original beam LAW44 rate/curve/profile changed')
        if any(cards[0]['text'][n:n+10].strip() for n in (50,60,70)):
            raise ValueError('Original beam tangent/failure declaration changed')
    print('All four original beam material rate/curve/blank-failure associations authenticated')
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--fixture',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();prepare(a.fixture,a.output)
