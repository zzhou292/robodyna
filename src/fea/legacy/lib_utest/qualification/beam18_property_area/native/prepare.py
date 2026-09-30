"""Reuse full qualified DEFBEAM_SECT and mechanically extract source phase values."""
from pathlib import Path
import argparse
import hashlib
import json
ROOT=Path(__file__).resolve().parent
TL=ROOT.parents[3]
MANIFEST=ROOT/'source-manifest.json'

def generate():
    donors={}
    for p,e in json.loads(MANIFEST.read_text())['files'].items():
        raw=(TL/p).read_bytes()
        assert len(raw)==e['bytes'] and hashlib.sha256(raw).hexdigest()==e['sha256']
        donors[Path(p).name]=raw.decode()
    source=donors['hm_read_prop18.F']
    first=source.index('          AREA_I= ZERO\n')
    last=source.index('        IF(.NOT. IS_ENCRYPTED)THEN\n',first)
    printed=source[first:last]
    lines=[x for x in source.splitlines(keepends=True) if x.strip()=='GEO(1)  = AREA']
    assert len(lines)==1
    beam=donors['i25sti3.F'].split('      DO I=1,NUMELP\n',1)[1].split('      ENDDO\n',1)[0]
    rows=[x for x in beam.splitlines(keepends=True) if x.strip()=='DX=HALF*SQRT(GEO(1,MG))']
    assert len(rows)==1
    text=(ROOT/'PropertyArea.F.in').read_text()
    for key,value in [('READER_PRINT_VALUES',printed),('PROPERTY_STORE',lines[0]),
                      ('BEAM_GAP',rows[0].replace('GEO(1,MG)','GEO(1)'))]:
        assert text.count('@'+key+'@')==1
        text=text.replace('@'+key+'@',value)
    return text

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--check',action='store_true')
    a=p.parse_args();text=generate();path=a.output/'PropertyArea.F'
    if a.check: assert path.read_text()==text
    else:
        a.output.mkdir(parents=True,exist_ok=True);path.write_text(text)
    print('Original reader property AREA/GEO1 and beam gap expression authenticated')
