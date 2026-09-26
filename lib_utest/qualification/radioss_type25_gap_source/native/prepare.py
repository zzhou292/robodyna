#!/usr/bin/env python3
"""Original ordered gap blocks, qualification only; no production arithmetic."""
from pathlib import Path
import argparse,hashlib,json,re
ROOT=Path(__file__).resolve().parent
def sources():
    out={}
    for row in json.loads((ROOT/'source-manifest.json').read_text())['files']:
        data=(ROOT/row['path']).read_bytes()
        assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256']
        assert hashlib.sha1(f'blob {len(data)}\0'.encode()+data).hexdigest()==row['git_blob']
        out[row['name']]=data.decode()
    return out
def between(s,start,end):
    begin=s.index(start)
    return s[begin:s.index(end,begin)]
def generate():
    donor=sources()
    secondary=donor['i25sti3.F'].split('C     GAP NODES SECONDS',1)[1]
    secondary=between(secondary,'      DO I=1,NUMNOD\n','C---------put SECONDARY node on the free edge')
    extrema=between(donor['i25sti3.F'],'      DO I=1,NSN\n        IF(IGAP /= 3) THEN','C ---- FRUCTION Model Secondary Nodes Parts')
    main=donor['i25neigh.F'].split('      SUBROUTINE I25INI_GAP_N(',1)[1]
    main=between(main,'       DO I=1,NUMNOD\n','      CALL MY_DEALLOC(WA)')
    assert 'DO I=1,NUMELT' in secondary and 'DO I=1,NUMELP' in secondary and 'DO I=1,NUMELR' in secondary
    assert 'IF (IGTYP==12)' in secondary and 'IF (MSEGTYP(I)==0)' in main
    names={'ZERO','ONE','TWO','TEN','HUNDRED','HALF','EP30'}
    names.update('EP%02d'%i for i in range(2,21))
    declarations=[]
    for line in donor['constant_mod.F'].splitlines():
        match=re.search(r'my_real, parameter ::\s*(\w+)\s*=',line)
        if match and match[1] in names:declarations.append(line.replace('my_real','REAL(C_DOUBLE)',1))
    assert len(declarations)==len(names)
    text=(ROOT/'Gap.F.in').read_text().replace('@CONSTANTS@','\n'.join(declarations))
    for name,block in [('SECONDARY',secondary),('EXTREMA',extrema),('MAIN',main)]:
        assert text.count('@'+name+'@')==1;text=text.replace('@'+name+'@',block)
    return text
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path);parser.add_argument('--check',action='store_true')
    args=parser.parse_args();text=generate()
    if args.output:
        path=args.output/'Gap.F'
        if args.check:assert path.read_text()==text
        else:path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)
    print('Pinned I25STI3 and I25INI_GAP_N ordered gap blocks verified')
