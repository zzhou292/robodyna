#!/usr/bin/env python3
"""Independent pinned native nodal contribution blocks; source-only generation."""
from pathlib import Path
import argparse,hashlib,importlib.util,json
ROOT=Path(__file__).resolve().parent
helper=ROOT.parent.parent/'radioss_type25_selection/native/Sources.py'
spec=importlib.util.spec_from_file_location('contribution_source_helpers',helper)
source=importlib.util.module_from_spec(spec);spec.loader.exec_module(source)
def generate():
    donor={}
    for item in json.loads((ROOT/'source-manifest.json').read_text())['files']:
        data=(ROOT.parent/item['path']).read_bytes()
        assert len(data)==item['bytes'] and hashlib.sha256(data).hexdigest()==item['sha256']
        assert hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()==item['git_blob']
        donor[Path(item['path']).name]=data.decode()
    bulk=source.routine(donor['sbulk3.F'],'SBULK3')
    rinit=source.routine(donor['rinit3.F'],'RINIT3')
    first='      IF (I7STIFS /= 0) THEN\n';last='      ENDIF ! IF (I7STIF /= 0)\n'
    assert rinit.count(first)==rinit.count(last)==1
    spring=first+rinit.split(first,1)[1].split(last,1)[0]+last
    first='        IF (ILENG > 0) THEN\n'
    assert rinit.splitlines(keepends=True).count(first)==1
    tail=rinit.split('\n'+first,1)[1]
    length=first+tail.split('      ENDDO\nC\nc      CALL ANCN',1)[0]
    assert length.endswith('        ENDIF\n') and 'XL(I)=ONE' in length
    constants=source.constants(donor['constant_mod.F'],[bulk,spring,length]).replace('selection_constants','contribution_constants')
    outputs={'Constants.F90':constants,'Bulk.F':bulk}
    for name,tag,block in [('Spring.F','SPRING',spring),('Length.F','LENGTH',length)]:
        text=(ROOT/(name+'.in')).read_text();assert text.count('@'+tag+'@')==1
        outputs[name]=text.replace('@'+tag+'@',block)
    for name in ('Boundary.F90','BulkWrapper.F90'):outputs[name]=(ROOT/name).read_text()
    declaration='       INTEGER MVSIZ\n       PARAMETER (MVSIZ = 512)\n'
    assert donor['starter_mvsiz_p.inc'].count(declaration)==1
    outputs['mvsiz_p.inc']=declaration
    outputs['implicit_f.inc']='      USE ISO_C_BINDING\n      USE CONTRIBUTION_CONSTANTS\n      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n'
    outputs['vect01_c.inc']='      INTEGER LFT,LLT\n      COMMON /CONTRIBUTION_RANGE/LFT,LLT\n'
    outputs['param_c.inc']='      INTEGER,PARAMETER::NPROPM=32\n'
    return outputs
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path);parser.add_argument('--check',action='store_true');args=parser.parse_args()
    for name,text in generate().items():
        if args.output:
            path=args.output/name
            if args.check:assert path.read_text()==text
            else:path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)
    print('Pinned SBULK3 and complete selected RINIT3 value/length blocks prepared')
