"""Whole INSOL3D/NORMA1D and original solid BUILD_CNEL blocks."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
ROOT=Path(__file__).resolve().parent

def generate(tl_root):
    spec=importlib.util.spec_from_file_location('solid_support_sources',
        tl_root/'lib_utest/qualification/radioss_type25_selection/native/Sources.py')
    sources=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sources)
    donors={}
    for row in json.loads((ROOT/'source-manifest.json').read_text())['files']:
        data=(ROOT/row['path']).read_bytes()
        assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256']
        assert hashlib.sha1(b'blob '+str(len(data)).encode()+bytes([0])+data).hexdigest()==row['git_blob']
        donors[Path(row['path']).name]=data.decode()
    solid=sources.routine(donors['insol3.F'],'INSOL3D')
    solid=solid.replace('SUBROUTINE INSOL3D(', 'SUBROUTINE RD_SOURCE_INSOL3D(')
    solid=solid.replace('USE MESSAGE_MOD','USE SOLID_SUPPORT_MESSAGE\n      USE SOLID_SUPPORT_OBSERVATION')
    solid=solid.replace('use element_mod','use solid_support_element')
    solid=solid.replace('CALL NORMA1D(', 'CALL RD_SOURCE_NORMA1D(')
    before='       IF (NUSERM==-1) RETURN'
    assert solid.count(before)==1
    solid=solid.replace(before,'       observed_matches=IC\n'+before)
    before='       XS1=ZERO'
    assert solid.count(before)==1
    solid=solid.replace(before,'       observed_effective=IC\n'+before)
    normal=sources.routine(donors['norma1.F'],'NORMA1D').replace('SUBROUTINE NORMA1D(', 'SUBROUTINE RD_SOURCE_NORMA1D(')
    cnel=donors['build_cnel.F']
    first=cnel.index('      DO  K=2,9\n')
    last=cnel.index('      DO K=2,5\n',first)
    counts=cnel[first:last]
    first=cnel.index('      DO  K=2,9\n',cnel.index('C building the matrix Nod -> Solid element'))
    last=cnel.index('C building the matrix Nod -> Shell element',first)
    fill=cnel[first:last]
    assert 'NUMELS10' in counts and 'NUMELS20' in fill and 'NUMELS16' in fill
    wrapper=(ROOT/'Wrapper.F.in').read_text()
    for name,value in [('COUNTS',counts),('FILL',fill)]:
        assert wrapper.count('@'+name+'@')==1
        wrapper=wrapper.replace('@'+name+'@',value)
    return {
        'Insol.F':solid,'Normal.F':normal,'Wrapper.F':wrapper,
        'Boundary.F90':(ROOT/'Boundary.F90').read_text(),
        'Constants.F90':sources.constants(donors['constant_mod.F'],[solid,normal]).replace('selection_constants','solid_support_constants'),
        'implicit_f.inc':'      USE ISO_C_BINDING\n      USE SOLID_SUPPORT_CONSTANTS\n      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n',
        'com04_c.inc':'      INTEGER NUMNOD,NUMELS,NUMELS8,NUMELS10,NUMELS20,NUMELS16,NINTER25,IPRI\n      COMMON /RD_SOLID_COUNTS/ NUMNOD,NUMELS,NUMELS8,NUMELS10,NUMELS20,NUMELS16,NINTER25,IPRI\n',
        'scr03_c.inc':''}

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--tl-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    for name,text in generate(args.tl_root).items():
        path=args.output/name
        if args.check:assert path.read_text()==text,name
        else:
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(text)
    print('Whole INSOL3D/NORMA1D and original solid CNEL blocks authenticated')
