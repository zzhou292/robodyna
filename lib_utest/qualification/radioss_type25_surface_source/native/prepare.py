"""Independent complete early surface source; hooks only observe source values."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import re
ROOT=Path(__file__).resolve().parent

def generate(tl_root):
    spec=importlib.util.spec_from_file_location('surface_source_extract',
        tl_root/'lib_utest/qualification/radioss_type25_selection/native/Sources.py')
    helper=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    donors={}
    for row in json.loads((ROOT/'source-manifest.json').read_text())['files']:
        raw=(ROOT/row['path']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256']
        assert hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()==row['git_blob']
        donors[Path(row['path']).name]=raw.decode()
    output={}
    routines=[('Solid.F','solid_surface_buffer.F',['SOLID_SURFACE_BUFFER','SURF_SEGMENT']),
      ('Shell.F','shell_surface_buffer.F',['SHELL_SURFACE_BUFFER']),
      ('Buffer.F','surface_buffer.F',['SURFACE_BUFFER']),
      ('Create.F','create_surface_from_element.F',['CREATE_SURFACE_FROM_ELEMENT']),
      ('Parts.F','create_element_from_part.F',['CREATE_ELEMENT_FROM_PART'])]
    for name,donor,names in routines:
        text='\n'.join(helper.routine(donors[donor],routine) for routine in names)
        if name=='Solid.F':
            # JJ, JS and the original insertion address are observed before
            # the complete unmodified SURF_SEGMENT writes its six-word row.
            lines=text.splitlines(keepends=True)
            text=''.join(('      CALL RD_FACE(IAD_SURF,JJ,JS)\n' if 'CALL SURF_SEGMENT(' in line else '')+line for line in lines)
            assert text.count('CALL RD_FACE(')==21
        if name=='Create.F':
            anchor='      CALL MY_ORDERS(0,IWORK,ITRI,INDEX,NSEG,5)\n'
            assert text.count(anchor)==1
            text=text.replace(anchor,anchor+'      CALL RD_SORTED(NSEG,INDEX)\n')
        output[name]=text
    cnel=donors['build_cnel.F']
    first=cnel.index('      DO  K=2,9\n')
    last=cnel.index('      DO I=1,NUMELIG3D\n',first)
    counts=cnel[first:last]
    at=cnel.index('C building the matrix Nod -> Solid element')
    first=cnel.index('      DO  K=2,9\n',at)
    last=cnel.index('      DO K=2,3\n',first)
    fill=cnel[first:last]
    wrapper=(ROOT/'Wrapper.F.in').read_text().replace('@CNEL_COUNTS@',counts).replace('@CNEL_FILL@',fill)
    output.update({'Wrapper.F':wrapper,'Modules.F90':(ROOT/'Modules.F90').read_text(),'my_orders.c':donors['my_orders.c'],
      'implicit_f.inc':'      USE ISO_C_BINDING\n      USE RD_SURFACE_OBSERVATION\n      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n',
      'com04_c.inc':'      INTEGER NUMNOD,NUMELS,NUMELS8,NUMELS10,NUMELS16,NUMELS20,\n     . NUMELC,NUMELTG,NUMELTG6,NUMELQ,NUMELTRIA,NUMELT,NUMELP,\n     . NUMELR,NPART\n      COMMON /RD_SURFACE_COUNTS/ NUMNOD,NUMELS,NUMELS8,NUMELS10,\n     . NUMELS16,NUMELS20,NUMELC,NUMELTG,NUMELTG6,NUMELQ,NUMELTRIA,\n     . NUMELT,NUMELP,NUMELR,NPART\n',
      'scr17_c.inc':'      INTEGER LIPART1\n      COMMON /RD_SURFACE_PART_WIDTH/ LIPART1\n',
      'param_c.inc':'',
      'remesh_c.inc':'      INTEGER NADMESH,KSH4TREE,KSH3TREE\n      COMMON /RD_SURFACE_ADAPT/ NADMESH,KSH4TREE,KSH3TREE\n'})
    return output

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--tl-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--check',action='store_true')
    a=p.parse_args()
    for name,text in generate(a.tl_root).items():
        path=a.output/name
        if a.check: assert path.read_text()==text,name
        else:
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(text)
    print('Full native surface routines, original CNEL and MY_ORDERS authenticated')
