#!/usr/bin/env python3
"""Bind native TYPE1 placement into the existing complete TAB1 family drivers."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

HERE=Path(__file__).resolve().parent
PROJECT=HERE.parents[3]
BASE=PROJECT/'lib_utest/qualification/shell_tab1_force/native'
LAYERS=PROJECT/'lib_utest/qualification/shell_layered_j2/native_recurrence'
MANIFEST_SHA256='02936c7021bd20da3fd91e189f50f1238bbb6b0243350bad8f7aaac1aa22a2ff'


def replace_once(text,old,new):
    if text.count(old)!=1:
        raise ValueError('Native placement force seam changed: '+old)
    return text.replace(old,new)


def material(family,short):
    text=(LAYERS/f'NativeLayered{family}Section.F').read_text()
    start=text.index(f'      SUBROUTINE LR_{short}_SECTION(')
    text=text[:start]+'      END MODULE\n'
    text=text.replace('      USE LR_SECTION_MOD, ONLY: LR_UPDATE_SECTION\n','')
    text=text.replace(f'PUBLIC :: LR_{short}_MATERIAL,LR_{short}_SECTION',f'PUBLIC :: LR_{short}_MATERIAL')
    text=text.replace('      CONTAINS','      INTERFACE\n'
        '        SUBROUTINE PLACEMENT_NATIVE_SHIFT(IPOS,VALUE)\n'
        "     .   BIND(C,NAME='placement_native_shift')\n"
        '          USE ISO_C_BINDING, ONLY:C_INT,C_DOUBLE\n'
        '          INTEGER(C_INT),VALUE :: IPOS\n'
        '          REAL(C_DOUBLE),INTENT(OUT) :: VALUE\n'
        '        END SUBROUTINE\n'
        '      END INTERFACE\n      CONTAINS')
    text=replace_once(text,f'SUBROUTINE LR_{short}_MATERIAL(INPUT,',
        f'SUBROUTINE LR_{short}_MATERIAL(IPOS,INPUT,')
    anchor='      INTEGER MAT(MVSIZ),PID(MVSIZ),IGEO(100,1)'
    text=replace_once(text,anchor,'      INTEGER(C_INT),INTENT(IN) :: IPOS\n'+anchor)
    anchor='      GEO=ZERO' if family=='Qeph' else '      CALL T3_PREPARE_MATERIAL(INPUT,G,H,M)'
    geo='GEO(199,1)' if family=='Qeph' else 'M%GEO(199,1)'
    text=replace_once(text,anchor,anchor+f'\n      CALL PLACEMENT_NATIVE_SHIFT(IPOS,{geo})')
    name='QEPH' if family=='Qeph' else 'T3'
    text=text.replace(f'LR_{name}_SECTION_MOD',f'PLACED_{name}_MATERIAL_MOD')
    text=text.replace(f'LR_{short}_MATERIAL',f'PLACED_{short}_MATERIAL')
    return text


def adapt(directory):
    outputs={}
    for family,name,short in [('Qeph','QEPH','QE'),('T3','T3','T3')]:
        outputs[f'Placed{family}Material.F']=material(family,short).encode()
        for part in ['Section','Force']:
            text=(directory/f'Tab1{family}{part}.F').read_text()
            text=re.sub(r'\bTAB1_', 'PLACED_TAB1_',text)
            text=text.replace(f'tab1_{family.lower()}_force',f'placed_tab1_{family.lower()}_force')
            text=text.replace(f'LR_{short}_MATERIAL',f'PLACED_{short}_MATERIAL')
            text=text.replace(f'LR_{name}_SECTION_MOD',f'PLACED_{name}_MATERIAL_MOD')
            if part=='Force':
                text=replace_once(text,f'SUBROUTINE PLACED_TAB1_{name}_FORCE(X,',f'SUBROUTINE PLACED_TAB1_{name}_FORCE(IPOS,X,')
                text=replace_once(text,'INTEGER(C_INT),VALUE :: NPTS,MFUNC','INTEGER(C_INT),VALUE :: IPOS,NPTS,MFUNC')
                text=replace_once(text,f'CALL PLACED_{short}_MATERIAL(MATERIAL',f'CALL PLACED_{short}_MATERIAL(IPOS,MATERIAL')
                text=replace_once(text,f'CALL PLACED_TAB1_{short}_SECTION(',f'CALL PLACED_TAB1_{short}_SECTION(IPOS,')
            else:
                text=replace_once(text,f'SUBROUTINE PLACED_TAB1_{short}_SECTION(',f'SUBROUTINE PLACED_TAB1_{short}_SECTION(IPOS,')
                text=replace_once(text,'INTEGER(C_INT),INTENT(IN) :: MFUNC','INTEGER(C_INT),INTENT(IN) :: IPOS,MFUNC')
                text=replace_once(text,'CALL PLACED_TAB1_FAMILY_CALLER_SSP(', 'CALL PLACED_TAB1_FAMILY_CALLER_SSP(IPOS,')
            outputs[f'PlacedTab1{family}{part}.F']=text.encode()
    return outputs


def prepare(output,check):
    raw=(HERE/'source-manifest.json').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=MANIFEST_SHA256:
        raise ValueError('Changed native placement-force manifest')
    manifest=json.loads(raw)
    for row in manifest['inputs']:
        data=(PROJECT/row['path']).read_bytes()
        if len(data)!=row['bytes'] or hashlib.sha256(data).hexdigest()!=row['sha256']:
            raise ValueError('Changed native placement-force input: '+row['path'])
    base_output=output/'base-family'
    command=[sys.executable,'-B',str(BASE/'prepare_sources.py'),'--output',str(base_output)]
    if check: command.append('--check')
    subprocess.run(command,check=True)
    for name,data in adapt(base_output).items():
        if hashlib.sha256(data).hexdigest()!=manifest['generated'][name]:
            raise ValueError('Changed placed native family output: '+name)
        path=output/name
        if check:
            if path.read_bytes()!=data: raise ValueError('Stale native placement force: '+name)
        else:
            path.write_bytes(data)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    prepare(args.output,args.check)
