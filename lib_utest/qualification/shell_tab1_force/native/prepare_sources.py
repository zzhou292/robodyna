#!/usr/bin/env python3
"""Reuse authenticated complete family drivers; adapt only the TAB1 caller packet."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import subprocess
import sys

HERE=Path(__file__).resolve().parent
PROJECT=HERE.parents[3]
BASE=PROJECT/'lib_utest/qualification/shell_failure_force/native'
NAMES={'lf_caller':'TAB1_FAMILY_CALLER','layered_failure_caller_ssp':'TAB1_FAMILY_CALLER_SSP',
       'lf_qeph_section_mod':'TAB1_QEPH_SECTION_MOD','lf_t3_section_mod':'TAB1_T3_SECTION_MOD',
       'lf_qe_section':'TAB1_QE_SECTION','lf_t3_section':'TAB1_T3_SECTION',
       'lf_qeph_force':'TAB1_QEPH_FORCE','lf_t3_force':'TAB1_T3_FORCE','d1':'TABLE_PARAMETERS'}
TOKEN=re.compile(r'\b(?:'+'|'.join(NAMES)+r')\b',re.I)

def adapt(directory):
    outputs={}
    for family in ('Qeph','T3'):
        for part in ('Section','Force'):
            text=(directory/f'Failure{family}{part}.F').read_text()
            text=TOKEN.sub(lambda match:NAMES[match.group().lower()],text)
            assert text.count('FAILURES(3,3)')==1
            text=text.replace('FAILURES(3,3)','FAILURES(5,3)')
            assert text.count('LINEAR(2),TABLE_PARAMETERS,TIME')==1
            text=text.replace('LINEAR(2),TABLE_PARAMETERS,TIME','LINEAR(2),TABLE_PARAMETERS(4),TIME')
            # C names remain lower case, matching the shared family convention.
            text=text.replace("NAME='TAB1_QEPH_FORCE'","NAME='tab1_qeph_force'")
            text=text.replace("NAME='TAB1_T3_FORCE'","NAME='tab1_t3_force'")
            text=text.replace('NAME="TAB1_QEPH_FORCE"','NAME="tab1_qeph_force"')
            text=text.replace('NAME="TAB1_T3_FORCE"','NAME="tab1_t3_force"')
            outputs[f'Tab1{family}{part}.F']=text.encode()
    return outputs

def prepare(output,check):
    raw=(HERE/'source-manifest.json').read_bytes()
    if hashlib.sha256(raw).hexdigest()!='1f71f2ab1c584773bcbc0970a02b714a9a11708d8bc6e3f6beccafb82fd2f1f0':
        raise RuntimeError('Changed TAB1 family manifest')
    manifest=json.loads(raw)
    for row in manifest['inputs']:
        data=(PROJECT/row['path']).read_bytes()
        if len(data)!=row['bytes'] or hashlib.sha256(data).hexdigest()!=row['sha256']:
            raise RuntimeError('Changed TAB1 family input: '+row['path'])
    base_output=output/'base-family'
    command=[sys.executable,'-B',str(BASE/'prepare_sources.py'),'--output',str(base_output)]
    if check: command.append('--check')
    subprocess.run(command,check=True)
    for name,data in adapt(base_output).items():
        if hashlib.sha256(data).hexdigest()!=manifest['generated'][name]:
            raise RuntimeError('Changed TAB1 family adaptation: '+name)
        path=output/name
        if check:
            if path.read_bytes()!=data: raise RuntimeError('Stale TAB1 family preparation: '+name)
        else:
            path.write_bytes(data)
    print('Verified complete family adaptation and distinct TAB1 packet')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    prepare(args.output,args.check)
