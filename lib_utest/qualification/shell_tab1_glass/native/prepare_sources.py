#!/usr/bin/env python3
"""Authenticate complete TAB1 donors and exact extracts; rename private symbols only."""
import argparse
import hashlib
import json
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parent
NAMES={name:'GLASS_TAB1_'+name.upper() for name in (
    'fail_tab_c','table_mod','interface_table_mod','table_interp','table_vinterp',
    'message_mod','finter','arret')}
NAMES['vinter2']='LAW44_POINT_REF_VINTER2'
TOKEN=re.compile(r'\b(?:'+'|'.join(NAMES)+r')\b',re.I)
def prepare(output,check):
    manifest=json.loads((ROOT/'source-manifest.json').read_text())
    raw={}
    for entry in manifest['sources']:
        data=(ROOT/entry['path']).read_bytes()
        blob=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256'] or blob!=entry['git_blob_sha1']:
            raise ValueError('Changed TAB1 donor: '+entry['path'])
        raw[entry['path']]=data
    outputs={'NativeFailure.F':raw['original/fail_tab_c.F'],'TableModule.F':raw['original/table_mod.F']}
    for entry in manifest['fragments']:
        lines=raw[entry['source']].splitlines(keepends=True)
        data=b''.join(b''.join(lines[a-1:b]) for a,b in entry['ranges'])
        if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256']:
            raise ValueError('Changed TAB1 native extraction: '+entry['file'])
        outputs[entry['file']]=data
    baseline=json.loads((ROOT/'baseline-manifest.json').read_text())
    for entry in baseline['harnesses']:
        data=(ROOT/entry['path']).read_bytes()
        if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256']:
            raise ValueError('Changed pre-refactor native harness: '+entry['path'])
        text=re.sub(r'\bLF_INTERFACES\b','LEGACY_LF_INTERFACES',data.decode(),flags=re.I)
        text=re.sub(r'\bLF_CALLER\b','LEGACY_LF_CALLER',text,flags=re.I)
        text=re.sub(r'\blayered_failure_caller\b','legacy_layered_failure_caller',text,flags=re.I)
        text=re.sub(r'\blayered_failure_parent\b','legacy_layered_failure_parent',text,flags=re.I)
        outputs[entry['prepared']]=text.encode()
    for name,data in outputs.items():
        result=TOKEN.sub(lambda m:NAMES[m.group().lower()],data.decode('latin1')).encode('latin1')
        path=output/name
        if check:
            if not path.is_file() or path.read_bytes()!=result:raise ValueError('Stale TAB1 preparation: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True)
            if not path.is_file() or path.read_bytes()!=result:path.write_bytes(result)
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args();prepare(args.output,args.check)
