#!/usr/bin/env python3
"""Authenticate LAW1 donors; only rename the native leaf and private PARAM common."""
import argparse,hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def prepare(output,check):
    manifest=json.loads((ROOT/'source-manifest.json').read_text());originals={}
    for e in manifest['sources']:
        b=(ROOT/e['path']).read_bytes()
        if len(b)!=e['bytes'] or hashlib.sha256(b).hexdigest()!=e['sha256']:
            raise RuntimeError('Changed native LAW1 source: '+e['path'])
        if 'git_blob' in e and hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()!=e['git_blob']:
            raise RuntimeError('Changed pinned LAW1 blob')
        originals[e['path']]=b
    prepared={}
    source=originals['original/sigeps01c.F'].decode('latin1')
    source=re.sub(r'\bSIGEPS01C\b','LAW1_POINT_REF_SIGEPS01C',source,flags=re.I)
    source=source.replace('"param_c.inc"','"law1_param_c.inc"')
    prepared['Sigeps01c.F']=source.encode('latin1')
    prepared['law1_param_c.inc']=re.sub(rb'\bPARAM\b',b'LAW1_POINT_REF_PARAM',
        originals['../qeph/original/engine/share/includes/param_c.inc'],flags=re.I)
    for e in manifest['fragments']:
        lines=originals[e['source']].splitlines(keepends=True)
        b=b''.join(b''.join(lines[a-1:z]) for a,z in e['ranges'])
        if len(b)!=e['bytes'] or hashlib.sha256(b).hexdigest()!=e['sha256']:
            raise RuntimeError('Changed native LAW1 fragment: '+e['file'])
        prepared[e['file']]=b
    for name,b in prepared.items():
        p=output/name
        if check:
            if not p.is_file() or p.read_bytes()!=b:raise RuntimeError('Stale prepared LAW1 source: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True)
            if not p.is_file() or p.read_bytes()!=b:p.write_bytes(b)
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--check',action='store_true')
    args=parser.parse_args();prepare(args.output,args.check)
