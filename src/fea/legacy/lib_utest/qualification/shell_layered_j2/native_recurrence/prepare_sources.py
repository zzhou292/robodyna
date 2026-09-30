#!/usr/bin/env python3
"""Authenticate complete donors and preserve exact selected driver bytes."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
def prepare(output,check):
    manifest=json.loads((ROOT/'source-manifest.json').read_text())
    originals={}
    for entry in manifest['sources']:
        data=(ROOT/entry['path']).read_bytes()
        if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256']:
            raise RuntimeError('Changed layered donor: '+entry['path'])
        originals[entry['path']]=data.splitlines(keepends=True)
    for entry in manifest['fragments']:
        lines=originals[entry['source']]
        data=b''.join(b''.join(lines[a-1:b]) for a,b in entry['ranges'])
        if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256']:
            raise RuntimeError('Changed native extraction: '+entry['file'])
        path=output/entry['file']
        if check:
            if not path.is_file() or path.read_bytes()!=data:
                raise RuntimeError('Stale native extraction: '+entry['file'])
        else:
            output.mkdir(parents=True,exist_ok=True)
            if not path.is_file() or path.read_bytes()!=data:path.write_bytes(data)
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args();prepare(args.output,args.check)
