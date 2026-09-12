#!/usr/bin/env python3
"""Authenticate complete native beam leaves; only symbol namespaces change."""
import argparse, hashlib, json, re
from pathlib import Path
ROOT = Path(__file__).resolve().parent
NAMES = {'main_beam18','mulaw_ib','pevec3','pdefo3','pcurv3','pdlen3','pdamp3',
         'pfint3','pfcum3','pmcum3','sigeps44pi','pibuf3','mat_elem_mod',
         'elbufdef_mod','matparam_def_mod','sigeps131pi_mod','my_alloc_mod',
         'my_dealloc_mod','my_alloc','my_dealloc','m2lawpi','fail_beam18',
         'sigeps34pi','sigeps36pi','sigeps71pi','sigeps131pi'}
def prepare(output, check):
    manifest = json.loads((ROOT/'source-manifest.json').read_text())
    pattern = re.compile(r'\b(?:'+'|'.join(sorted(NAMES, key=len, reverse=True))+r')\b', re.I)
    for entry in manifest['sources']:
        data = (ROOT/entry['path']).read_bytes()
        blob = hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256'] or blob!=entry['git_blob_sha1']:
            raise ValueError('Native source changed: '+entry['source'])
        if Path(entry['source']).stem == 'pforc3': continue
        value = pattern.sub(lambda m:'B18F_'+m.group().upper(),data.decode('latin1'))
        value = re.sub(r'\bVINTER\b','L44S_REF_VINTER',value,flags=re.I).encode('latin1')
        path = output/Path(entry['source']).name
        if check:
            if not path.is_file() or path.read_bytes()!=value: raise ValueError('Prepared donor changed: '+str(path))
        else:
            output.mkdir(parents=True,exist_ok=True);path.write_bytes(value)
    print('Complete beam force callers and leaves authenticated')
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path);parser.add_argument('--check',action='store_true')
    args=parser.parse_args();prepare(args.output,args.check)
