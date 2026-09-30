#!/usr/bin/env python3
"""Prepare unchanged native LAW42 and complete spectral helper subroutines."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent

def prepare(output, check):
    sources = {}
    names = {'constant_mod', 'precision_mod', 'sigeps42', 'valpvec_v',
             'valpvecdp_v', 'floatmin', 'floatmin_', 'floatmin__', 'finter',
             'prodaat', 'prodmat', 'my_shiftl', 'my_shiftl_', 'my_shiftr',
             'my_shiftr_', 'my_and', 'my_and_', 'my_or', 'my_or_'}
    for entry in json.loads((ROOT/'source-manifest.json').read_text())['sources']:
        value = (ROOT/entry['path']).read_bytes()
        blob = hashlib.sha1(b'blob '+str(len(value)).encode()+b'\0'+value).hexdigest()
        if len(value) != entry['bytes'] or hashlib.sha256(value).hexdigest() != entry['sha256'] or blob != entry['git_blob_sha1']:
            raise RuntimeError('Native donor changed: '+entry['source'])
        sources[entry['source']] = value.decode('latin1')
        names.update(n.lower() for n in re.findall(r'COMMON\s*/\s*(\w+)\s*/',value.decode('latin1'),re.I))
    result = {}
    for source,value in sources.items():
        leaf=Path(source).name
        if '/share/' in source or '/modules/' in source or leaf in {'sigeps42.F','prodAAT.F','prodmat.F','precision.c'}:
            result[leaf]=value
        if leaf=='sigeps33.F':
            for name in ('VALPVEC_V','VALPVECDP_V'):
                match=re.search(r'^      SUBROUTINE '+name+r'\(.*?^      END\s*$',value,re.M|re.S)
                if not match: raise RuntimeError('Complete native subroutine missing: '+name)
                result[name+'.F']=match.group()+'\n'
        if leaf=='hm_read_mat42.F':
            # Exact setup expressions; scalar wrapper supplies already selected MU/AL.
            result['bulk.inc']=''.join(value.splitlines(keepends=True)[196:198])
    tokens=re.compile(r'\b(?:'+'|'.join(sorted(names,key=len,reverse=True))+r')\b',re.I)
    for name,value in result.items():
        namespace=(lambda m:'law42_ref_'+m.group()) if name.endswith('.c') else (lambda m:'LAW42_REF_'+m.group().upper())
        data=tokens.sub(namespace,value).encode('latin1')
        path=output/name
        if check:
            if not path.is_file() or path.read_bytes()!=data:raise RuntimeError('Prepared source differs: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True)
            path.write_bytes(data)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    prepare(args.output,args.check)
