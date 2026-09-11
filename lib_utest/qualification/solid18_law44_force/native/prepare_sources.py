#!/usr/bin/env python3
"""Authenticate complete donors and selected exact caller statements."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
PIN = 'a62b27e6baa555d222a580d6218867d0be4d70b5'
SLICES = {
    'intab.F': ('engine/source/implicit/ind_glob_k.F', 4515, 4538),
    'plastic_work.inc': ('engine/source/materials/mat_share/mulaw.F90', 2205, 2217),
    'degeneracy_call.inc': ('engine/source/elements/solid/solide8e/s8eforc3.F', 1091, 1091),
}
LEAVES = {'degenes8.F','s8edefoc3.F','s8zfintp3.F'}

def prepare(output, check):
    manifest = json.loads((ROOT/'donor-manifest.json').read_text())
    if manifest['revision'] != PIN:
        raise ValueError('Rear caller donor pin changed')
    data = {}
    for entry in manifest['sources']:
        raw = (ROOT/entry['path']).read_bytes()
        blob = hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
        if (len(raw) != entry['bytes'] or blob != entry['git_blob_sha1'] or
                hashlib.sha256(raw).hexdigest() != entry['sha256']):
            raise ValueError('Rear donor changed: '+entry['path'])
        data[entry['source']] = raw.decode('latin1')
    values = {Path(s).name:d for s,d in data.items() if Path(s).name in LEAVES}
    for name,(source,first,last) in SLICES.items():
        values[name] = ''.join(data[source].splitlines(keepends=True)[first-1:last])
    if 'LOGICAL FUNCTION INTAB' not in values['intab.F'] or not values['intab.F'].rstrip().endswith('END'):
        raise ValueError('Complete INTAB extraction boundary changed')
    if not values['plastic_work.inc'].strip().startswith('do i') or not values['plastic_work.inc'].strip().endswith('end do'):
        raise ValueError('Plastic work extraction boundary changed')
    if 'IF (IDEG(I)>0) IDEG(I) = IDEG(I) + 10' not in values['degeneracy_call.inc']:
        raise ValueError('Selected caller degeneracy boundary changed')
    symbols = re.compile(r'\b(degenes8|s8edefoc3|s8zfintp3|intab|element_mod)\b',re.I)
    def renamed(match):
        return ('SOLID18_REF_ELEMENT_MOD' if match[0].lower() == 'element_mod'
                else 'REAR18_FORCE_'+match[0].upper())
    for name,value in values.items():
        raw = symbols.sub(renamed,value).encode('latin1')
        path = output/name
        if check:
            if not path.is_file() or path.read_bytes() != raw:
                raise ValueError('Prepared rear native bytes changed: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True)
            path.write_bytes(raw)

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--check',action='store_true')
    args = p.parse_args()
    prepare(args.output,args.check)
