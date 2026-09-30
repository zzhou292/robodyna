#!/usr/bin/env python3
"""Verify pinned full donors; namespace leaves and extract exact caller ranges.

No material/geometry expressions, loop bounds or branch predicates are changed.
The selected wrapper owns the explicit profile and actual eight-point packets.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
MANIFEST = json.loads((ROOT/'source-manifest.json').read_text())
SOLID = 'engine/source/elements/solid/'
MATERIAL = 'engine/source/materials/mat_share/'
SLICES = {
    'point_selection.inc': (SOLID+'solide8e/s8e_sig.F', 67, 103),
    'selection_modulus.inc': (SOLID+'solide8e/s8eforc3.F', 628, 631),
    'selection_poisson.inc': (SOLID+'solide8e/s8eforc3.F', 641, 645),
    'point_length.inc': (SOLID+'solide8e/s8ederi_2.F', 132, 144),
    'density_update.inc': (SOLID+'solide/srho3.F', 144, 149),
    'density.inc': (MATERIAL+'mmain.F90', 693, 693),
    'average_volume.inc': (MATERIAL+'mmain.F90', 683, 683),
    'internal_work.inc': (MATERIAL+'mulaw.F90', 2996, 3010),
    'stored_energy.inc': (MATERIAL+'mmain.F90', 1997, 2003),
    'viscosity_precision.inc': (MATERIAL+'mqviscb.F', 121, 127),
    'viscosity_rates.inc': (MATERIAL+'mqviscb.F', 141, 153),
    'viscosity_length.inc': (MATERIAL+'mqviscb.F', 179, 190),
    'viscosity_pressure.inc': (MATERIAL+'mqviscb.F', 196, 220),
    'viscosity_dt.inc': (MATERIAL+'mqviscb.F', 259, 262),
    'viscosity_stiffness.inc': (MATERIAL+'mqviscb.F', 381, 399),
}
LEAVES = {
    's8ederic3.F','s8ejacip3.F','s8ederipr3.F','s8ederig3.F',
    's8eselecsh.F','s8ederi_bij.F','s8edefo3.F','s8efint3.F','s8efmoy3.F',
    's8eprst_ini.F','schkjabt3.F','srepiso3.F','sortho3.F','srrota3.F',
    's8zsigp3.F','s8esigp3i.F','s8ederish2.F','s8ea2bp8.F',
}

def prepare(output, check):
    if MANIFEST['revision'] != 'a62b27e6baa555d222a580d6218867d0be4d70b5':
        raise RuntimeError('Native source revision changed')
    license_record = MANIFEST['license']
    license_bytes = (ROOT/license_record['path']).read_bytes()
    if (len(license_bytes) != license_record['bytes'] or
            hashlib.sha256(license_bytes).hexdigest() != license_record['sha256']):
        raise RuntimeError('Native license changed')
    data = {}
    names = {'constant_mod','precision_mod','message_mod','elbufdef_mod','arret'}
    for entry in MANIFEST['sources']:
        value = (ROOT/entry['path']).read_bytes()
        blob = hashlib.sha1(b'blob '+str(len(value)).encode()+b'\0'+value).hexdigest()
        if (len(value) != entry['bytes'] or blob != entry['git_blob_sha1'] or
                hashlib.sha256(value).hexdigest() != entry['sha256']):
            raise RuntimeError('Native donor changed: '+entry['path'])
        decoded = value.decode('latin1')
        data[entry['source']] = decoded
        names.update(n.lower() for n in re.findall(r'COMMON\s*/\s*(\w+)\s*/',decoded,re.I))
        if Path(entry['source']).name in LEAVES:
            names.update(n.lower() for n in re.findall(r'^\s*SUBROUTINE\s+(\w+)',decoded,re.M|re.I))
    tokens = re.compile(r'\b(?:'+'|'.join(sorted(names,key=len,reverse=True))+r')\b',re.I)
    outputs = {}
    for source, value in data.items():
        name = Path(source).name
        if name in LEAVES or '/share/' in source or '/modules/' in source:
            if name in outputs and outputs[name] != value:
                raise RuntimeError('Ambiguous native source basename: '+name)
            outputs[name] = value
    for name,(source,first,last) in SLICES.items():
        outputs[name] = ''.join(data[source].splitlines(keepends=True)[first-1:last])
    for name,value in outputs.items():
        result = tokens.sub(lambda m:'SOLID18_FORCE_'+m.group().upper(),value).encode('latin1')
        path = output/name
        if check:
            if not path.is_file() or path.read_bytes() != result:
                raise RuntimeError('Prepared donor changed: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True)
            path.write_bytes(result)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args = parser.parse_args()
    prepare(args.output,args.check)
