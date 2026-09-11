"""Authenticate complete native donors and extract unchanged selected caller statements."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
MATERIAL = 'engine/source/materials/mat_share/'
SLICES = {
    'density_update.inc': ('engine/source/elements/solid/solide/srho3.F',144,149),
    'average_volume.inc': (MATERIAL+'mmain.F90',683,683),
    'total_strain.inc': (MATERIAL+'mulaw.F90',926,952),
    'engineering_strain.inc': (MATERIAL+'mulaw.F90',984,988),
    'internal_work.inc': (MATERIAL+'mulaw.F90',2996,3010),
    'stored_energy.inc': (MATERIAL+'mmain.F90',1997,2003),
    'viscosity_precision.inc': (MATERIAL+'mqviscb.F',121,127),
    'viscosity_rates.inc': (MATERIAL+'mqviscb.F',141,153),
    'viscosity_length.inc': (MATERIAL+'mqviscb.F',179,190),
    'viscosity_pressure.inc': (MATERIAL+'mqviscb.F',196,220),
    'viscosity_dt.inc': (MATERIAL+'mqviscb.F',259,262),
    'viscosity_stiffness.inc': (MATERIAL+'mqviscb.F',381,399),
    'initial_gs.inc': ('starter/source/materials/mat/mat042/hm_read_mat42.F',186,189),
    'initial_slots.inc': ('starter/source/materials/mat/mat042/hm_read_mat42.F',252,254),
    'initial_pm.inc': ('starter/source/materials/mat/hm_read_mat.F90',1464,1476),
    'observed_stretches.inc': ('engine/source/materials/mat/mat042/sigeps42.F',182,184),
    'observed_volume.inc': ('engine/source/materials/mat/mat042/sigeps42.F',231,233),
}

def prepare(output, check):
    manifest = json.loads((ROOT/'source-manifest.json').read_text())
    if manifest['revision'] != 'a62b27e6baa555d222a580d6218867d0be4d70b5':
        raise RuntimeError('Native revision changed')
    record = manifest['license']
    data = (ROOT/record['path']).read_bytes()
    if len(data) != record['bytes'] or hashlib.sha256(data).hexdigest() != record['sha256']:
        raise RuntimeError('Native license changed')
    sources = {}
    for record in manifest['sources']:
        data = (ROOT/record['path']).read_bytes()
        blob = hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if (len(data) != record['bytes'] or blob != record['git_blob_sha1'] or
                hashlib.sha256(data).hexdigest() != record['sha256']):
            raise RuntimeError('Native donor changed: '+record['source'])
        sources[record['source']] = data.decode('latin1')
    prepared = {Path(source).name:data for source,data in sources.items()
                if '/share/' in source or '/modules/' in source}
    for name,(source,first,last) in SLICES.items():
        prepared[name] = ''.join(sources[source].splitlines(keepends=True)[first-1:last])
    names = {'constant_mod','precision_mod'}
    for data in prepared.values():
        names.update(x.lower() for x in re.findall(r'COMMON\s*/\s*(\w+)\s*/',data,re.I))
    tokens = re.compile(r'\b(?:'+'|'.join(sorted(names,key=len,reverse=True))+r')\b',re.I)
    for name,data in prepared.items():
        value = tokens.sub(lambda m:'LAW42_CALLER_'+m.group().upper(),data).encode('latin1')
        path = output/name
        if check:
            if not path.is_file() or path.read_bytes() != value:
                raise RuntimeError('Prepared source changed: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True)
            path.write_bytes(value)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args = parser.parse_args()
    prepare(args.output,args.check)
