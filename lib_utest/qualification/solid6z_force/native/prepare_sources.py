#!/usr/bin/env python3
"""Authenticate complete S6Z donors and apply the one declared CXX repair.

All other changes are private symbol names. No native arithmetic is synthesized.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
LEAVES = {
    's6zrcoor3.F90', 's6zderi3.F90', 's6zderito3.F90', 's6zdefot3.F90',
    's6zdefc3.F90', 's6zdefo3.F90', 's6zhourg3.F90', 's6zfint3.F90',
    's6zrrota3.F90', 'sgcoor3.F', 'sdlen3.F', 'slen.F', 'srepiso3.F',
    'sortho3.F', 'srrota3.F', 'szetfac.F', 'sztorth3.F', 'sordeft3.F',
    'schkjabt3.F',
}


def prepare(output, check):
    manifest = json.loads((ROOT/'source-manifest.json').read_text())
    if manifest['revision'] != 'a62b27e6baa555d222a580d6218867d0be4d70b5':
        raise RuntimeError('S6Z native revision changed')
    license_record = manifest['license']
    raw_license = (ROOT/license_record['path']).read_bytes()
    if len(raw_license) != license_record['bytes'] or hashlib.sha256(raw_license).hexdigest() != license_record['sha256']:
        raise RuntimeError('Native license changed')
    data = {}
    names = {'constant_mod', 'precision_mod', 'message_mod', 'mvsiz_mod',
             'element_mod', 'prop_param_mod', 'elbufdef_mod', 'aleanim_mod',
             'ale_mod', 'arret', 'sorthdir3', 'slena', 'sldege', 'mdama24', 'szsvm'}
    for entry in manifest['sources']:
        value = (ROOT/entry['path']).read_bytes()
        blob = hashlib.sha1(b'blob '+str(len(value)).encode()+b'\0'+value).hexdigest()
        if len(value) != entry['bytes'] or blob != entry['git_blob_sha1'] or hashlib.sha256(value).hexdigest() != entry['sha256']:
            raise RuntimeError('Native donor changed: '+entry['source'])
        decoded = value.decode('latin1')
        data[entry['source']] = decoded
        names.update(n.lower() for n in re.findall(r'COMMON\s*/\s*(\w+)\s*/',decoded,re.I))
        if Path(entry['source']).name in LEAVES:
            names.update(n.lower() for n in re.findall(r'^\s*(?:SUBROUTINE|MODULE)\s+(\w+)',decoded,re.I|re.M))
    repairs = manifest['repairs']
    if len(repairs) != 1 or repairs[0]['policy'] != 'law42_material_sound_speed_v1':
        raise RuntimeError('Unexpected native repair policy')
    repair = repairs[0]
    value = data[repair['source']]
    before = repair['insertion_before']
    if before != '        select case(mtn)' or repair['inserted_line'] != '        cxx(1:nel) = ssp(1:nel)' or value.count(before) != 1:
        raise RuntimeError('Native CXX repair range changed')
    if hashlib.sha256(value.encode('latin1')).hexdigest() != repair['original_sha256']:
        raise RuntimeError('Native CXX original changed')
    value = value.replace(before,repair['inserted_line']+'\n'+before)
    if hashlib.sha256(value.encode('latin1')).hexdigest() != repair['repaired_sha256']:
        raise RuntimeError('Native CXX repaired source changed')
    for observation in manifest['observations']:
        if observation['source'] != repair['source']:
            raise RuntimeError('Unexpected observation source')
        anchor = observation.get('before',observation.get('after'))
        if value.count(anchor) != 1 or hashlib.sha256(anchor.encode()).hexdigest() != observation['anchor_sha256']:
            raise RuntimeError('Native observation anchor changed')
        insertion = observation['text']
        replacement = insertion+anchor if 'before' in observation else anchor+insertion
        value = value.replace(anchor,replacement)
    data[repair['source']] = value
    tokens = re.compile(r'\b(?:'+'|'.join(sorted(names,key=len,reverse=True))+r')\b',re.I)
    files = {}
    for source, value in data.items():
        name = Path(source).name
        if name in LEAVES or name.endswith('.inc') or '/modules/' in source:
            prepared = tokens.sub(lambda m: 'SOLID6Z_FORCE_'+m.group().upper(),value).encode('latin1')
            if name in files and files[name] != prepared:
                raise RuntimeError('Ambiguous source basename: '+name)
            files[name] = prepared
    property_source = data['common_source/modules/mat_elem/prop_param_mod.F90']
    files['property_bound.inc'] = ''.join(property_source.splitlines(keepends=True)[64:65]).encode('latin1')
    for name, value in files.items():
        destination = output/name
        if check:
            if not destination.is_file() or destination.read_bytes() != value:
                raise RuntimeError('Prepared native source changed: '+name)
        else:
            output.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(value)
    print(f'{len(data)} full native sources and explicit CXX repair verified')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args = parser.parse_args()
    prepare(args.output,args.check)
