#!/usr/bin/env python3
"""Authenticate complete placement donors and generate a scoped native packet."""
import argparse
import hashlib
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
MANIFEST_SHA256 = "bc749b214d84179d449950cf4a33dc5d535c17bdb8d6d3c685e73ca458609364"


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Native placement packet seam changed: ' + old)
    return text.replace(old, new)


def generate(manifest, inputs):
    outputs = {}
    for entry in manifest['fragments']:
        lines = inputs[entry['source']].splitlines(keepends=True)
        data = b''.join(b''.join(lines[a-1:b]) for a, b in entry['ranges'])
        if hashlib.sha256(data).hexdigest() != entry['sha256']:
            raise ValueError('Changed exact placement extract: ' + entry['file'])
        outputs[entry['file']] = data
    text = inputs[manifest['wm_source']].decode()
    text = re.sub(r'\bconstant_mod\b', 'LAW44_POINT_REF_CONSTANT_MOD', text, flags=re.I)
    text = re.sub(r'\bprecision_mod\b', 'LAW44_POINT_REF_PRECISION_MOD', text, flags=re.I)
    text = re.sub(r'\bshell_offset_wm_ini_mod\b', 'PLACEMENT_SHELL_OFFSET_WM_INI_MOD', text, flags=re.I)
    outputs['PlacementWm.F90'] = text.encode()
    text = inputs[manifest['caller_source']].decode()
    for symbol in ('LF_CALLER', 'layered_failure_caller_ssp', 'layered_failure_caller', 'layered_failure_packet'):
        text = re.sub(r'\b' + symbol + r'\b', 'placement_' + symbol, text, flags=re.I)
    text = replace_once(text, 'point_failure,parent_failure,israte_override,ssp_override)',
                        'point_failure,parent_failure,israte_override,ssp_override,position_override,moment_override)')
    text = replace_once(text, 'real(c_double),optional,intent(in) :: ssp_override',
                        'real(c_double),optional,intent(in) :: ssp_override,position_override(3),moment_override(3)')
    text = replace_once(text, 'call law44_point_section(posly(1,:),thkly,wm)',
                        'call law44_point_section(posly(1,:),thkly,wm)\n'
                        '    if(present(position_override)) posly(1,:)=position_override\n'
                        '    if(present(moment_override)) wm=moment_override')
    outputs['PlacementPacket.F90'] = text.encode()
    return outputs


def prepare(output, check):
    raw = (HERE / 'source-manifest.json').read_bytes()
    if hashlib.sha256(raw).hexdigest() != MANIFEST_SHA256:
        raise ValueError('Changed placement source manifest')
    manifest = json.loads(raw)
    inputs = {}
    for entry in manifest['inputs']:
        data = (ROOT / entry['path']).read_bytes()
        if len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
            raise ValueError('Changed placement input: ' + entry['path'])
        if 'git_blob_sha1' in entry:
            blob = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
            if blob != entry['git_blob_sha1']:
                raise ValueError('Changed placement donor identity: ' + entry['path'])
        inputs[entry['path']] = data
    outputs = generate(manifest, inputs)
    for name, data in outputs.items():
        if hashlib.sha256(data).hexdigest() != manifest['generated'][name]:
            raise ValueError('Changed generated placement packet: ' + name)
        path = output / name
        if check:
            if not path.is_file() or path.read_bytes() != data:
                raise ValueError('Stale placement source: ' + name)
        else:
            output.mkdir(parents=True, exist_ok=True)
            if not path.is_file() or path.read_bytes() != data:
                path.write_bytes(data)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    prepare(args.output, args.check)
