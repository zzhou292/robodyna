#!/usr/bin/env python3
"""Authenticate TT0 schedules, unchanged arithmetic and additive entry points."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
TL = ROOT.parents[2]
manifest = json.loads((ROOT / 'source-manifest.json').read_text())
for name, expected in manifest['files'].items():
    value = (TL / name).read_bytes()
    if len(value) != expected['bytes'] or hashlib.sha256(value).hexdigest() != expected['sha256']:
        raise RuntimeError('Rear startup source changed: ' + name)
for source in manifest['schedule_sources']:
    value = (TL / source['path']).read_bytes()
    blob = hashlib.sha1(b'blob ' + str(len(value)).encode() + b'\0' + value).hexdigest()
    if (len(value) != source['bytes'] or hashlib.sha256(value).hexdigest() != source['sha256'] or
            blob != source['git_blob_sha1']):
        raise RuntimeError('Complete native schedule changed: ' + source['path'])
    first, last = source['selected_lines']
    selected = b''.join(value.splitlines(keepends=True)[first-1:last])
    if hashlib.sha256(selected).hexdigest() != source['selected_sha256']:
        raise RuntimeError('TT0 schedule boundary changed')
for source in manifest['unchanged_arithmetic']:
    text = (TL / source['path']).read_text()
    begin = text.index(source['begin'])
    end = text.index(source['end'], begin)
    selected = text[begin:end].encode()
    if hashlib.sha256(selected).hexdigest() != source['sha256']:
        raise RuntimeError('Previously qualified arithmetic changed: ' + source['path'])
for source in manifest.get('additive_analytic_branches', []):
    text = (TL / source['path']).read_text()
    begin = text.index(source['begin'])
    end = text.index(source['end'], begin)
    if hashlib.sha256(text[begin:end].encode()).hexdigest() != source['sha256']:
        raise RuntimeError('Explicit analytic branch container changed: ' + source['path'])
subprocess.run([sys.executable, '-B', str(ROOT.parent / 'solid18_law44_force/verify_source.py')], check=True)
print('Rear TT0 schedules, unchanged force/work slices, explicit analytic material branches and strict ordinary entries PASS')
