#!/usr/bin/env python3
"""Owning analytic declarations, production and wrapper identities; complete donors reused."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys

here = Path(__file__).resolve().parent
root = here.parents[2]
for entry in json.loads((here / 'source-manifest.json').read_text())['files']:
    raw = (root / entry['path']).read_bytes()
    if len(raw) != entry['bytes'] or hashlib.sha256(raw).hexdigest() != entry['sha256']:
        raise ValueError('analytic owning source changed: ' + entry['path'])
receipt = json.loads((here / 'source_fixture/source-receipt.json').read_text())
part = receipt['part']
if (part['pid'] != 2000945 or part['count'] != 80 or part['unique_source_nodes'] != 156 or
        len(set(part['source_element_ids'])) != 80 or part['topology'] !=
        {'eight_distinct': 76, 'collapsed_top_edges_A_B_C_D_E_E_F_F': 4}):
    raise ValueError('airbag source census changed')
for name in ('export_fixture.py', 'prepare_fixture.py'):
    ast.parse((here / 'source_fixture' / name).read_text())
subprocess.run([sys.executable, '-B', str(here.parent / 'solid_law44_point/verify_source.py')], check=True)
print('Analytic source identity, fixed airbag declaration inventory and complete native donors PASS')
