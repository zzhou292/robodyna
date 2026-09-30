#!/usr/bin/env python3
"""Verify the immutable coefficient adapter and unchanged native reference owners."""
from pathlib import Path
import hashlib
import json
import runpy
import re
here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest() == 'bcc6b5c45c72a5929147d30c788d913272cf599397c0bc1228aaab6fcb93a93b'
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    value = (root/path).read_bytes()
    assert len(value) == row['bytes'] and hashlib.sha256(value).hexdigest() == row['sha256'], path

def body(text, marker):
    start = text.index('{', text.index(marker))
    end, depth = start+1, 1
    while depth:
        depth += (text[end] == '{')-(text[end] == '}')
        end += 1
    return text[start+1:end-1]
checks = (root/'lib_src/assembly/SolidContributionChecks.h').read_text()
for name, sha in manifest['unchanged_bodies'].items():
    assert hashlib.sha256(body(checks, name).encode()).hexdigest() == sha, name
header = (root/'lib_src/assembly/SolidNodeContributions.h').read_text()
assert body(header, 'struct SolidCoefficientInput').startswith(manifest['original_input_prefix'])
assert 'Solid18, Solid24, Solid6z, Solid18Law44, Solid18Law90' in header
assert 'isotropic_inertia_kg_m2() noexcept { return 0; }' in header
budget = (root/'lib_src/assembly/NodalCoefficientBudget.cpp').read_text()
assert budget.index('solids->profile()!=SolidCoefficientProfile::OriginalThreeFamilies') < budget.index('const auto& shells=')
for folder in ['solid18_law44_reference', 'law90_solid18_reference']:
    runpy.run_path(str(here.parent/folder/'verify_source.py'))
print(json.dumps({'status':'passed', 'records':len(manifest['files']),
    'scope':'typed coefficient snapshot only; old ledger closed', 'native_execution':False}))
