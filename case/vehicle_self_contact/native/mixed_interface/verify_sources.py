#!/usr/bin/env python3
"""Read-only identity and source-boundary check; numerical oracles stay in tests."""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
p = argparse.ArgumentParser()
p.add_argument('--workspace', type=Path, required=True)
p.add_argument('--tl-root', type=Path, required=True)
a = p.parse_args()
here = Path(__file__).resolve().parent
app = here.parents[3]
manifest = json.loads((here/'source-manifest.json').read_text())
for group, root in [('app_files', app), ('tl_files', a.tl_root)]:
    for row in manifest[group]:
        relative = Path(row['path'])
        assert not relative.is_absolute() and '..' not in relative.parts
        data = (root/relative).read_bytes()
        assert len(data) == row['bytes'], relative
        assert hashlib.sha256(data).hexdigest() == row['sha256'], relative
subprocess.run([sys.executable, '-B', str(here.parent/'solid_surfaces/verify_sources.py'),
    '--workspace', str(a.workspace)], check=True)
public = (here.parent/'MixedInterfaceSource.h').read_text()
assert 'ClassifiedAndSidesBeforeSupport' in public
assert 'finalized_cin_available = false' in public
assert 'matched_solid' not in public and 'reader_row' not in public
owner = (here.parent/'MixedInterfaceSource.cpp').read_text()
for token in ['coated::Classify', 'd::Certify', 'f::Build', 's::BuildMixedSides', 'd::Census']:
    assert token in owner
assert 'coefficients()' not in owner
certificate = (here/'Certificate.cpp').read_text()
for token in ['NeedsNativeReaderOrder', 'role.matches != 1', 'result.raw_origins',
              'result.surface_solid_flags', 'MultipleOrigins', 'published = next']:
    assert token in certificate
assert '741' not in certificate
print(json.dumps({'status':'passed', 'scope':'mixed classified sides before support',
    'app_files':len(manifest['app_files']), 'tl_files':len(manifest['tl_files']), 'numerical_execution':False}))
