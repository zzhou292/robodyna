#!/usr/bin/env python3
"""Pin actual owner-bridge/source-report code without a source or GPU run."""
from pathlib import Path
import hashlib
import json

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here / 'source-manifest.json').read_bytes()
EXPECTED = '14e9abb3f40965d0f57c34c45608b094e71905cd6e34f73d81f1d0fcd47c28d9'
assert hashlib.sha256(raw).hexdigest() == EXPECTED
manifest = json.loads(raw)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    data = (root / path).read_bytes()
    assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path
trial_path = 'case/vehicle_dynamics/Trial.cpp'
trial = (root / trial_path).read_text()
addition = '''        if(config.structural.capture_limiter)
            detail::Require(s.owner.CopyPreparedCinStructuralLimit(token,
                &candidate().structural_limiter),"Copy actual structural limiter");
'''
assert trial.count(addition) == 1
prior = next(row['prior'] for row in manifest['files'] if row['path'] == trial_path)
assert hashlib.sha256(trial.replace(addition, '', 1).encode()).hexdigest() == prior['sha256']
report = (here.parent / 'StructuralLimiterReport.cpp').read_text()
assert 'cudaMemcpy' not in report and 'PrepareStep(' not in report and 'CommitStep(' not in report
assert report.index('limiter::MatchesAccepted(') < report.index('limiter::SelectNodes(')
assert report.index('Render(count,') < report.index('result.reserve(') < report.index('Render(write,')
print(json.dumps({'status': 'passed', 'records': len(manifest['files']),
    'numerical_execution': False, 'source_execution': False,
    'unchanged': ['force order', 'candidate evaluation', 'owner clock', 'archive schemas'],
    'growth': 'fixed observation receipt; bounded on-demand report'}))
