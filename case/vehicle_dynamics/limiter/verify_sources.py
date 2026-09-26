#!/usr/bin/env python3
"""Pin actual owner-bridge/source-report code without a source or GPU run."""
from pathlib import Path
import hashlib
import json

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here / 'source-manifest.json').read_bytes()
EXPECTED = 'deebda9e0712c575dc0b632ae68bed268140177c605f6fec8015ade70c180fd9'
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
# The independently qualified observation is now a separate tail. Preserve
# the original limiter/force/evaluate proof over the byte-identical prefix;
# the complete current Trial remains hash-pinned above.
prefix = trial.split('void VehiclePhysicalDynamics::Storage::Capture() {', 1)[0]
restored = prefix.replace(addition, '', 1).encode()
review = manifest['reviewed_motion_observation']
assert len(restored) == review['unchanged_physics_prefix_bytes']
assert hashlib.sha256(restored).hexdigest() == review['unchanged_physics_prefix_sha256']
# The compile correction changes only access to the actual counted-view APIs.
# Retain and authenticate the original report/source bodies as prior evidence.
counted_access = {
    'case/vehicle_dynamics/limiter/SourceNodes.h': (
        '    for(std::size_t i = 0; i < cin.count; ++i) {\n'
        '        const auto& row = cin.data[i];',
        '    for(const auto& row : cin) {'),
    'case/vehicle_dynamics/limiter/SourceRows.h': (
        '        for(std::size_t i = 0; i < type25->connection_count(); ++i) {\n'
        '            const auto& row = type25->connections()[i];',
        '        for(const auto& row : type25->connections()) {'),
    'case/vehicle_dynamics/StructuralLimiterReport.cpp': (
        '        const auto& row = rows.data[i];',
        '        const auto& row = rows[i];'),
}
for path, (current, original) in counted_access.items():
    source = (root / path).read_text()
    assert source.count(current) == 1, path
    restored = source.replace(current, original, 1).encode()
    evidence = next(row['prior'] for row in manifest['files'] if row['path'] == path)
    assert len(restored) == evidence['bytes'], path
    assert hashlib.sha256(restored).hexdigest() == evidence['sha256'], path
test_path = 'case/vehicle_run/tests/LimiterTest.cpp'
test = (root / test_path).read_text()
full_precision = 'document.Parse<rapidjson::kParseFullPrecisionFlag>(first.data(),first.size());'
assert test.count(full_precision) == 1
restored = test.replace(full_precision, 'document.Parse(first.c_str());', 1).encode()
evidence = next(row['prior'] for row in manifest['files'] if row['path'] == test_path)
assert len(restored) == evidence['bytes']
assert hashlib.sha256(restored).hexdigest() == evidence['sha256']
report = (here.parent / 'StructuralLimiterReport.cpp').read_text()
assert 'cudaMemcpy' not in report and 'PrepareStep(' not in report and 'CommitStep(' not in report
assert report.index('limiter::MatchesAccepted(') < report.index('limiter::SelectNodes(')
assert report.index('Render(count,') < report.index('result.reserve(') < report.index('Render(write,')
print(json.dumps({'status': 'passed', 'records': len(manifest['files']),
    'numerical_execution': False, 'source_execution': False,
    'unchanged': ['force order', 'candidate evaluation', 'owner clock', 'archive schemas'],
    'growth': 'fixed observation receipt; bounded on-demand report'}))
