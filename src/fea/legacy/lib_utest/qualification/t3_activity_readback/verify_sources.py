#!/usr/bin/env python3
"""Exact current ownership plus complete baseline/extraction verification."""
from pathlib import Path
import hashlib
import json
from generate_flows import body, generated

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MANIFEST_SHA256 = '576c9c772caf9116fff610b2d25e01a4911a03b01f31e041edd4844966de58cf'


def verify():
    raw = (HERE / 'source-manifest.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == MANIFEST_SHA256
    manifest = json.loads(raw)
    for row in manifest['files']:
        path = Path(row['path'])
        assert not path.is_absolute() and '..' not in path.parts
        value = (ROOT / path).read_bytes()
        assert len(value) == row['bytes'], path
        assert hashlib.sha256(value).hexdigest() == row['sha256'], path
    for name, value in generated().items():
        assert (HERE / name).read_text() == value, name
    for current, frozen in [
            ('lib_src/elements/t3/mapped/Readback.cpp', 'FrozenMapped.cpp.txt'),
            ('lib_src/elements/t3/T3Batch.cu', 'FrozenBatch.cu.txt'),
            ('lib_src/elements/failure/ShellFailureReadback.h', 'FrozenReadFailure.h.txt')]:
        assert (ROOT / current).read_bytes() == (HERE / frozen).read_bytes()
    old = (HERE / 'FrozenStorage.h.txt').read_text()
    declaration = '  BatchReport ValidateOnePointReadback(unsigned slab,double time,std::uint64_t epoch);\n'
    assert old.count(declaration) == 1
    assert (ROOT / 'lib_src/elements/t3/T3BatchStorage.h').read_text() == old.replace(
        declaration, declaration + '  BatchReport ReadParentActivity(unsigned slab,double time,std::uint64_t epoch);\n')
    old = (HERE / 'FrozenFailure.cpp.txt').read_text()
    first = old.index('  const auto report = shell_batch_plasticity_detail::ReadFailure(state,',
                      old.index('BatchReport T3Batch::CopyAcceptedParentActivity'))
    last = old.index('  const auto* history =', first)
    expected = old[:first] + (
        '  const auto report = state.ReadParentActivity(state.AcceptedSlabIndex(),\n'
        '      state.accepted_diagnostics.time, state.accepted_stamp.epoch);\n'
        '  if (report.status != BatchStatus::Success) return report;\n') + old[last:]
    first = expected.index('  report = shell_batch_plasticity_detail::ReadFailure(state, slab, expected.time);')
    last = expected.index('  if (report.status != BatchStatus::Success) {', first)
    expected = expected[:first] + '  report = state.ReadParentActivity(slab, expected.time, expected.epoch);\n' + expected[last:]
    assert (ROOT / 'lib_src/elements/t3/T3BatchFailureReadback.cpp').read_text() == expected
    current = (ROOT / 'lib_src/elements/t3/T3BatchOnePointReadback.cpp').read_text()
    assert body(current, 'BatchReport T3Batch::Impl::ValidateOnePointReadback') == (
        '{\n  return ReadOnePoint(*this, slab, time, epoch,\n'
        '      [&] { return ReadResults(&storage->slab[slab]); });\n}')
    return {'status': 'passed', 'records': len(manifest['files']),
            'complete_public_preflight_and_output_unchanged': True,
            'exact_full_one_point_loop_extraction': True,
            'mapped_transport_and_full_read_failure_unchanged': True,
            'numerical_execution': False}


if __name__ == '__main__':
    print(json.dumps(verify(), sort_keys=True))
