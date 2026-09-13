#!/usr/bin/env python3
"""Exact additive composition with the current ordered solid assembly storage."""
from pathlib import Path
import hashlib
import json
from operand_proof import once
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BASELINE_SHA256 = '0ff7b0455dd5b2a438a35e0e423d6248e259c429bf0c9552f239f825fc7b9328'

def verify():
    raw = (HERE / 'integration-baseline.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == BASELINE_SHA256
    baseline = json.loads(raw)
    assert baseline['commit'] == '90a4a8060966358714049a775e4f9f7e7838481e'
    assert {Path(row['path']).name for row in baseline['files']} == {'Arena.h', 'Arena.cpp', 'Storage.h', 'Upload.cpp'}
    for row in baseline['files']:
        frozen = (HERE / row['fixture']).read_bytes()
        assert len(frozen) == row['bytes'] and hashlib.sha256(frozen).hexdigest() == row['sha256']
        current = (ROOT / row['path']).read_text()
        name = Path(row['path']).name
        if name == 'Arena.h':
            current = once(current, '#include "MeasurementTypes.h"\n', '')
            current = once(current, '  MeasurementOperands<Traits::nodes>* measurement = nullptr;\n', '')
            current = once(current, 'status, staging, result_valid, measurement;', 'status, staging, result_valid;')
        elif name == 'Arena.cpp':
            current = once(current, '      device.Append<MeasurementOperands<Traits::nodes>>(count, output.measurement) &&\n', '')
            current = once(current, '      util::ArenaPointer<std::uint8_t>(base, layout.result_valid),\n'
                '      util::ArenaPointer<MeasurementOperands<Traits::nodes>>(base, layout.measurement)};',
                '      util::ArenaPointer<std::uint8_t>(base, layout.result_valid)};')
        elif name == 'Storage.h':
            current = once(current, 'void LaunchMeasurementValidation(Storage*, unsigned accepted, unsigned trial,\n'
                '    NodalPreparedView, double time, std::uint64_t epoch, bool initial, cudaStream_t);\n', '')
        else:
            current = once(current, '      !arena.Construct<std::uint8_t>(layout.result_valid) ||\n'
                '      !arena.Construct<MeasurementOperands<Traits::nodes>>(layout.measurement)) return false;',
                '      !arena.Construct<std::uint8_t>(layout.result_valid)) return false;')
        assert current.encode() == frozen, row['path']
    return {'status': 'passed', 'current_baseline': baseline['commit'],
            'complete_storage_reversals': True, 'numerical_execution': False}

if __name__ == '__main__':
    print(json.dumps(verify(), sort_keys=True))
