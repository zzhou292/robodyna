#!/usr/bin/env python3
"""Exact owning source and complete serial-predicate/fold preservation."""
from pathlib import Path
import hashlib
import json
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MANIFEST_SHA256 = 'a6ac531d9ff3200a6dd68878170a74debec4dcaf5c4c11c844384d3bbdc6f501'
sys.path.insert(0, str(HERE.parent / 'solid_measurement_operands'))
from operand_proof import (authenticate_baseline, legacy_candidate, legacy_measure,
                           legacy_result_validation)


def body(text, signature):
    first = text.index('{', text.index(signature))
    depth = 0
    for end in range(first, len(text)):
        depth += text[end] == '{'
        depth -= text[end] == '}'
        if depth == 0:
            return text[first:end + 1]
    raise ValueError('incomplete body: ' + signature)


def verify():
    authenticate_baseline()
    raw = (HERE / 'source-manifest.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == MANIFEST_SHA256
    manifest = json.loads(raw)
    for row in manifest['files']:
        path = Path(row['path'])
        assert not path.is_absolute() and '..' not in path.parts
        data = (ROOT / path).read_bytes()
        assert len(data) == row['bytes'], path
        assert hashlib.sha256(data).hexdigest() == row['sha256'], path
    prefix = ROOT / 'lib_src/elements/solids/resident'
    assert (prefix / 'ResultChecks.h').read_bytes() == (HERE / 'frozen/ResultChecks.h.txt').read_bytes()
    old = (HERE / 'frozen/Measure.h.txt').read_text()
    new = legacy_measure((prefix / 'Measure.h').read_text())
    expected = body(old, 'bool MeasureFamily(').replace(
        '!ValidResult(parent,\n'
        '        MaterialAt<Traits>(state, parent.material_index), now, diagnostics.time, diagnostics.epoch)',
        '!check(p, diagnostics.time, diagnostics.epoch)')
    assert body(new, 'bool MeasureFamilyWithCheck(') == expected
    old = (HERE / 'frozen/Candidate.cu.txt').read_text()
    new = legacy_candidate((prefix / 'Candidate.cu').read_text())
    legacy_result_validation((prefix / 'ResultValidation.cu').read_text())
    for signature in ['void Initialize(', 'void Evaluate(']:
        assert body(new, signature) == body(old, signature), signature
    assert body(new, 'void Finalize(') == body(old, 'void Finalize(').replace(
        'MeasureFamily<', 'MeasureValidatedFamily<')
    for signature, insertion in [
        ('void LaunchInitialize(', '  LaunchResultValidation(storage, 0, 0, 0, stream);\n'
         '  if (cudaPeekAtLastError() != cudaSuccess) return;\n'),
        ('void LaunchCandidate(', '  LaunchResultValidation(storage, trial, identity.time, identity.epoch, view.stream);\n'
         '  if (cudaPeekAtLastError() != cudaSuccess) return;\n')]:
        expected = body(old, signature).replace('  Finalize<<<', insertion + '  Finalize<<<')
        assert body(new, signature) == expected, signature
    return {'status': 'passed', 'records': len(manifest['files']),
            'full_valid_result_unchanged': True, 'ordered_fold_unchanged': True,
            'initialize_evaluate_unchanged': True, 'numerical_execution': False}


if __name__ == '__main__':
    print(json.dumps(verify(), sort_keys=True))
