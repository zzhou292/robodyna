#!/usr/bin/env python3
"""Exact owning source and complete serial-predicate/fold preservation."""
from pathlib import Path
import hashlib
import json

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MANIFEST_SHA256 = '63e6f0af96ac93fa1ff9d5cef42b09ba5a332a55022ac8dd0b040bbfe053607c'


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
    new = (prefix / 'Measure.h').read_text()
    expected = body(old, 'bool MeasureFamily(').replace(
        '!ValidResult(parent,\n'
        '        MaterialAt<Traits>(state, parent.material_index), now, diagnostics.time, diagnostics.epoch)',
        '!check(p, diagnostics.time, diagnostics.epoch)')
    assert body(new, 'bool MeasureFamilyWithCheck(') == expected
    old = (HERE / 'frozen/Candidate.cu.txt').read_text()
    new = (prefix / 'Candidate.cu').read_text()
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
