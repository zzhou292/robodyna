#!/usr/bin/env python3
"""Owning source identity plus composed serial, resident, and LAW44 proofs."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

from operand_proof import verify as verify_operand_proof
from integration_proof import verify as verify_integration_proof

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MANIFEST_SHA256 = 'd0baa2ba4c6b7a0dce72d56c61e5909c65081bd797a86d18d7991b17226d1319'
COMPOSED = [
    ROOT / 'lib_utest/qualification/solid_candidate_validation/verify_sources.py',
    ROOT / 'lib_utest/qualification/extended_solid_resident/verify_sources.py',
    ROOT / 'lib_utest/qualification/solid_law44_analytic/verify_source.py',
]


def verify():
    raw = (HERE / 'source-manifest.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == MANIFEST_SHA256
    manifest = json.loads(raw)
    paths = [row['path'] for row in manifest['files']]
    assert len(paths) == len(set(paths)), 'duplicate owning source record'
    for row in manifest['files']:
        path = Path(row['path'])
        assert not path.is_absolute() and '..' not in path.parts
        data = (ROOT / path).read_bytes()
        assert len(data) == row['bytes'], path
        assert hashlib.sha256(data).hexdigest() == row['sha256'], path
    proof = verify_operand_proof()
    integration = verify_integration_proof()
    for verifier in COMPOSED:
        subprocess.run([sys.executable, '-B', str(verifier)], cwd=ROOT, check=True)
    return {
        'status': 'passed',
        'records': len(paths),
        'baseline_commit': manifest['baseline_commit'],
        'checked_extraction_and_complete_reversals': all([
            proof['complete_measure_reversal'],
            proof['complete_candidate_reversal'],
            proof['complete_validation_reversal'],
            proof['literal_work_extraction'],
            proof['literal_ordered_operand_fold'],
            proof['conditional_fresh_producer'],
        ]),
        'composed_candidate_validation': True,
        'composed_extended_resident_donors': True,
        'composed_law44_analytic_donors': True,
        'current_storage_baseline': integration['current_baseline'],
        'complete_storage_reversals': integration['complete_storage_reversals'],
        'numerical_execution': False,
    }


if __name__ == '__main__':
    print(json.dumps(verify(), sort_keys=True))
