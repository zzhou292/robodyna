"""Exact orientation extraction and reversible dependent-drift scheduling."""
from pathlib import Path
import hashlib
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def legacy_build(current):
    """Reverse only the additive Drift.cu owner source from shared base 1d62620."""
    addition = ', "cin_advance/Drift.cu"'
    assert current.count(addition) == 1
    restored = current.replace(addition, '')
    assert hashlib.sha256(restored.encode()).hexdigest() == 'b586a6bb9ba463a00dafd0908944711faf0134816ebdf18383bbd161f2d9fedf'
    return restored


def body(text, name):
    start = text.index('{', text.index(name+'('))
    end, depth = start+1, 1
    while depth:
        depth += (text[end] == '{')-(text[end] == '}')
        end += 1
    return text[start+1:end-1]


def same(actual, expected, label):
    assert re.sub(r'\s+', '', actual) == re.sub(r'\s+', '', expected), label


def orientation_proof():
    old = (HERE/'reference/NodalNodeStep.h').read_text()
    current = (ROOT/'lib_src/solvers/NodalNodeStep.h').read_text()
    original = body(old, 'PrepareNodeOrientation')
    stores = original.index('  auto* q=trial+9*n+4*i;')
    same(body(current, 'PrepareNodeOrientationValue'),
         original[:stores]+'output=candidate; return NodalStatus::Ok;',
         'complete increment/angle-before-quaternion/native computation, success-only value')
    same(body(current, 'PrepareNodeOrientation'), '''
      tl::math::Quaternion candidate;
      const auto status=PrepareNodeOrientationValue(accepted,trial,i,n,h,maximum_angle,fixed_rotation,candidate);
      if(status!=NodalStatus::Ok) return status;
    '''+original[stores:], 'wrapper keeps exactly the same four success stores')
    same(body(current, 'AdvanceOrdinaryNode'), body(old, 'AdvanceOrdinaryNode'),
         'no ordinary force/acceleration/rotation admission change')
    assert (ROOT/'lib_src/solvers/cin_advance/RecoveryDrift.h').read_bytes() == (HERE/'reference/RecoveryDrift.h').read_bytes()


def legacy_owner(current):
    orientation_proof()
    kernels = (ROOT/'lib_src/solvers/cin_advance/Recovery.cu').read_text()
    old = (HERE/'reference/Recovery.cu').read_text()
    kernels = kernels.replace('#include "DriftValues.h"\n', '')
    kernels = kernels.replace('Complete(Input input, bool defer_drift)', 'Complete(Input input)')
    kernels = kernels.replace('  if (!defer_drift) Drift(input);', '  Drift(input);')
    addition = '''  const bool parallel_drift = input.prepared_drift != nullptr;
  Complete<<<1, 1, 0, stream>>>(input, parallel_drift);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  return parallel_drift ? drift::Launch(input, stream) : cudaSuccess;'''
    assert kernels.count(addition) == 1
    kernels = kernels.replace(addition, '''  Complete<<<1, 1, 0, stream>>>(input);
  return cudaGetLastError();''')
    assert kernels == old, 'all recovery operations/error precedence and null-tail serial route unchanged'
    addition = 'cin->recovery_failure, cin->prepared_drift}, stream);'
    assert current.count(addition) == 1
    return current.replace(addition, 'cin->recovery_failure}, stream);')


if __name__ == '__main__':
    restored = legacy_owner((ROOT/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text())
    assert restored == (HERE/'reference/ExplicitNodalCinStep.cu').read_text()
