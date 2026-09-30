"""Exact recovery extraction and reversible scheduling for prior source gates."""
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def body(text, name):
    start = text.index('{', text.index(name+'('))
    end, depth = start+1, 1
    while depth:
        depth += (text[end] == '{')-(text[end] == '}')
        end += 1
    return text[start+1:end-1]


def same(actual, expected, label):
    assert re.sub(r'\s+', '', actual) == re.sub(r'\s+', '', expected), label


def motion_proof():
    old = (HERE/'reference/CinMotionStage.h').read_text()
    helper = (ROOT/'lib_src/constraints/tied_shell/runtime/CinMotionRows.h').read_text()
    same(body(helper, 'WriteXyz'), body(old, 'WriteXyz'), 'all three stores unchanged')
    original = body(old, 'RecoverMotionTrial')
    start = original.index('    const auto row = model.rows[r];')
    write = original.index('    detail::WriteXyz')
    end = original.rfind('\n  }')
    same(body(helper, 'PrepareMotionRow'), original[start:write]+'''output = recovered;
        return {};''', 'complete native four-slot leaf, read order and failure')
    same(body(helper, 'ApplyMotionRow'), 'const auto row = model.rows[r];'+original[write:end],
         'complete four secondary vectors, unchanged store expressions/order')
    same(body(helper, 'MotionPointersValid'), '''return model.rows && trial.patches &&
        trial.velocity_xyz && trial.angular_velocity_xyz && trial.acceleration_xyz &&
        trial.angular_acceleration_xyz;''', 'exact header short-circuit order')
    current = (ROOT/'lib_src/constraints/tied_shell/runtime/CinMotionStage.h').read_text()
    same(body(current, 'RecoverMotionTrial'), '''
      if (!detail::MotionPointersValid(model, trial)) return {StageStatus::InvalidInput};
      for (std::uint32_t r = 0; r < model.row_count; ++r) {
        SecondaryMotion recovered;
        const auto report = detail::PrepareMotionRow(model, trial, r, recovered);
        if (!report) return report;
        detail::ApplyMotionRow(model, trial, r, recovered);
      }
      return {};
    ''', 'legacy wrapper remains sequential and retains failure prefixes')
    assert (ROOT/'lib_src/constraints/tied_shell/TiedPatchMotion.h').read_bytes() == (HERE/'reference/TiedPatchMotion.h').read_bytes()


def legacy_owner(current):
    import runpy
    drift = runpy.run_path(str(HERE.parent/'cin_parallel_drift/drift_proof.py'))
    current = drift['legacy_owner'](current)
    motion_proof()
    old = (HERE/'reference/ExplicitNodalCinStep.cu').read_text()
    start = old.index('  for (std::uint32_t row = 0; row < r; ++row) {', old.index('__global__ void CompleteCin'))
    end = old.index('\n\n}', start)
    drift = old[start:end]
    helper = (ROOT/'lib_src/solvers/cin_advance/RecoveryDrift.h').read_text()
    same(body(helper, 'Drift'), '''
      const auto model = input.model;
      const auto n = model.node_count;
      const auto r = model.row_count;
      const auto* accepted = input.accepted;
      auto* trial = input.trial;
      const auto durations = input.durations;
      const auto maximum_angle = input.maximum_angle;
    '''+drift.replace('Fail(control,', 'Fail(input,'), 'entire dependent drift/orientation loop')
    same(body(helper, 'Fail'), body(old, 'Fail').replace('control->', 'input.control->'), 'same nodal error mapping')
    value = current.replace('#include "cin_advance/Recovery.h"\n', '')
    value = value.replace('#include "cin_advance/RecoveryDrift.h"\n', '')
    value = value.replace('bool groups_prepared = false,\n    bool defer_recovery = false)', 'bool groups_prepared = false)')
    value = value.replace('  if (defer_recovery) return;\n', '')
    value = value.replace('  cin_advance::recovery::Drift(input);', drift)
    addition = '  const bool parallel_recovery = input.prepared_recovery && input.recovery_failure && input.model.row_count;\n'
    assert value.count(addition) == 1
    value = value.replace(addition, '')
    value = value.replace('(input, parallel_groups, parallel_recovery);', '(input, parallel_groups);')
    addition = '''  if (parallel_recovery) {
    error = recovery::Launch(input, stream);
    if (error != cudaSuccess) return error;
  }
'''
    assert value.count(addition) == 1
    value = value.replace(addition, '')
    value = value.replace('cin->prepared_transfers, cin->prepared_recovery, cin->recovery_failure}, stream);',
                          'cin->prepared_transfers}, stream);')
    assert value == old, 'complete old force/screen/ordinary/rigid/drift/owner scheduling view'
    return value


if __name__ == '__main__':
    import runpy
    witness = runpy.run_path(str(HERE.parent/'cin_limiter/witness_proof.py'))
    legacy_owner(witness['legacy_owner']((ROOT/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text()))
