"""Checked serial compatibility views. No runtime or numerical test substitute."""
from pathlib import Path
import re
import runpy

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


def legacy_force_stage(current):
    old = (HERE/'reference/CinForceStage.h').read_text()
    helper = (ROOT/'lib_src/constraints/tied_shell/runtime/CinForceTransfer.h').read_text()
    for name in ('ReadXyz', 'Nonnegative'):
        same(body(helper, name), body(old, name), name)
    for name in ('CheckForcePointers', 'CheckForceNode', 'CheckForceAfterNodes',
                 'CheckForceInputs', 'PrepareForceTrial'):
        same(body(current, name), body(old, name), name)
    original = body(old, 'TransferForceTrial')
    start = original.index('    const auto row = model.rows[r];')
    split = original.index('    // Native NSV order')
    end = original.rfind('\n  }')
    same(body(helper, 'PrepareForceRow'), 'const auto n = model.node_count;'+original[start:split]+'''
      output.patch = patch;
      output.transferred_load = transferred_load;
      output.transferred_coefficients = transferred_coefficients;
      output.secondary_mass = coefficients.secondary.mass;
      return {};
    ''', 'complete pure row prefix, exact source-slot expressions')
    apply = original[split:end].replace('coefficients.secondary.mass', 'secondary_mass')
    same(body(helper, 'ApplyForceRow'), '''
      if (!prepared.report) return prepared.report;
      const auto n = model.node_count;
      const auto row = model.rows[r];
      const auto secondary = row.secondary;
      const auto& patch = prepared.patch;
      const auto& transferred_load = prepared.transferred_load;
      const auto& transferred_coefficients = prepared.transferred_coefficients;
      const auto secondary_mass = prepared.secondary_mass;
    '''+apply+'return {};', 'complete ordered apply and partial-error priority')
    same(body(current, 'TransferForceTrial'), '''
      for (std::uint32_t r = 0; r < model.row_count; ++r) {
        PreparedForceRow prepared;
        prepared.report = PrepareForceRow(model, trial, r, prepared);
        const auto report = ApplyForceRow(model, trial, r, prepared);
        if (!report) return report;
      }
      return {};
    ''', 'legacy wrapper remains sequential')
    return old


def legacy_owner(current):
    recovery = runpy.run_path(str(HERE.parent/"cin_parallel_recovery/recovery_proof.py"))
    current = recovery["legacy_owner"](current)
    value = current.replace('#include "cin_advance/ForceTransfers.h"\n', '')
    value = value.replace('bool inputs_prepared, bool parallel_screen,\n    bool transfers_prepared = false)',
                          'bool inputs_prepared, bool parallel_screen)')
    value = value.replace('transfers_prepared ? cin_advance::force_transfers::Apply(input)\n      : inputs_prepared ?',
                          'inputs_prepared ?')
    addition = '''  const bool parallel_transfers = parallel_inputs && input.prepared_transfers;
  if (parallel_transfers) {
    error = force_transfers::Launch(input, stream);
    if (error != cudaSuccess) return error;
  }
'''
    assert value.count(addition) == 1
    value = value.replace(addition, '')
    value = value.replace('(input, parallel_inputs, parallel_screen, parallel_transfers);',
                          '(input, parallel_inputs, parallel_screen);')
    value = value.replace('cin->screen, cin->group_reports,\n      cin->prepared_transfers}, stream)',
                          'cin->screen, cin->group_reports}, stream)')
    assert value == (HERE/'reference/ExplicitNodalCinStep.cu').read_text(), 'exact scheduling-only owner delta'
    return value


if __name__ == '__main__':
    import runpy
    witness = runpy.run_path(str(HERE.parent/'cin_limiter/witness_proof.py'))
    legacy_force_stage((ROOT/'lib_src/constraints/tied_shell/runtime/CinForceStage.h').read_text())
    legacy_owner(witness['legacy_owner']((ROOT/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text()))
