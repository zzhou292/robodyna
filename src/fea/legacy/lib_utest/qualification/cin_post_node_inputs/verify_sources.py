"""Prove independent leaves and serial precedence against the frozen full header."""
from pathlib import Path
import re
HERE = Path(__file__).resolve().parent

def body(text, name):
    start = text.index('{', text.index(name+'('))
    end, depth = start+1, 1
    while depth:
        depth += (text[end] == '{')-(text[end] == '}')
        end += 1
    return text[start+1:end-1]

def same(a, b, label):
    assert re.sub(r'\s+', '', a) == re.sub(r'\s+', '', b), label

def legacy_force_stage(current):
    old = (HERE/'CinForceStage.baseline.txt').read_text()
    checks = body(old, 'CheckForceAfterNodes')
    witness_start = checks.index('    if (model.first_witness')
    witness_end = checks.index('  for (std::uint32_t r')
    witness = checks[witness_start:witness_end].rsplit('  }', 1)[0]
    row = checks[checks.index('    const auto row = model.rows[r];'):checks.rfind('  }')]
    same(body(current, 'CheckForceWitness'), witness+'return {};', 'complete witness leaf')
    same(body(current, 'CheckForceRow'), row+'return {};', 'complete row leaf')
    same(body(current, 'CheckForceAfterNodes'), """
      if (!tied_shell::detail::math::Finite(*trial.numerical_mass)) return {StageStatus::InvalidInput};
      for (std::uint32_t w = 0; w < model.witness_count; ++w) {
        const auto report = CheckForceWitness(model, trial, w);
        if (!report) return report;
      }
      for (std::uint32_t r = 0; r < model.row_count; ++r) {
        const auto report = CheckForceRow(model, trial, r);
        if (!report) return report;
      }
      return {};
    """, 'serial numerical mass, witness and row order')
    return current.replace(body(current, 'CheckForceAfterNodes'), checks)

if __name__ == '__main__':
    import hashlib, json
    root = HERE.parents[2]
    baseline = (HERE/'CinForceStage.baseline.txt').read_bytes()
    assert hashlib.sha256(baseline).hexdigest() == '3f0c63195bc8f60aedddfec71ceb0b9cd7f8f4272a88a5c7bcefaed305b7aa50'
    current = (root/'lib_src/constraints/tied_shell/runtime/CinForceStage.h').read_text()
    restored = legacy_force_stage(current)
    for name in ('CheckForcePointers', 'CheckForceNode', 'CheckForceAfterNodes',
                 'CheckForceInputs', 'TransferForceTrial', 'PrepareForceTrial'):
        same(body(restored, name), body(baseline.decode(), name), name)
    kernel = (root/'lib_src/solvers/cin_advance/ForceInputs.cu').read_text()
    launch = body(kernel, 'Launch')
    names = ('BeginInputs<<<', 'CheckNodes<<<', 'CompleteNodeInputs<<<', 'CheckWitnesses<<<',
             'CompleteWitnessInputs<<<', 'CheckRows<<<', 'CompleteInputs<<<', 'CopyEntryInertia<<<')
    locations = [launch.index(name) for name in names]
    assert locations == sorted(locations)
    assert 'atomicAdd' not in kernel and 'cudaMalloc' not in kernel
    for name in ('CheckNodes', 'CheckWitnesses', 'CheckRows', 'CopyEntryInertia'):
        value = body(kernel, name)
        assert value.index('input.control->status != NodalStatus::Ok') < value.index('const auto force')
    print(json.dumps({'status':'passed', 'baseline':'90a4a806', 'scope':'exact leaf extraction and phase order', 'numerical_execution':False}))
