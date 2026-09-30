#!/usr/bin/env python3
"""Checked edab8e1 reversals and literal measurement-operand source proofs."""
from pathlib import Path
import hashlib
import json
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BASELINE_SHA256 = 'd1f4948f38aca191d59892f1ae2e5ebec774ba4b6b810a0743f0ae1fcfa66e82'
EXPECTED_BASELINE = {
    'lib_src/elements/solids/resident/Measure.h',
    'lib_src/elements/solids/resident/Candidate.cu',
    'lib_src/elements/solids/resident/Arena.h',
    'lib_src/elements/solids/resident/Arena.cpp',
    'lib_src/elements/solids/resident/ResultValidation.cu',
}


def body(text, signature):
    begin = text.index('{', text.index(signature))
    depth = 0
    for end in range(begin, len(text)):
        depth += text[end] == '{'
        depth -= text[end] == '}'
        if depth == 0:
            return text[begin:end + 1]
    raise AssertionError('incomplete body: ' + signature)


def statement(text, signature):
    begin = text.index(signature)
    finish = begin + len(signature)
    while finish < len(text) and text[finish].isspace():
        finish += 1
    if finish >= len(text) or text[finish] != '{':
        raise AssertionError('statement has no body: ' + signature)
    block = body(text[begin:], signature)
    return text[begin:begin + text[begin:].index(block) + len(block)]


def tokens(text):
    text = re.sub(r'//[^\n]*', '', text)
    return re.sub(r'\s+', '', text)


def once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


def counted(text, old, new, count):
    assert text.count(old) == count, (old, text.count(old), count)
    return text.replace(old, new)


def authenticate_baseline():
    raw = (HERE / 'baseline.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == BASELINE_SHA256
    baseline = json.loads(raw)
    assert baseline['commit'] == 'edab8e17e6d6e641f77f093fdb258d98152db492'
    assert {row['path'] for row in baseline['files']} == EXPECTED_BASELINE
    assert len(baseline['files']) == len(EXPECTED_BASELINE)
    for row in baseline['files']:
        fixture = Path(row['fixture'])
        assert not fixture.is_absolute() and '..' not in fixture.parts
        data = (HERE / fixture).read_bytes()
        assert len(data) == row['bytes'], fixture
        assert hashlib.sha256(data).hexdigest() == row['sha256'], fixture
    return baseline


def legacy_measure(text):
    """Reverse only the reviewed include and extracted accepted-work loop."""
    old = (HERE / 'frozen/Measure.h.txt').read_text()
    restored = once(text, '#include "MeasurementWork.h"',
                    '#include "../../ShellBatchFields.h"')
    call = '''      AccumulateMeasurementWork<Traits>(parent, family.slab[accepted][p].cache, *view,
          diagnostics.internal_kick_work_j, diagnostics.internal_drift_work_j);'''
    old_loop = '      ' + statement(old, 'for (unsigned n = 0; n < Traits::nodes; ++n)')
    restored = once(restored, call, old_loop)
    assert restored == old, 'complete Measure.h reversal'
    return restored


def legacy_candidate(text):
    """Reverse only operand dispatch and launch plumbing to complete edab8e1."""
    old = (HERE / 'frozen/Candidate.cu.txt').read_text()
    restored = once(text, '#include "MeasurementValues.h"', '#include "Measure.h"')
    restored = once(restored,
        '''    NodalPreparedView view, BatchDiagnostics identity, bool initial, bool operands_prepared = false) {''',
        '''    NodalPreparedView view, BatchDiagnostics identity, bool initial) {''')
    traits = ['Traits18', 'Traits24', 'Traits6z', 'Traits18Law44', 'Traits18Law90']
    for index, trait in enumerate(traits):
        restored = once(restored,
            f'MeasureFinalFamily<{trait}>(state, accepted, trial, {index}, prepared, operands_prepared)',
            f'MeasureValidatedFamily<{trait}>(state, accepted, trial, {index}, prepared)')
    restored = once(restored,
        'LaunchMeasurementValidation(storage, 0, 0, {}, 0, 0, true, stream);',
        'LaunchResultValidation(storage, 0, 0, 0, stream);')
    restored = once(restored,
        'Finalize<<<1, 1, 0, stream>>>(storage, 0, 0, {}, {}, true, true);',
        'Finalize<<<1, 1, 0, stream>>>(storage, 0, 0, {}, {}, true);')
    restored = once(restored,
        'LaunchMeasurementValidation(storage, accepted, trial, view, identity.time, identity.epoch, false, view.stream);',
        'LaunchResultValidation(storage, trial, identity.time, identity.epoch, view.stream);')
    restored = once(restored,
        'Finalize<<<1, 1, 0, view.stream>>>(storage, accepted, trial, view, identity, false, true);',
        'Finalize<<<1, 1, 0, view.stream>>>(storage, accepted, trial, view, identity, false);')
    assert restored == old, 'complete Candidate.cu reversal'
    return restored


def legacy_result_validation(text):
    """Reverse the optional producer branch while preserving the old full predicate loop."""
    old = (HERE / 'frozen/ResultValidation.cu.txt').read_text()
    restored = once(text, '#include "MeasurementValues.h"', '#include "ResultValidation.h"')
    restored = once(restored,
        '''template<class Traits> __global__ void ValidateResults(Storage* storage, unsigned trial,
    double time, std::uint64_t epoch, unsigned accepted, NodalPreparedView view,
    bool initial, bool measurements) {''',
        '''template<class Traits> __global__ void ValidateResults(Storage* storage, unsigned trial,
    double time, std::uint64_t epoch) {''')
    restored = once(restored,
        '''    if (measurements) {
      PrepareMeasurementOperands<Traits>(*storage, accepted, trial, parent, time, epoch,
          initial ? nullptr : &view);
    } else {
      family.result_valid[parent] = CheckParentResult<Traits>(*storage, trial, parent, time, epoch);
    }''',
        '''    family.result_valid[parent] = CheckParentResult<Traits>(*storage, trial, parent, time, epoch);''')
    launch = body(restored, 'void LaunchValidation(')
    launch = counted(launch, ', accepted, view, initial, measurements)', ')', 5)
    begin = restored.index('void LaunchValidation(')
    measurement = restored.index('void LaunchMeasurementValidation(')
    end = measurement + restored[measurement:].index(
        body(restored[measurement:], 'void LaunchMeasurementValidation(')) + len(
            body(restored[measurement:], 'void LaunchMeasurementValidation('))
    replacement = '''} // namespace

void LaunchResultValidation(Storage* storage, unsigned trial, double time,
    std::uint64_t epoch, cudaStream_t stream) ''' + launch
    restored = restored[:begin] + replacement + restored[end:]
    assert restored == old, 'complete ResultValidation.cu reversal'
    return restored


def verify_work_extraction():
    old = (HERE / 'frozen/Measure.h.txt').read_text()
    work = (ROOT / 'lib_src/elements/solids/resident/MeasurementWork.h').read_text()
    extracted = statement(work, 'for (unsigned n = 0; n < Traits::nodes; ++n)')
    extracted = counted(extracted, 'view.', 'view->', 5)
    extracted = once(extracted, 'accepted.rhs_force_n[n]',
                     'family.slab[accepted][p].cache.rhs_force_n[n]')
    extracted = once(extracted, 'kick +=',
                     'diagnostics.internal_kick_work_j +=')
    extracted = once(extracted, 'drift +=',
                     'diagnostics.internal_drift_work_j +=')
    original = statement(old, 'for (unsigned n = 0; n < Traits::nodes; ++n)')
    assert tokens(extracted) == tokens(original), 'literal accepted-work extraction'


def verify_operand_fold():
    old = (HERE / 'frozen/Measure.h.txt').read_text()
    values = (ROOT / 'lib_src/elements/solids/resident/MeasurementValues.h').read_text()
    expected = body(old, 'bool MeasureFamilyWithCheck(')
    expected = once(expected,
        '''    const auto& parent = family.parents[p];
    const auto& now = family.slab[trial][p];
''', '')
    expected = once(expected,
        '''    if (family.status[p] != 0 || !check(p, diagnostics.time, diagnostics.epoch)) {''',
        '''    if (family.status[p] != 0 || family.result_valid[p] != 1) {''')
    expected = once(expected, '    const auto& cache = now.cache;',
                    '    const auto& value = family.measurement[p];')
    expected = once(expected,
        '    diagnostics.native_internal_work_increment_j[family_index] += Work(cache);',
        '    diagnostics.native_internal_work_increment_j[family_index] += value.work;')
    expected = once(expected,
        '    diagnostics.physical_hourglass_work_increment_j[family_index] += HourglassWork(cache);',
        '    diagnostics.physical_hourglass_work_increment_j[family_index] += value.hourglass_work;')
    expected = once(expected,
        '    diagnostics.plastic_work_increment_j += PlasticWork(cache);',
        '    diagnostics.plastic_work_increment_j += value.plastic_work;')
    expected = counted(expected, 'NativeDt(cache)', 'value.native_dt', 2)
    expected = once(expected,
        '''    if (view) {
      for (unsigned n = 0; n < Traits::nodes; ++n) {
        namespace fields = shell_batch_fields;
        const auto node = parent.domain_nodes[n];
        const auto v0 = fields::ReadVector(view->base_kinematics.velocity_xyz, node);
        const auto v1 = fields::ReadVector(view->kinematics.velocity_xyz, node);
        const auto dx = fields::Difference(fields::ReadVector(view->kinematics.position_xyz, node),
            fields::ReadVector(view->base_kinematics.position_xyz, node));
        const auto rhs = family.slab[accepted][p].cache.rhs_force_n[n];
        diagnostics.internal_kick_work_j += view->kick_dt * fields::Dot(rhs, fields::Mean(v0, v1));
        diagnostics.internal_drift_work_j += fields::Dot(rhs, dx);
      }
    }''',
        '''    if (view) {
      for (unsigned n = 0; n < Traits::nodes; ++n) {
        diagnostics.internal_kick_work_j += value.kick[n];
        diagnostics.internal_drift_work_j += value.drift[n];
      }
    }''')
    assert tokens(expected) == tokens(body(values, 'bool MeasureOperandFamily(')), \
        'literal status/scalar/slot fold'
    assert tokens(body(values, 'bool MeasureFinalFamily(')) == tokens(
        '''{
  return operands_prepared ? MeasureOperandFamily<Traits>(state, family_index, view)
      : MeasureValidatedFamily<Traits>(state, accepted, trial, family_index, view);
}''')


def verify_producer_order():
    values = (ROOT / 'lib_src/elements/solids/resident/MeasurementValues.h').read_text()
    assert tokens(body(values, 'void operator+=')) == tokens(
        '{ values[next++] = value; }')
    assert values.count('double* values;\n  unsigned next = 0;') == 1
    producer = body(values, 'void PrepareMeasurementOperands(')
    ordered = [
        'MeasurementOperands<Traits::nodes> next;',
        'const auto valid = CheckParentResult<Traits>',
        'family.result_valid[parent_index] = valid;',
        'if (valid == 1) {',
        'const auto& cache = family.slab[trial][parent_index].cache;',
        'next.work = Work(cache);',
        'next.hourglass_work = HourglassWork(cache);',
        'next.plastic_work = PlasticWork(cache);',
        'next.native_dt = NativeDt(cache);',
        'if (view) {',
        'MeasurementAddends kick{next.kick}, drift{next.drift};',
        'AccumulateMeasurementWork<Traits>(family.parents[parent_index],',
        'family.slab[accepted][parent_index].cache, *view, kick, drift);',
        'family.measurement[parent_index] = next;',
    ]
    cursor = 0
    for item in ordered:
        cursor = producer.index(item, cursor) + len(item)
    assert producer.count('CheckParentResult<Traits>') == 1
    conditional = producer.index('if (valid == 1) {')
    for read in ['family.slab[trial][parent_index].cache',
                 'family.parents[parent_index]',
                 'family.slab[accepted][parent_index].cache', '*view']:
        assert producer.index(read) > conditional, read


def verify_types():
    types = (ROOT / 'lib_src/elements/solids/resident/MeasurementTypes.h').read_text()
    assert tokens(types).count('doublework=0;') == 1
    for field in ['hourglass_work', 'plastic_work', 'native_dt']:
        assert tokens(types).count(f'double{field}=0;') == 1
    assert tokens(types).count('doublekick[Slots]{};') == 1
    assert tokens(types).count('doubledrift[Slots]{};') == 1
    assert 'static_assert(sizeof(MeasurementOperands<8>) == 160);' in types
    assert 'static_assert(sizeof(MeasurementOperands<6>) == 128);' in types
    assert types.count('std::is_trivially_copyable_v<MeasurementOperands<') == 2


def verify():
    authenticate_baseline()
    prefix = ROOT / 'lib_src/elements/solids/resident'
    legacy_measure((prefix / 'Measure.h').read_text())
    legacy_candidate((prefix / 'Candidate.cu').read_text())
    legacy_result_validation((prefix / 'ResultValidation.cu').read_text())
    verify_work_extraction()
    verify_operand_fold()
    verify_producer_order()
    verify_types()
    return {
        'baseline_commit': 'edab8e17e6d6e641f77f093fdb258d98152db492',
        'complete_measure_reversal': True,
        'complete_candidate_reversal': True,
        'complete_validation_reversal': True,
        'literal_work_extraction': True,
        'literal_ordered_operand_fold': True,
        'conditional_fresh_producer': True,
        'numerical_execution': False,
    }
