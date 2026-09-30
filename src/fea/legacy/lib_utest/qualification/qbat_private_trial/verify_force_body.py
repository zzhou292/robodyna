"""Verify that the resident output refactor preserves the prior numerical body."""
from pathlib import Path
import hashlib
import json
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def tokens(text):
    return re.sub(r'\s+', '', re.sub(r'//[^\n]*', '', text))


def once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


def body(text, signature):
    first = text.index('{', text.index(signature))
    level = 0
    for end in range(first, len(text)):
        level += text[end] == '{'
        level -= text[end] == '}'
        if not level:
            return text[first:end + 1]
    raise AssertionError(signature)


def verify():
    manifest = json.loads((HERE / 'baseline.json').read_text())
    assert manifest['base'] == '69a1d05c182797c1788cb81adbf89efee74dce83'
    for item in manifest['files']:
        data = (HERE / item['path']).read_bytes()
        assert len(data) == item['bytes'] and hashlib.sha256(data).hexdigest() == item['sha256']
    source = ROOT / 'lib_src/elements/qbat'
    old = (HERE / 'QbatForce.h.baseline').read_text()
    new = (source / 'QbatForceBody.h').read_text()
    first = '  const double endpoint=interval.base_time+interval.dt;'
    expected = old[old.index(first):old.index('\n  output=trial;')]
    expected = once(expected, 'detail::ValidHistory(accepted.data(),material,stamp.time)',
                    'detail::ValidHistory(base,material,stamp.time)')
    expected = once(expected, '  ForceTrial trial;', '  ClearForceFields(trial);')
    expected = once(expected, 'geometry_input.native_off=accepted.data().element_active?1.:0.;',
                    'geometry_input.native_off=base.element_active?1.:0.;')
    expected = once(expected, '  const auto& base=accepted.data();\n  auto next=base;',
                    '  auto& next = trial.history;\n  next = base;')
    expected = once(expected,
        'PreparePrescribedHistory(reference,material,failure,next,{endpoint,interval.sample_index},trial.proposed_history)',
        'ValidateHistoryPreparation(reference,material,failure,next,{endpoint,interval.sample_index})')
    start = new.index(first)
    actual = new[start:new.index('\n  trial.stamp =', start)]
    assert tokens(expected) == tokens(actual), 'force arithmetic/order changed'
    public = (source / 'QbatForce.h').read_text()
    old_checks = old[old.index('  if (!accepted.prepared()'):old.index('  const auto stamp=accepted.stamp();')]
    new_checks = public[public.index('  if (!accepted.prepared()'):public.index('  ForceTrial trial;')]
    assert tokens(old_checks) == tokens(new_checks), 'public identity admission changed'
    old_history = (HERE / 'QbatHistory.h.baseline').read_text()
    check = body(old_history, 'Status PreparePrescribedHistory(')
    check = check[check.index('  if ('):check.index('\n  History next;')]
    check = check.replace('detail::ValidMaterial', 'ValidMaterial').replace('detail::ValidHistory', 'ValidHistory')
    new_history = body((source / 'QbatHistory.h').read_text(), 'Status ValidateHistoryPreparation(')
    new_history = new_history[new_history.index('  if ('):new_history.index('\n  return Status::kSuccess;')]
    assert tokens(check) == tokens(new_history), 'initial history validation changed'
    advance = (source / 'QbatBatchAdvance.h').read_text()
    baseline_advance = (HERE / 'QbatBatchAdvance.h.baseline').read_text()
    for signature in ['BatchResult PackResult(', 'Status InitializeResult(', 'Status Advance(']:
        assert body(advance, signature) == body(baseline_advance, signature), signature
    kernel = (HERE / 'QbatBatchKernels.cu.baseline').read_text()
    kernel = once(kernel, 's.candidate_status[parent]=Advance(', 's.candidate_status[parent]=AdvanceIntoTrial(')
    assert kernel == (source / 'QbatBatchKernels.cu').read_text(), 'resident orchestration changed'
    assert 'OpenRadioss (C) 2026 Siemens' in new
    return {'status': 'passed', 'shared_force_body': True, 'unchanged_numerical_order': True,
            'unchanged_public_identity_admission': True, 'unchanged_resident_launches': True,
            'numerical_execution': False}


if __name__ == '__main__':
    print(json.dumps(verify(), sort_keys=True))
