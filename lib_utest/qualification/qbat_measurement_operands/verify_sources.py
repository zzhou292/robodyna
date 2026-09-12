#!/usr/bin/env python3
"""Authenticated frozen caller, exact fold adaptation and owning source receipt."""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MANIFEST_SHA256 = '67f5ad3e1acde706ddbee60e2ed121fa1599a1e22871e4239a88a01ff8ec0857'


def body(text, signature):
    begin = text.index('{', text.index(signature))
    depth = 0
    for end in range(begin, len(text)):
        depth += text[end] == '{'
        depth -= text[end] == '}'
        if depth == 0:
            return text[begin:end + 1]
    raise AssertionError(signature)


def tokens(text):
    text = re.sub(r'//[^\n]*', '', text)
    return re.sub(r'\s+', '', text)


def once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


def check_fold(old, new):
    expected = body(old, 'bool MeasureParents(')
    expected = once(expected, '''    const auto& old=accepted.element[parent];
    const auto& now=trial.element[parent];
    const auto& element=model.element[parent];
    if(!valid(parent,now,element.material,d.time,d.epoch)) return false;''',
                    '    const auto& now=values[parent];\n    if(now.valid!=1) return false;')
    expected = once(expected, '  auto& d=diagnostics;\n', '')
    expected = once(expected, 'old.history.element_active&&!now.history.element_active', 'now.newly_removed')
    expected = once(expected, 'now.history.element_active', 'now.active')
    for declaration in [
        '    const double area=now.kinematics.geometry.area_m2/element.reference.quadrilateral().area;\n',
        '    const double thickness=now.history.thickness_m/element.reference.input().quadrilateral.thickness;\n',
        '    const double dt=now.diagnostics.unscaled_element_dt_s;\n']:
        expected = once(expected, declaration, '')
    for old_name, new_name in [('area', 'now.area_ratio'), ('thickness', 'now.thickness_ratio'), ('dt', 'now.native_dt')]:
        expected = re.sub(r'\b' + old_name + r'\b', new_name, expected)
    for name, staged in [('internal_work_j[channel]', 'internal_increment[channel]'),
                         ('plastic_work_j', 'plastic_increment'),
                         ('numerical_viscous_work_j', 'viscous_increment')]:
        expected = once(expected, 'now.history.' + name + '-old.history.' + name, 'now.' + staged)
    for name, staged in [('internal_work_j[channel]', 'internal_work[channel]'),
                         ('plastic_work_j', 'plastic_work'),
                         ('numerical_viscous_work_j', 'viscous_work')]:
        expected = once(expected, 'now.history.' + name, 'now.' + staged)
    strain_loop = 'for(unsigned point=0;point<4;++point) ' + body(old, 'for(unsigned point=0;point<4;++point)')
    expected = once(expected, strain_loop,
                    'if(now.maximum_strain>d.maximum_absolute_strain) d.maximum_absolute_strain=now.maximum_strain;')
    expected = once(expected, '''shell_batch_fields::AccumulateInternalWork(element.nodes,old.internal_force_n,old.internal_couple_nm,
          view,model.config.owner.fixed_dt,d.internal_kick_work,d.internal_drift_work);''',
                    '''for(unsigned slot=0;slot<4;++slot) {
      d.internal_kick_work-=now.kick_operand[slot];
      d.internal_drift_work-=now.drift_operand[slot];
    }''')
    assert tokens(expected) == tokens(body(new, 'bool MeasureStagedParents(')), 'ordered fold'


def verify():
    raw = (HERE / 'source-manifest.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == MANIFEST_SHA256
    manifest = json.loads(raw)
    for row in manifest['files']:
        path = Path(row['path'])
        assert not path.is_absolute() and '..' not in path.parts
        data = (ROOT / path).read_bytes()
        assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], path
    baseline = json.loads((HERE / 'baseline.json').read_bytes())
    for row in baseline['files']:
        data = (HERE / row['fixture']).read_bytes()
        assert len(data) == row['bytes'] and hashlib.sha256(data).hexdigest() == row['sha256'], row['fixture']
    prefix = ROOT / 'lib_src/elements/qbat'
    measure = (HERE / 'frozen/QbatBatchMeasure.h.txt').read_text()
    assert (prefix / 'QbatBatchMeasure.h').read_text() == measure
    check_fold(measure, (prefix / 'mapped/MeasurementValues.h').read_text())
    old = (HERE / 'frozen/Measurement.cu.txt').read_text()
    new = (prefix / 'mapped/Measurement.cu').read_text()
    assert body(old, 'void MaximumDisplacement(') == body(new, 'void MaximumDisplacement(')
    launch = once(body(old, 'void LaunchMappedMeasurements('),
                  'ValidateParents<<<256,128,0,view.stream>>>(storage,trial,identity);',
                  'PrepareParents<<<256,128,0,view.stream>>>(storage,accepted,trial,view,identity);')
    assert body(new, 'void LaunchMappedMeasurements(') == launch, 'same ordered launches'
    assert tokens(body(new, 'void FinalizeMapped(')) == tokens(
        '{ mapped::FinalizeMeasurement(*state,view,identity,blocks); }')
    finalize = once(body(old, 'void FinalizeMapped('), '  auto& s=*state;\n', '')
    finalize = re.sub(r'\bs\b', 'state', finalize)
    finalize = once(finalize, 'mapped::Measure(state,*accepted,*trial,view,state.control.diagnostics,blocks)',
                    'MeasureStaged(state,view,state.control.diagnostics,blocks)')
    header = (prefix / 'mapped/Measurement.h').read_text()
    assert tokens(finalize) == tokens(body(header, 'void FinalizeMeasurement(')), 'status and publication order'
    old_header = (HERE / 'frozen/Measurement.h.txt').read_text()
    finish = body(old_header, 'bool Measure(')
    finish = '{\n  ' + finish[finish.index('MaximumSummary maximum'):]
    assert tokens(finish) == tokens(body(header, 'bool FinishMeasurement(')), 'unchanged displacement/finite suffix'
    legacy = (ROOT / 'lib_utest/qualification/qbat_mapped_gather/SerialCandidate.cuh').read_text()
    oracle = (HERE / 'SerialFinalize.h').read_text()
    assert body(oracle, 'void Finalize(') == body(legacy, 'void FinalizeCandidate(').replace(
        'serial::Measure(', 'qbat_gather_test::serial::Measure(')
    subprocess.run([sys.executable, '-B', str(ROOT / 'lib_utest/qualification/qbat_mapped_gather/verify_sources.py')], check=True)
    return {'status': 'passed', 'records': len(manifest['files']), 'frozen_full_caller': True,
            'ordered_operand_fold': True, 'unchanged_full_result_validation': True,
            'numerical_execution': False}


if __name__ == '__main__':
    print(json.dumps(verify(), sort_keys=True))
