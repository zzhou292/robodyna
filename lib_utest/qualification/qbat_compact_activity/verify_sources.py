#!/usr/bin/env python3
"""Protect complete predicates, full snapshot bodies and ordered publication."""
from pathlib import Path
import hashlib
import json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
BASELINE_SHA256='c76f42e8439c4440f3ef7354469f695090a6bba21e3e9b094f59a5987c83b989'
MANIFEST_SHA256='1ff0755ba8b63f752670872f0efd47a672d7e066aae4a2b09cb67a9eac3cc8f8'
def once(text,old,new):
    assert text.count(old)==1,(old,text.count(old))
    return text.replace(old,new)
def verify():
    raw=(HERE/'baseline.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest()==BASELINE_SHA256
    baseline=json.loads(raw)
    assert baseline['commit']=='95664290ccf60c486174944f7399c8a9afe3b760'
    for row in baseline['files']:
        raw=(HERE/row['fixture']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256']
        if not row['path'].endswith('QbatBatchReadback.cu'):
            assert (ROOT/row['path']).read_bytes()==raw
    for row in baseline['unchanged_source_inputs']:
        raw=(ROOT/row['path']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256']
    raw=(HERE/'source-manifest.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest()==MANIFEST_SHA256
    for row in json.loads(raw)['files']:
        raw=(ROOT/row['path']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256'],row['path']
    readback=(ROOT/'lib_src/elements/qbat/QbatBatchReadback.cu').read_text()
    readback=once(readback,'      !Disjoint(output,bytes,activity_staging.data(),activity_staging.size())||\n','')
    readback=once(readback,'state.ReadParentActivity(state.accepted,state.accepted_diagnostics)',
        'state.ReadResults(state.accepted,state.accepted_diagnostics)')
    readback=once(readback,'state.ReadParentActivity(state.trial, expected)','state.ReadResults(state.trial, expected)')
    assert readback.count('output[parent] = state.ParentActivity()[parent];')==2
    readback=readback.replace('output[parent] = state.ParentActivity()[parent];',
        'output[parent] = state.staging[parent].history.element_active ? 1 : 0;')
    assert readback==(HERE/'frozen/QbatBatchReadback.cu').read_text()
    values=(ROOT/'lib_src/elements/qbat/activity/Values.h').read_text()
    assert 'return batch_detail::ValidResult(state.slab[slab].element[parent],\n      state.model.element[parent].material, time, epoch);' in values
    assert 'if (!lookup(parent) || parent == first_invalid || flags[parent] > 1)' in values
    assert 'return Invalid(static_cast<std::uint32_t>(parent));' in values
    kernel=(ROOT/'lib_src/elements/qbat/activity/Kernels.cu').read_text()
    assert kernel.index('packet.active[parent] = activity::InvalidFlag;')<kernel.index('activity::ValidParent(')
    assert kernel.index('activity::ValidParent(')<kernel.index('history.element_active ? 1 : 0;')
    compact=(ROOT/'lib_src/elements/qbat/activity/Readback.cu').read_text()
    assert compact.index('cudaStreamSynchronize(stream)')<compact.index('activity::Complete(')
    assert 'Failure().catalog()->Parameters(ShellBindingFamily::Qbat,parent,&material)' in compact
    assert 'if(report.status!=BatchStatus::Success) Discard();' in compact
    return {'status':'passed','full_snapshot_reader_reversal':True,'complete_result_predicate_unchanged':True,
        'material_build_rebase_and_catalog_unchanged':True,'source_ordered_lookup_before_verdict':True,
        'fresh_per_call_validation_and_post_drain_publication':True,'numerical_execution':False}
if __name__=='__main__':print(json.dumps(verify(),sort_keys=True))
