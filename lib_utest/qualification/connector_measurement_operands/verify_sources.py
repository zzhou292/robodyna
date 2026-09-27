#!/usr/bin/env python3
"""Authenticate the frozen serial caller and unchanged evaluation/failure order."""
from pathlib import Path
import hashlib
import json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
BASELINE_SHA256='82f8d6a78321482cac8da57f6239a9fbaad480233c4612ec99bb98bfe6f65803'
MANIFEST_SHA256='34fe132194ace7fbb447808cc2da96dc27116661f853d1427b950f02ca061b74'
def once(text,old,new):
    assert text.count(old)==1,(old,text.count(old))
    return text.replace(old,new)
def verify():
    raw=(HERE/'baseline.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest()==BASELINE_SHA256
    baseline=json.loads(raw)
    assert baseline['commit']=='90a4a8060966358714049a775e4f9f7e7838481e'
    for row in baseline['files']:
        data=(HERE/row['fixture']).read_bytes()
        assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256'],row['path']
    raw=(HERE/'source-manifest.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest()==MANIFEST_SHA256
    for row in json.loads(raw)['files']:
        data=(ROOT/row['path']).read_bytes()
        assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256'],row['path']
    for label,relative in [('Type13','type13/resident/Measure.h'),('Type25','type25/Type25BatchMeasure.h')]:
        assert (ROOT/'lib_src/elements'/relative).read_bytes()==(HERE/'frozen'/f'{label}Measure.h').read_bytes()
    current=(ROOT/'lib_src/elements/type13/resident/Candidate.cu').read_text()
    current=once(current,'#include "Measurement.h"','#include "Measure.h"')
    current=once(current,'MeasurePrepared(state.model, state.measurement, state.control.diagnostics)',
        'Measure(state.model, state.slab[accepted], state.slab[trial], view,\n               state.control.diagnostics)')
    current=once(current,'  LaunchMeasurement(storage, accepted, trial, view, count);\n  if (cudaPeekAtLastError() != cudaSuccess) return;\n','')
    assert current==(HERE/'frozen/Type13Candidate.cu').read_text()
    current=(ROOT/'lib_src/elements/type25/Type25BatchKernels.cu').read_text()
    current=once(current,'#include "Type25BatchMeasurement.h"\n#include "Type25BatchStorage.h"','#include "Type25BatchMeasure.h"')
    current=once(current,'MeasurePrepared(s.model,s.measurement,s.control.diagnostics)',
        'Measure(s.model,*accepted,*trial,view,s.control.diagnostics)')
    current=once(current,'  LaunchMeasurement(s,accepted,trial,view,count);\n  if(cudaPeekAtLastError()!=cudaSuccess)return;\n','')
    assert current==(HERE/'frozen/Type25Candidate.cu').read_text()
    return {'status':'passed','complete_evaluation_and_failure_scan_reversals':True,
        'original_measurement_bodies_unchanged':True,'numerical_execution':False}
if __name__=='__main__':print(json.dumps(verify(),sort_keys=True))
