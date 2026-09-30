#!/usr/bin/env python3
"""Exact shared formatter extraction and unchanged timer/summary surroundings."""
from pathlib import Path
import argparse
import hashlib
import json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
BASELINE_SHA256='638417f9f85f4f014c763f6a5bec72a94f922c686566929f759dd3545c7e8f5d'
def function(text, signature):
    start=text.index(signature);begin=text.index('{',start);depth=0
    for end in range(begin,len(text)):
        depth+=(text[end]=='{')-(text[end]=='}')
        if depth==0:return text[start:end+1]
    raise ValueError('Incomplete function')
def once(text,old,new):
    assert text.count(old)==1,(old,text.count(old))
    return text.replace(old,new)
def verify():
    raw=(HERE/'timing-baseline.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest()==BASELINE_SHA256
    baseline=json.loads(raw)
    assert baseline['head']=='354260d12e9688ef0cfc7930eafac3fdf5c1d389'
    for row in baseline['files']:
        raw=(HERE/row['fixture']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256']
    old=(HERE/'timing_frozen/LegacySummary.cpp').read_text()
    formatter=function(old,'output::Document Timings(')
    shared=(ROOT/'case/vehicle_run/StageTimingDocument.cpp').read_text()
    assert function(shared,'output::Document StageTimingDocument(').replace(' StageTimingDocument(',' Timings(',1)==formatter
    legacy=(ROOT/'case/vehicle_run/Summary.cpp').read_text()
    legacy=once(legacy,'#include "StageTimingDocument.h"\n','')
    legacy=once(legacy,'auto timing=StageTimingDocument(result.mechanics_timing);','auto timing=Timings(result.mechanics_timing);')
    legacy=once(legacy,'}\nrecords::RecordFile WriteSummary',formatter+'\n}\nrecords::RecordFile WriteSummary')
    assert legacy==old
    native=(ROOT/'case/vehicle_native_contact/run/Summary.cpp').read_text()
    native=once(native,'#include "case/vehicle_run/StageTimingDocument.h"\n','')
    native=once(native,'    array_json::Child(doc, "mechanics_stage_timing",\n'
        '        vehicle_run::detail::StageTimingDocument(result.loop.progress.mechanics_timing));\n','')
    assert native==(HERE/'timing_frozen/NativeSummary.cpp').read_text()
    assert (ROOT/'benchmarks/stage_timing/StageTimer.h').read_bytes()==(HERE/'timing_frozen/StageTimer.h').read_bytes()
    return formatter
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path)
    args=parser.parse_args();formatter=verify()
    if args.output:
        args.output.mkdir(parents=True,exist_ok=True)
        (args.output/'FrozenTimingDocument.h').write_text('#pragma once\n#include "case/vehicle_run/StageTimingDocument.h"\n'
            'namespace crash::cases::vehicle_run::test::timing_frozen {\n'+formatter+'\n}\n')
    print(json.dumps({'status':'passed','exact_formatter_extraction':True,
        'legacy_summary_reversal':True,'native_summary_addition_only':True,'timer_unchanged':True}))
