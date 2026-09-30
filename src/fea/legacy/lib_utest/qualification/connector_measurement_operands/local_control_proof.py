#!/usr/bin/env python3
"""Checked terminal-Control transformation of the exact759 connector callers."""
from pathlib import Path
import hashlib
import json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
BASELINE_SHA256='429ddd424a1088d2c5d90bddbf3495e3efbd232f7866be95ee7c79cf44c17ddd'
def once(text,old,new):
    assert text.count(old)==1,(old,text.count(old))
    return text.replace(old,new)
def transform(label,text):
    if label=='Type13Candidate.cu':
        text=once(text,'  state.control = {};\n  state.control.diagnostics = identity;',
            '  Control next;\n  next.diagnostics = identity;')
        text=once(text,'      state.control.status = BatchStatus::ElementFailure;\n'
            '      state.control.element = e;\n'
            '      state.control.element_status = state.candidate_status[e];\n'
            '      return;',
            '      next.status = BatchStatus::ElementFailure;\n'
            '      next.element = e;\n'
            '      next.element_status = state.candidate_status[e];\n'
            '      break;')
        text=once(text,'  if (!MeasurePrepared(state.model, state.measurement, state.control.diagnostics)) {\n'
            '    state.control.status = BatchStatus::NonfiniteResult;\n'
            '    return;\n'
            '  }\n'
            '  state.control.diagnostics.valid = true;',
            '  if (next.status == BatchStatus::Success) {\n'
            '    if (!MeasurePrepared(state.model, state.measurement, next.diagnostics)) {\n'
            '      next.status = BatchStatus::NonfiniteResult;\n'
            '    } else {\n'
            '      next.diagnostics.valid = true;\n'
            '    }\n'
            '  }\n'
            '  state.control = next;')
    else:
        text=once(text,'  auto& s=*storage;s.control={};s.control.diagnostics=identity;',
            '  auto& s=*storage;Control next;next.diagnostics=identity;')
        text=once(text,'    s.control.status=BatchStatus::ElementFailure;s.control.element=static_cast<std::uint32_t>(e);\n'
            '    s.control.element_status=s.candidate_status[e];return;',
            '    next.status=BatchStatus::ElementFailure;next.element=static_cast<std::uint32_t>(e);\n'
            '    next.element_status=s.candidate_status[e];break;')
        text=once(text,'  if(!MeasurePrepared(s.model,s.measurement,s.control.diagnostics)) {s.control.status=BatchStatus::NonfiniteResult;return;}\n'
            '  s.control.diagnostics.valid=true;',
            '  if(next.status==BatchStatus::Success) {\n'
            '    if(!MeasurePrepared(s.model,s.measurement,next.diagnostics)) next.status=BatchStatus::NonfiniteResult;\n'
            '    else next.diagnostics.valid=true;\n'
            '  }\n'
            '  s.control=next;')
    return text

def baseline():
    raw=(HERE/'local-control-baseline.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest()==BASELINE_SHA256
    values=json.loads(raw)
    assert values['commit']=='759e486a0e1a8c072bca48423be833b9c04e9b21'
    for entry in values['callers']:
        raw=(HERE/entry['fixture']).read_bytes()
        assert len(raw)==entry['bytes'] and hashlib.sha256(raw).hexdigest()==entry['sha256']
    for entry in values['unchanged']:
        raw=(ROOT/entry['path']).read_bytes()
        assert len(raw)==entry['bytes'] and hashlib.sha256(raw).hexdigest()==entry['sha256'],entry['path']
    return values

def reverse(label,current):
    values=baseline()
    entry=next(row for row in values['callers'] if Path(row['fixture']).name==label)
    original=(HERE/entry['fixture']).read_text()
    assert current==transform(label,original),entry['path']
    return original

if __name__=='__main__':
    for row in baseline()['callers']:
        reverse(Path(row['fixture']).name,(ROOT/row['path']).read_text())
    print(json.dumps({'status':'passed','only_terminal_control_storage_changed':True,
        'ordered_leaves_layout_ownership_and_launch_order_unchanged':True}))
