#!/usr/bin/env python3
"""Authenticate the observer boundary and the unchanged serial/cache equations."""
from pathlib import Path
import hashlib,json,subprocess,sys
here=Path(__file__).resolve().parent
root=here.parents[2]
raw=(here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest()=='9a5f36fff318b643f12595985ccbe532d02685b3dd0b8b5cb0ed0ea9f9dc3937'
manifest=json.loads(raw)
for row in manifest['files']:
    path=Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    value=(root/path).read_bytes()
    assert len(value)==row['bytes'] and hashlib.sha256(value).hexdigest()==row['sha256'],path
def body(text,name):
    begin=text.index('{',text.index(name+'('))
    end=begin+1;level=1
    while level:
        level+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[begin:end]
old=(here/'reference/ShellBatchFields.h').read_text()
current=(root/'lib_src/elements/ShellBatchFields.h').read_text()
assert body(old,'AccumulateInternalWork')==body(current,'AccumulateInternalWork')
assert 'template<unsigned Count,class Sum=double>' in current
assert 'Sum& kick_work,Sum& drift_work)' in current
subprocess.run([sys.executable,'-B',str(here.parent/'qeph_candidate_diagnostics/verify_sources.py')],check=True)
kernels=(root/'lib_src/elements/qeph/QephBatchKernels.cu').read_text()
assert 'if(mapped) LaunchMappedObserverDiagnostics(s,a,b,v,d,mixed);' in kernels
print(json.dumps({'status':'passed','records':len(manifest['files']),
    'unchanged':['cache_work_expression','candidate_mechanics','legacy_finalizer','serial_fallback_measure'],
    'numerical_execution':False}))
