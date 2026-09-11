#!/usr/bin/env python3
"""Frozen pre-parallelization assembly and unchanged shared leaf receipt."""
from pathlib import Path
import hashlib,json
here=Path(__file__).resolve().parent
root=here.parents[2]
raw=(here/'source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest()=="d3f061f45b8b1ef2a9d1338370e018da7978b8600439a15717a2e5f08e9890b7"
manifest=json.loads(raw)
for row in manifest['files']:
    path=Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    value=(root/path).read_bytes()
    assert len(value)==row['bytes'] and hashlib.sha256(value).hexdigest()==row['sha256'],path
serial=(here/'SerialAssembly.cuh').read_text()
serial=serial.replace('namespace t3_gather_test {\nusing namespace tl::fea;\nusing namespace tl::fea::t3;\nusing namespace tl::fea::t3::batch_detail;', 'namespace tl::fea::t3::batch_detail {')
serial=serial.replace('} // namespace t3_gather_test','} // namespace tl::fea::t3::batch_detail')
serial=serial.replace('LaunchSerialAssembly','LaunchMappedAssembly')
for absolute,relative in [('lib_src/elements/t3/mapped/Stiffness.h','Stiffness.h'),
    ('lib_src/elements/t3/mapped/Result.h','Result.h'),
    ('lib_src/elements/t3/T3BatchStorage.h','../T3BatchStorage.h'),
    ('lib_src/elements/ShellMixedSectionArenaLayout.h','../../ShellMixedSectionArenaLayout.h'),
    ('lib_src/elements/ShellMappedNode.h','../../ShellMappedNode.h'),
    ('lib_src/solvers/NodalForceAssembly.h','../../../solvers/NodalForceAssembly.h'),
    ('lib_src/solvers/NodalCinRuntime.h','../../../solvers/NodalCinRuntime.h')]:
    serial=serial.replace('#include "'+absolute+'"','#include "'+relative+'"')
assert len(serial.encode())==manifest['baseline_bytes']
assert hashlib.sha256(serial.encode()).hexdigest()==manifest['baseline_sha256']
print(json.dumps({'status':'passed','records':len(manifest['files']),'baseline':manifest['baseline_commit'],'numerical_execution':False}))
