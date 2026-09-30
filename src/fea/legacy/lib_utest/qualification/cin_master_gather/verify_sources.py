"""Authenticate reused gather math and reverse only current-owner scheduling hooks."""
from pathlib import Path
import hashlib,json,runpy
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
raw=(HERE/'reuse-source-manifest.json').read_bytes()
assert hashlib.sha256(raw).hexdigest()=='ab4951145de70b9f276af81a9506fbb67512f29bdf3eb2eb860219ea7fea26a0'
doc=json.loads(raw)
for row in doc['files']:
    data=(ROOT/row['path']).read_bytes()
    assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256'],row['path']
proof=runpy.run_path(str(HERE/'gather_proof.py'))
proof['prove']()
runpy.run_path(str(HERE.parent/'cin_post_node_inputs/verify_sources.py'),run_name='__main__')
for name in ('CMakeLists.txt','BUILD.bazel'):
    assert (ROOT/'lib_src/solvers'/name).read_text().count('cin_advance/ForceGather.cu')==1
startup=(ROOT/'lib_src/solvers/NodalCinStartup.cpp').read_text()
assert startup.index('if (next->dependent[node])')<startup.index('if (layout.gather.device_bytes)')
print(json.dumps({'status':'passed','donor':doc['donor'],'integration_base':doc['integration_base'],
    'unchanged_core_files':len(doc['files']),'owner_restorations':len(proof['BASELINE']),
    'historical_receipts_preserved':True,'numerical_execution':False}))
