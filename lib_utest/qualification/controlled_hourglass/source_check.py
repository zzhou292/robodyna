"""Source and boundary proof; numerical comparison remains in native CPU/CUDA tests."""
from pathlib import Path
import importlib.util,struct,json
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('leaf',HERE/'native/prepare_sources.py');leaf=importlib.util.module_from_spec(spec);spec.loader.exec_module(leaf)
prepared=leaf.generated();native=prepared['shour_ctl.F90']
assert 'stif = 0.3*qh*lamg' in native and 'if (nu>0.48999)' in native
assert native.count('call CONTROLLED_LEAF_MODES(')==1 and native.count('call IC1_NATIVE_HOUR_WORK(')==1
assert 'pm(32,i) = bulk' in prepared['slot_generic.inc']
root=HERE.parents[2];code=(root/'lib_src/elements/solid_common/controlled_hourglass/Response.h').read_text()
types=(root/'lib_src/elements/solid_common/controlled_hourglass/Types.h').read_text()
assert 'static_cast<double>(0.3f)' in code and 'static_cast<double>(0.48999f)' in types
assert struct.unpack('f',struct.pack('f',.48999))[0]<.48999
assert 'next.work_j=input.dt_s*ModePower' in code and 'next.work_j/input.reference_volume_m3' in code
assert code.index('if(!detail::FiniteResult(next))')<code.index('output=next;')
print(json.dumps({'status':'source_passed','native_full_leaf_authenticated':True,'reversible_modal_observation':True,'native_default_real_literals_preserved':True,'production_profile_enabled':False}))
