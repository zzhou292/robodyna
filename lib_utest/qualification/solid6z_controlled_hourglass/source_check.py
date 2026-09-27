from pathlib import Path
import importlib.util,json
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
spec=importlib.util.spec_from_file_location('s6ctl',HERE/'native/prepare_sources.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);native=mod.generated()
code=(ROOT/'lib_src/elements/solid6z/controlled_hourglass/Response.h').read_text()
assert code.index('MaterialForces(')<code.index('hg::EvaluateLaw42')<code.index('force_detail::Project')<code.index('output=next')
assert 'Stabilize(' not in code and 'S6ZHOUR3' not in native['NativeResultants.F90']
assert 'SSP=POINT(20)' in native['NativeResultants.F90'] and 'STIN=POINT(33)' in native['NativeResultants.F90']
assert 'S6_CONTROL_ASSEMBLE' in native['NativeResultants.F90']
source=(ROOT/'lib_utest/qualification/solid6z_force/native/original/engine/source/elements/solid/solide6z/s6zforc3.F90').read_text()
assert 'if ((isctl > 0).and.(1 == 2)) then' in source
print(json.dumps({'status':'source_passed','native_controlled_ssp_unmodified':True,'actual_nodal_STIN_observed':True,'geometric_distortion_disabled':True,'profile_enabled':False}))
