"""Stage isolation, extraction identity and unchanged complete native source closure."""
from pathlib import Path
import hashlib,json,importlib.util
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
pins=json.loads((HERE/'extraction-identity.json').read_text())
for name,digest in pins['headers'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
spec=importlib.util.spec_from_file_location('h24_native_sources',HERE/'native/prepare_sources.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);native=mod.generated()
code=(ROOT/'lib_src/elements/solid24/controlled_hourglass/BeforeDistortion.h').read_text()
assert code.index('hg::EvaluateLaw42')<code.index('force_detail::MaterialForces')<code.index('force_detail::RotateAndMapForces')<code.index('output=next')
assert 'CalculateForce(' not in code and 'Stabilization(' not in code and 'EvaluateForce(' not in code
assert 'point.sound_speed_m_s' in code and 'geometry.hourglass_projection[n][h]' in code
assert 'WorkingLengthUnit::Metre' in code and 'world_native_force_before_distortion_n' in code
assert native['NativeCaller.F90'].count('call IC1_DISTORTION(')==1
assert native['NativeCaller.F90'].count('call IC1_ASSEMBLE(')==1
assert native['shour_ctl.F90'].count('call H24_ADAPTER_MODES(')==1
assert native['NativeMaterialForces.F'].index('CALL H24_FORCE(F,24,3)')<native['NativeMaterialForces.F'].index('CALL HEPH_NATIVE_SRROTA3(')
units=(ROOT/'lib_src/elements/solid24/controlled_hourglass/UnitResponse.h').read_text()
assert units.index('native_interval.position_m[n]=')<units.index('force_detail::CurrentKinematics')<units.index('force_detail::EvaluateMaterial')<units.index('status=EvaluateBeforeDistortion')<units.index('units_detail::StageToSi')<units.index('output=next;',units.index('units_detail::StageToSi'))
assert 'input.profile.working_length=WorkingLengthUnit::Metre' in units
assert 'reference.input().profile.working_length!=expected' in units
assert 'native_modal_work.work=next.stage.hourglass.work_j' in units
print(json.dumps({'status':'source_passed','shared_extraction_pins':True,'complete_native_sequence':True,'only_output_observations':True,'profile_enabled':False}))
