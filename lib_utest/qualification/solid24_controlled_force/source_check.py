from pathlib import Path
import json
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];BASE=ROOT/'lib_src/elements/solid24/controlled_distortion'
p=(BASE/'Prepare.h').read_text();s=(BASE/'Stage.h').read_text()
assert p.index('working_detail::EvaluateNumeric')<p.index('distortion::native::PrepareParameters')<p.index('distortion::native::ClassifyDamping')
assert 'accepted->native_values()' in p and 'accepted->native_distortion_energy()' in p
assert 'ToNative' not in p and 'ToSi' not in p
assert s.index('distortion::native::EvaluateForce')<s.index('force.force[n],f.base.force')<s.index('HistoryWriter::Set')<s.index('output=next')
assert 'force.raw_stiffness*f.base.stiffness' in s
print(json.dumps({'status':'source_passed','typed_native_history_retained':True,'distortion_before_SI_conversion':True,'required_native_batch_flag':True,'public_owner_enabled':False}))
