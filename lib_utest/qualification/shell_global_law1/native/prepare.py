#!/usr/bin/env python3
"""Mechanical ITHK forwarding around the already qualified global native leaves."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
Q = ROOT / 'lib_utest/qualification/native/qeph'
T = ROOT / 'lib_utest/qualification/native/t3'


def one(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Adapter seam changed: ' + old)
    return text.replace(old, new)


def verify():
    manifest = json.loads((HERE / 'source-manifest.json').read_text())
    for row in manifest['files']:
        source = (ROOT / row['path']).read_bytes()
        if len(source) != row['bytes'] or hashlib.sha256(source).hexdigest() != row['sha256']:
            raise ValueError('Pinned inherited source changed: ' + row['path'])
    for parent in (Q, T):
        subprocess.run([sys.executable, '-B', str(parent / 'verify_sources.py')], check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def generated():
    verify()
    out = {}
    for parent, prefix, family in ((Q, 'Qeph', 'qeph'), (T, 'T3', 't3')):
        modules = (['QE_Q2_MATERIAL_MOD', 'QE_Q2_LAW1_MOD', 'QE_Q2_STIFFNESS_MOD']
                   if family == 'qeph' else
                   ['T3_ENGINE_MATERIAL_MOD', 'T3_ENGINE_LAW1_MOD', 'T3_ENGINE_STIFFNESS_MOD'])
        for kind in ('Material', 'Law1', 'Stiffness', 'Force'):
            filename = ('Native' + prefix + 'Force.F' if kind == 'Force' else prefix + 'Native' + kind + '.F')
            text = (parent / filename).read_text()
            for module in modules:
                text = re.sub(r'\b' + module + r'\b', 'GL1_' + module, text)
            if kind == 'Material':
                text = one(text, 'USE ISO_C_BINDING, ONLY: C_DOUBLE', 'USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT')
                if family == 'qeph':
                    text = one(text, 'SUBROUTINE QE_PREPARE_MATERIAL(INPUT,AREA,THK,STEP,M)',
                               'SUBROUTINE QE_PREPARE_MATERIAL(INPUT,AREA,THK,STEP,ITHK,M)')
                    text = one(text, 'TYPE(QE_MATERIAL_),INTENT(OUT) :: M',
                               'INTEGER(C_INT),INTENT(IN) :: ITHK\n      TYPE(QE_MATERIAL_),INTENT(OUT) :: M')
                    text = one(text, 'M%RHO,M%VOL0,M%GS,1,0,0,M%DT1C', 'M%RHO,M%VOL0,M%GS,1,ITHK,0,M%DT1C')
                else:
                    text = one(text, 'SUBROUTINE T3_PREPARE_MATERIAL(INPUT,G,H,M)',
                               'SUBROUTINE T3_PREPARE_MATERIAL(INPUT,G,H,ITHK,M)')
                    text = one(text, 'TYPE(T3_MATERIAL_),INTENT(OUT) :: M',
                               'INTEGER(C_INT),INTENT(IN) :: ITHK\n      TYPE(T3_MATERIAL_),INTENT(OUT) :: M')
                    text = one(text, '1,0,0,-1,M%VOL00', '1,ITHK,0,-1,M%VOL00')
            if kind == 'Force':
                if family == 'qeph':
                    text = one(text, 'SUBROUTINE QE_Q2_FORCE(X,V,VR,MATERIAL,BASE_HISTORY,STEP,',
                               'SUBROUTINE GL1_QE_FORCE(X,V,VR,MATERIAL,BASE_HISTORY,STEP,ITHK,')
                    text = one(text, 'NAME="qeph_q2_force"', 'NAME="global_law1_qeph_force"')
                    text = one(text, 'INTEGER(C_INT),INTENT(OUT) :: IS_PLANAR,STATUS',
                               'INTEGER(C_INT),INTENT(IN) :: ITHK\n      INTEGER(C_INT),INTENT(OUT) :: IS_PLANAR,STATUS')
                    text = one(text, 'CALL QE_PREPARE_MATERIAL(MATERIAL(1:4),GEOMETRY%AREA,H%THK,STEP,M)',
                               'CALL QE_PREPARE_MATERIAL(MATERIAL(1:4),GEOMETRY%AREA,H%THK,STEP,ITHK,M)')
                else:
                    text = one(text, 'SUBROUTINE T3_R3_FORCE(X,V,VR,MATERIAL,BASE_HISTORY,STEP,OUTPUT,STATUS)',
                               'SUBROUTINE GL1_T3_FORCE(X,V,VR,MATERIAL,BASE_HISTORY,STEP,ITHK,OUTPUT,STATUS)')
                    text = one(text, 'NAME="t3_r3_force"', 'NAME="global_law1_t3_force"')
                    text = one(text, 'INTEGER(C_INT),INTENT(OUT) :: STATUS',
                               'INTEGER(C_INT),INTENT(IN) :: ITHK\n      INTEGER(C_INT),INTENT(OUT) :: STATUS')
                    text = one(text, 'CALL T3_PREPARE_MATERIAL(MATERIAL,G,H,M)',
                               'CALL T3_PREPARE_MATERIAL(MATERIAL,G,H,ITHK,M)')
                    text = one(text, '#include "com08_c.inc"', '#include "com08_c.inc"\n#include "impl1_c.inc"')
                    text = one(text, 'CALL T3_PREPARE_FRAME(X,V,VR,STEP,G,STATUS)',
                               'ISMDISP=0\n      CALL T3_PREPARE_FRAME(X,V,VR,STEP,G,STATUS)')
            out[prefix + kind + '.F'] = text
        text = (parent / (prefix + 'ForceReference.cpp')).read_text()
        opening = 'namespace tl::qualification::' + family + ' {'
        private = 'tl::qualification::global_law1_native::' + family + '_adapter'
        text = one(text, opening, 'namespace ' + private + ' {\nusing namespace ::tl::qualification::' + family + ';')
        text = '#include "NativeReference.h"\n' + text
        declaration = ('extern "C" void global_law1_' + family + '_force(const double*,const double*,const double*,\n'
                       '    const double*,const double*,const double*,const int*,double*,' +
                       ('int*,int*);\n' if family == 'qeph' else 'int*);\n'))
        text = one(text, 'using namespace ::tl::qualification::' + family + ';',
                   'using namespace ::tl::qualification::' + family + ';\n' + declaration)
        if family == 'qeph':
            text = one(text, 'const PrescribedInterval& interval,ForceTrial& output) noexcept {',
                       'const PrescribedInterval& interval,int ithk,ForceTrial& output) noexcept {\n  if(ithk!=0&&ithk!=1)return Status::kInvalidInput;')
            text = one(text, 'detail::qeph_q2_force(', 'global_law1_qeph_force(')
            text = one(text, '&interval.dt,values.data(),&planar,&status)', '&interval.dt,&ithk,values.data(),&planar,&status)')
        else:
            text = one(text, 'const PrescribedInterval& in,ForceTrial& output) noexcept {',
                       'const PrescribedInterval& in,int ithk,ForceTrial& output) noexcept {\n  if(ithk!=0&&ithk!=1)return Status::kInvalidInput;')
            text = one(text, 'detail::t3_r3_force(', 'global_law1_t3_force(')
            text = one(text, '&in.dt,values.data(),&status)', '&in.dt,&ithk,values.data(),&status)')
            text = one(text, 'd.effective_thickness!=p.thickness', 'd.effective_thickness!=(ithk?base.data().thickness:p.thickness)')
        text = text.replace('// namespace tl::qualification::' + family, '// namespace ' + private)
        alias = 'q' if family == 'qeph' else 't'
        text += ('\nnamespace tl::qualification::global_law1_native {\n' + alias + '::Status Evaluate(const ' + alias +
                 '::Reference& r,const ' + alias + '::History& h,const ' + alias + '::PrescribedInterval& in,int ithk,' +
                 alias + '::ForceTrial& out) noexcept {return ' + family + '_adapter::EvaluateForce(r,h,in,ithk,out); }\n}\n')
        out[prefix + 'Adapter.cpp'] = text
    return out


def prepare(output, check):
    texts = generated()
    if not check:
        output.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, text in texts.items():
        data = text.encode(); path = output / name
        if check:
            if path.read_bytes() != data:
                raise ValueError('Generated native adapter differs: ' + name)
        elif not path.exists() or path.read_bytes() != data:
            path.write_bytes(data)
        rows.append(dict(file=name, bytes=len(data), sha256=hashlib.sha256(data).hexdigest()))
    return dict(status='passed', numerical_leaf_changes=0, generated=rows, check=check)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    print(json.dumps(prepare(args.output, args.check), indent=2))
