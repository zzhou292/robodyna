#!/usr/bin/env python3
from pathlib import Path
import argparse,hashlib,json
parser=argparse.ArgumentParser(description='Authenticate and adapt existing complete native family callers')
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--check',action='store_true')
args=parser.parse_args()
here=Path(__file__).resolve().parent
project=here.parents[3]
root=project/'lib_utest/qualification'
out=args.output
raw=(here/'source-manifest.json').read_bytes()
if hashlib.sha256(raw).hexdigest()!='2adc63262e71445d258bd1faed674f6b189b78ba2eb49352bf61e9317cada5d5':
    raise RuntimeError('Native adaptation manifest changed')
manifest=json.loads(raw)
for row in manifest['inputs']:
    data=(project/row['path']).read_bytes()
    if len(data)!=row['bytes'] or hashlib.sha256(data).hexdigest()!=row['sha256']:
        raise RuntimeError('Native adaptation input changed: '+row['path'])
def emit(name,text):
    data=text.encode()
    if hashlib.sha256(data).hexdigest()!=manifest['generated'][name]:
        raise RuntimeError('Native adaptation changed: '+name)
    path=out/name
    if args.check:
        if path.read_bytes()!=data: raise RuntimeError('Prepared native caller changed: '+name)
    else:
        out.mkdir(parents=True,exist_ok=True)
        path.write_bytes(data)
source=root/'shell_layered_j2/native_recurrence'
for family,name,short in [('Qeph','QEPH','QE'),('T3','T3','T3')]:
    s=(source/f'NativeLayered{family}Section.F').read_text()
    a=s.index(f'      SUBROUTINE LR_{short}_MATERIAL(')
    b=s.index('      END SUBROUTINE',a)+len('      END SUBROUTINE\n')
    s=s[:a]+s[b:]
    s=s.replace(f'LR_{name}_SECTION_MOD',f'LF_{name}_SECTION_MOD').replace(f'LR_{short}_SECTION',f'LF_{short}_SECTION')
    s=s.replace('      USE LR_SECTION_MOD, ONLY: LR_UPDATE_SECTION',f'      USE LR_{name}_SECTION_MOD, ONLY: LR_{short}_MATERIAL\n      USE LF_CALLER, ONLY: LAYERED_FAILURE_CALLER')
    s=s.replace('POINTS,DIAG,','POINTS,MFUNC,LINEAR,D1,TIME,FAILURES,POINT_VALUES,REMOVED,DIAG,')
    s=s.replace('      REAL(C_DOUBLE),INTENT(INOUT) :: POINTS(7,3)', '''      REAL(C_DOUBLE),INTENT(INOUT) :: POINTS(7,3),FAILURES(3,3)
      REAL(C_DOUBLE),INTENT(IN) :: LINEAR(2),D1,TIME
      REAL(C_DOUBLE),INTENT(OUT) :: POINT_VALUES(13,3)
      INTEGER(C_INT),INTENT(IN) :: MFUNC
      INTEGER(C_INT),INTENT(OUT) :: REMOVED''')
    s=s.replace('DIAG(8)','DIAG(9)')
    if family=='Qeph': s=s.replace('TYPE(QE_GEOMETRY_),INTENT(IN) :: G','TYPE(QE_GEOMETRY_),INTENT(INOUT) :: G')
    a=s.index('      CALL LR_UPDATE_SECTION(')
    b=s.index('      IF(STATUS/=0) RETURN',a)+len('      IF(STATUS/=0) RETURN')
    dx='DX(1,:)' if family=='Qeph' else 'G%DEF(1,:)'
    replacement=f'''      CALL LAYERED_FAILURE_CALLER(MFUNC,NPTS,CURVE,BASIC,LINEAR,RATE_CONTROL,D1,STEP,TIME,
     . {dx},M%THK0(1:1),G%AREA(1:1),M%DM,POINTS,FAILURES,H%ACTIVE,
     . H%FOR_G,H%FOR,H%MOM,H%THK(1:1),H%EINT,POINT_VALUES,DIAG,REMOVED)
      G%OFF(1)=H%ACTIVE
      STATUS=0'''
    s=s[:a]+replacement+s[b:]
    # EPSD stays diagnostic channel8, not newly appended viscosity channel9.
    s=s.replace('H%EPSD=ONE*DIAG(9)', 'H%EPSD=ONE*DIAG(8)')
    emit(f'Failure{family}Section.F',s)
    s=(source/f'NativeLayered{family}Force.F').read_text()
    s=s.replace(f'LR_{name}_FORCE',f'LF_{name}_FORCE').replace(f'lr_{family.lower()}_force',f'lf_{family.lower()}_force')
    s=s.replace(f'LR_{name}_SECTION_MOD',f'LF_{name}_SECTION_MOD').replace(f'LR_{short}_SECTION',f'LF_{short}_SECTION')
    s=s.replace('RATE_CONTROL,POINTS,SECTION_DIAG,','RATE_CONTROL,POINTS,MFUNC,LINEAR,D1,TIME,FAILURES,POINT_VALUES,REMOVED,SECTION_DIAG,')
    s=s.replace('RATE_CONTROL,\n     . POINTS,SECTION_DIAG,','RATE_CONTROL,\n     . POINTS,MFUNC,LINEAR,D1,TIME,FAILURES,POINT_VALUES,REMOVED,SECTION_DIAG,')
    s=s.replace('      INTEGER(C_INT),VALUE :: NPTS','      INTEGER(C_INT),VALUE :: NPTS,MFUNC')
    s=s.replace('      REAL(C_DOUBLE),INTENT(INOUT) :: POINTS(7,3)', '''      REAL(C_DOUBLE),INTENT(INOUT) :: POINTS(7,3),FAILURES(3,3)
      REAL(C_DOUBLE),INTENT(IN) :: LINEAR(2),D1,TIME
      REAL(C_DOUBLE),INTENT(OUT) :: POINT_VALUES(13,3)
      INTEGER(C_INT),INTENT(OUT) :: REMOVED''')
    s=s.replace('SECTION_DIAG(8)','SECTION_DIAG(9)')
    obj='GEOMETRY' if family=='Qeph' else 'G'
    anchor=f'      CALL {short if family=="Qeph" else "T3"}_UNPACK_HISTORY(BASE_HISTORY,H)'
    assert anchor in s
    if family=='T3':
        # The old wrapper's active1-only guard follows complete OFF-independent
        # C3DEFO3/C3CURV3. C3COEF3 also declares but never reads its OFF dummy.
        # Restore actual activity only after this unchanged geometry/rate gate.
        anchor='      CALL T3_PACK_GEOMETRY(G,OUTPUT(1:38))'
    s=s.replace(anchor,anchor+f'\n      {obj}%OFF(1)=H%ACTIVE')
    emit(f'Failure{family}Force.F',s)

print(f'Verified {len(manifest["inputs"])} native inputs and 4 generated failure-force callers')
