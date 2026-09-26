#!/usr/bin/env python3
"""Observe native IC1/IC2 return and VOLINT without production calls."""
import argparse,importlib.util
from pathlib import Path
ROOT=Path(__file__).resolve().parent
original=ROOT.parent.parent/'radioss_type25_main_geometry/native/prepare.py'
spec=importlib.util.spec_from_file_location('reader_geometry_donor',original)
native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
def generate():
    text=native.generate() # Validates every pinned byte extent/SHA/git blob.
    text=text.replace('RD_MAIN_','RD_READER_')
    text=text.replace('RD_READER_GEOMETRY(P,LAYOUT,R,SLOTS,REVERSED)',
        'RD_READER_GEOMETRY(P,LAYOUT,BRANCH,R,SLOTS,REVERSED)')
    text=text.replace(':: LAYOUT\n',':: LAYOUT,BRANCH\n')
    text=text.replace('CALL RD_READER_ORIENT(P,IRECT,IXS,N1,N2,N3,AREA,DDS,NINV)',
        'CALL RD_READER_ORIENT(P,IRECT,IXS,N1,N2,N3,AREA,DDS,NINV,BRANCH)')
    text=text.replace('R=(/N1,N2,N3,AREA,VOL,DDS/)',
        'R(1:5)=(/N1,N2,N3,AREA,VOL/)\n      IF(BRANCH==1)R(6)=DDS')
    text=text.replace('SUBROUTINE RD_READER_ORIENT(X,IRECT,IXS,N1,N2,N3,AREA,DDS,NINV)',
        'SUBROUTINE RD_READER_ORIENT(X,IRECT,IXS,N1,N2,N3,AREA,DDS,NINV,BRANCH)')
    text=text.replace('INTEGER,INTENT(IN) :: IXS(13,1)',
        'INTEGER,INTENT(IN) :: IXS(13,1),BRANCH')
    text=text.replace('! Membership/caller contract already resolved: one exterior EightSlot, IR=0.',
        '! Effective caller support IC is explicit; no membership claim, IR=0.')
    assert text.count('      IC=1\n')==1
    text=text.replace('      IC=1\n','      IC=BRANCH\n')
    assert 'IF(IC>=2)RETURN' in text and 'IF(BRANCH==1)R(6)=DDS' in text
    text += """
      SUBROUTINE RD_READER_RAW_VOLUME(P,R) BIND(C)
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: P(3,8)
      REAL(C_DOUBLE),INTENT(OUT) :: R
      CALL RD_READER_VOLUME(P(1,:),P(2,:),P(3,:),R)
      END SUBROUTINE
"""
    return text
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path);p.add_argument('--check',action='store_true');a=p.parse_args()
    text=generate()
    if a.output:
        path=a.output/'Reader.F'
        if a.check:assert path.read_text()==text
        else:path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)
    print('Pinned complete selected NORMA1D/VOLINT/INSOL3D geometry with explicit IC1/2 verified')
