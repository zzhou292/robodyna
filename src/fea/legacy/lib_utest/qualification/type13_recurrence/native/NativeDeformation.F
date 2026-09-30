C SPDX-License-Identifier: AGPL-3.0-or-later
C Exact R4DEF3 source fragments; wrapper binds current/midpoint native frame.
      SUBROUTINE TYPE13_DEFORMATION(MID,LENGTH,VEL,OMEGA,BASE,L0,STEP,FRESH,RESULT,REFERENCE_LENGTH)
     . BIND(C,NAME='type13_native_deformation')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN)::MID(9),LENGTH(2),VEL(3,2),OMEGA(3,2),BASE(6),L0,STEP
      INTEGER(C_INT),INTENT(IN)::FRESH
      REAL(C_DOUBLE),INTENT(OUT)::RESULT(6),REFERENCE_LENGTH
      INTEGER,PARAMETER::NEL=1
      DOUBLE PRECISION,PARAMETER::ZERO=0D0,TWO=2D0,EM30=1D-30
      INTEGER I,SCODVER
      DOUBLE PRECISION TT,DT1,DT05,X0(1),X0_ERR(1),X0DP(1),ALDP(1),AL2DP(1),
     . VX1(1),VY1(1),VZ1(1),VX2(1),VY2(1),VZ2(1),RX1(1),RY1(1),RZ1(1),RX2(1),RY2(1),RZ2(1),
     . EXX2(1),EYX2(1),EZX2(1),EXY2(1),EYY2(1),EZY2(1),EXZ2(1),EYZ2(1),EZZ2(1),
     . DX(1),DY(1),DZ(1),DXOLD(1),DYOLD(1),DZOLD(1),RX(1),RY(1),RZ(1),CRIT(1),
     . VX21,VY21,VZ21,EPXY,EPXZ,X21,Y21,Z21,RYAV1,RZAV1,AT,RYAV,RZAV
      TT=1D0
      IF(FRESH/=0)TT=ZERO
      SCODVER=101
      DT1=STEP
      DT05=.5D0*DT1
      X0=L0
      X0_ERR=ZERO
      ALDP=LENGTH(1)
      AL2DP=LENGTH(2)
      VX1=VEL(1,1)
      VY1=VEL(2,1)
      VZ1=VEL(3,1)
      VX2=VEL(1,2)
      VY2=VEL(2,2)
      VZ2=VEL(3,2)
      RX1=OMEGA(1,1)
      RY1=OMEGA(2,1)
      RZ1=OMEGA(3,1)
      RX2=OMEGA(1,2)
      RY2=OMEGA(2,2)
      RZ2=OMEGA(3,2)
      EXX2=MID(1)
      EXY2=MID(2)
      EXZ2=MID(3)
      EYX2=MID(4)
      EYY2=MID(5)
      EYZ2=MID(6)
      EZX2=MID(7)
      EZY2=MID(8)
      EZZ2=MID(9)
      DXOLD=BASE(1)
      DYOLD=BASE(2)
      DZOLD=BASE(3)
#include "extracted/FreshLength.inc"
#include "extracted/ReferenceLength.inc"
#include "extracted/Translation.inc"
      DXOLD=BASE(4)
      DYOLD=BASE(5)
      DZOLD=BASE(6)
#include "extracted/Rotation.inc"
      RESULT=(/DX(1),DY(1),DZ(1),RX(1),RY(1),RZ(1)/)
      REFERENCE_LENGTH=X0DP(1)
      END
