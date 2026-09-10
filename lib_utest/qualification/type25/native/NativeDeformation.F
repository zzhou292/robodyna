C SPDX-License-Identifier: AGPL-3.0-or-later
C OpenRadioss Copyright (C) 2026 Siemens. Exact R6DEF3 arithmetic fragments.
      SUBROUTINE TYPE25_DEFORMATION(OLD,VEL,OMEGA,MID,LENGTH,REFL,STEP,RESULT)
     . BIND(C,NAME='type25_native_deformation')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: OLD(6),VEL(3,2),OMEGA(3,2),MID(9),LENGTH(2),REFL,STEP
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT(6)
      DOUBLE PRECISION,PARAMETER :: ZERO=0D0,TWO=2D0,EM30=1D-30
      INTEGER I
      DOUBLE PRECISION DT05,DT1,VX21,VY21,VZ21,EPXY,EPXZ,X21,Y21,Z21,
     . RYAV1,RZAV1,AT,RYAV,RZAV,
     . VX1(1),VY1(1),VZ1(1),VX2(1),VY2(1),VZ2(1),RX1(1),RY1(1),RZ1(1),RX2(1),RY2(1),RZ2(1),
     . EXX2(1),EYX2(1),EZX2(1),EXY2(1),EYY2(1),EZY2(1),EXZ2(1),EYZ2(1),EZZ2(1),
     . DXOLD(1),DYOLD(1),DZOLD(1),DX(1),DY(1),DZ(1),RX(1),RY(1),RZ(1),CRIT(1),X0DP(1),ALDP(1),AL2DP(1)
      I=1
      DT1=STEP
      DT05=.5D0*DT1
      X0DP=REFL
      ALDP=LENGTH(1)
      AL2DP=LENGTH(2)
      EXX2=MID(1)
      EXY2=MID(2)
      EXZ2=MID(3)
      EYX2=MID(4)
      EYY2=MID(5)
      EYZ2=MID(6)
      EZX2=MID(7)
      EZY2=MID(8)
      EZZ2=MID(9)
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
      DXOLD=OLD(1)
      DYOLD=OLD(2)
      DZOLD=OLD(3)
#include "extracted/Translation.inc"
      DXOLD=OLD(4)
      DYOLD=OLD(5)
      DZOLD=OLD(6)
#include "extracted/Rotation.inc"
      RESULT=(/DX(1),DY(1),DZ(1),RX(1),RY(1),RZ(1)/)
      END
