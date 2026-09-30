C SPDX-License-Identifier: AGPL-3.0-or-later
C OpenRadioss Copyright (C) 2026 Siemens. Exact R4CUM3 value operations.
      SUBROUTINE TYPE25_SCATTER(AXES,LENGTH,FORCE,COUPLE,RESULT)
     . BIND(C,NAME='type25_native_scatter')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: AXES(9),LENGTH,FORCE(3),COUPLE(3)
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT(12)
      DOUBLE PRECISION,PARAMETER :: HALF=.5D0
      INTEGER I
      DOUBLE PRECISION EXX(1),EYX(1),EZX(1),EXY(1),EYY(1),EZY(1),EXZ(1),EYZ(1),EZZ(1),
     . FORX(1),FORY(1),FORZ(1),XMOM(1),YMOM(1),ZMOM(1),AL(1),F11(1),F21(1),F31(1),
     . M11(1),M21(1),M31(1),M12(1),M22(1),M32(1),YMOM1,ZMOM1,YMOM2,ZMOM2
      I=1
      EXX=AXES(1)
      EXY=AXES(2)
      EXZ=AXES(3)
      EYX=AXES(4)
      EYY=AXES(5)
      EYZ=AXES(6)
      EZX=AXES(7)
      EZY=AXES(8)
      EZZ=AXES(9)
      AL=LENGTH
      FORX=FORCE(1)
      FORY=FORCE(2)
      FORZ=FORCE(3)
      XMOM=COUPLE(1)
      YMOM=COUPLE(2)
      ZMOM=COUPLE(3)
#include "extracted/ForceToWorld.inc"
#include "extracted/CouplesToWorld.inc"
      RESULT=(/F11(1),F21(1),F31(1),M11(1),M21(1),M31(1),-F11(1),-F21(1),-F31(1),-M12(1),-M22(1),-M32(1)/)
      END
