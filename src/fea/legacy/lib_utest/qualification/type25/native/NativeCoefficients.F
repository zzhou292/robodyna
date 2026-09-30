C SPDX-License-Identifier: AGPL-3.0-or-later
C OpenRadioss Copyright (C) 2026 Siemens. RMASS and unscaled R2LEN3 operations.
      SUBROUTINE TYPE25_MASS(MASS,INERTIA,RESULT)
     . BIND(C,NAME='type25_native_mass')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: MASS,INERTIA
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT(2)
      DOUBLE PRECISION,PARAMETER :: HALF=.5D0
      INTEGER I,NFT,IXR(3,1)
      DOUBLE PRECISION GEO(9,1),XL(1),EMS(1),XI,MSR(3,1),INR(3,1)
      I=1
      NFT=0
      IXR=1
      XL=1D0
      GEO=0D0
      GEO(1,1)=MASS
      GEO(9,1)=INERTIA
#include "extracted/HalfMass.inc"
#include "extracted/HalfInertia.inc"
      RESULT=(/MSR(1,1),INR(1,1)/)
      END
      SUBROUTINE TYPE25_DT(MASS,INERTIA,KT,KR,CT,CR,RESULT)
     . BIND(C,NAME='type25_native_dt')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: MASS,INERTIA,KT,KR,CT,CR
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT
      DOUBLE PRECISION,PARAMETER :: EM15=1D-15,ONE=1D0
      INTEGER I
      DOUBLE PRECISION XM(1),XIN(1),XKM(1),XKR(1),XCM(1),XCR(1),DT(1),DTC(1)
      I=1
      XM=MASS
      XIN=INERTIA
      XKM=KT
      XKR=KR
      XCM=CT
      XCR=CR
#include "extracted/CriticalStep.inc"
      RESULT=DT(1)
      END
