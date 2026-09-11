C SPDX-License-Identifier: AGPL-3.0-or-later
C R4DEF3 source coefficient preparation followed by unscaled R2LEN3.
      SUBROUTINE TYPE13_STABILITY(MASS,INERTIA,K,L0,LENGTH,RESULT)
     . BIND(C,NAME='type13_native_stability')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN)::MASS,INERTIA,K(6),L0,LENGTH
      REAL(C_DOUBLE),INTENT(OUT)::RESULT(3)
      INTEGER,PARAMETER::NEL=1,JFT=1,JLT=1
      DOUBLE PRECISION,PARAMETER::ZERO=0D0,ONE=1D0,EM15=1D-15
      INTEGER I,MGN(1),IFAIL(1),IFAIL2(1)
      DOUBLE PRECISION GEO(146,1),EPLA(1),XM(1),XIN(1),XK(1),YK(1),ZK(1),XC(1),YC(1),ZC(1),
     . HX(1),HY(1),HZ(1),XH(1),XHR(1),XKM(1),XCM(1),XKR(1),XCR(1),VRT(1),VRR(1),
     . ALDP(1),XL0(1),XKA,YKA,ZKA,DT(1),DTC(1),KT,KR
      GEO=ZERO
      MGN=1
      GEO(1,1)=MASS
      GEO(9,1)=INERTIA
      GEO(3,1)=K(1)
      GEO(10,1)=K(2)
      GEO(15,1)=K(3)
      GEO(19,1)=K(4)
      GEO(23,1)=K(5)
      GEO(27,1)=K(6)
      GEO(41,1)=ONE
      GEO(45,1)=ONE
      GEO(49,1)=ONE
      GEO(53,1)=ONE
      GEO(57,1)=ONE
      GEO(61,1)=ONE
      ALDP=LENGTH
      XL0=L0
#include "extracted/TranslationCoefficients.inc"
#include "extracted/RotationCoefficients.inc"
#include "extracted/NormalizeCoefficients.inc"
      KT=XKM(1)
      KR=XKR(1)
#include "extracted/CriticalStep.inc"
      RESULT=(/DT(1),KT,KR/)
      END
