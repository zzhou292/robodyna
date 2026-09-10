C SPDX-License-Identifier: AGPL-3.0-or-later
C OpenRadioss Copyright (C) 2026 Siemens. Exact REDEF3 and R6DEF3 operations.
      SUBROUTINE TYPE25_RESPONSE(K,C,X,OLDX,OLDF,OLDE,MINIMUM,MAXIMUM,WEIGHT,EXPONENT,
     . STEP,ACTIVE,PREVCRIT,FORCE,WORK,NEXTACTIVE,NEXTCRIT)
     . BIND(C,NAME='type25_native_response')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: K(4),C(4),X(4),OLDX(4),OLDF(4),OLDE(4),
     . MINIMUM(4),MAXIMUM(4),WEIGHT(4),EXPONENT(4),STEP,PREVCRIT
      INTEGER(C_INT),INTENT(IN) :: ACTIVE
      REAL(C_DOUBLE),INTENT(OUT) :: FORCE(4),WORK(4),NEXTCRIT
      INTEGER(C_INT),INTENT(OUT) :: NEXTACTIVE
      DOUBLE PRECISION,PARAMETER :: ZERO=0D0,ONE=1D0,HALF=.5D0
      INTEGER I,J,NEL,NINDX,INDX(1),IDEL7NOK,IFAIL(1)
      DOUBLE PRECISION FX(1),XK(1),DX(1),DXOLD(1),XC(1),DVX(1),D(1),AK(1),B(1),EE(1),GX(1),GF3(1),GX2(1),
     . OFF(1),E(1),FOLD(1),DVV,DFAC,CRIT(1),XA,XB,DLIM,XL0(1),CRITNEW(1)
      I=1
      NEL=1
      NINDX=0
      IDEL7NOK=0
      IFAIL=1
      XL0=1D0
      CRIT=0D0
      CRITNEW=PREVCRIT
      OFF=REAL(ACTIVE,KIND=8)
      D=1D0
      AK=1D0
      B=0D0
      EE=0D0
      GX=0D0
      GF3=1D0
      GX2=0D0
      DO J=1,4
        XK=K(J)
        XC=C(J)
        DX=X(J)
        DXOLD=OLDX(J)
        FOLD=OLDF(J)
        E=OLDE(J)
        DVX=(DX-DXOLD)/STEP
#include "extracted/LinearForce.inc"
#include "extracted/ForceWork.inc"
        FORCE(J)=FX(1)
        WORK(J)=E(1)
        IF(OFF(1)==ONE)THEN
          IF(FX(1)>ZERO)THEN
            DLIM=FX(1)/MAXIMUM(J)
          ELSE
            DLIM=FX(1)/MINIMUM(J)
          ENDIF
          XA=WEIGHT(J)
          XB=EXPONENT(J)
#include "extracted/Criterion.inc"
        ENDIF
      ENDDO
#include "extracted/Failure.inc"
      NEXTACTIVE=NINT(OFF(1))
      NEXTCRIT=CRITNEW(1)
      END
