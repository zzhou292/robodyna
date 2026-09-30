C SPDX-License-Identifier: AGPL-3.0-or-later
C OpenRadioss Copyright (C) 2026 Siemens. Finite/no-N3 R4BUF3 operations.
C Only warning/output branches are omitted; original scalar math is included.
      SUBROUTINE TYPE25_REFERENCE(X,SEED,RESULT)
     . BIND(C,NAME='type25_native_reference')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: X(3,2),SEED(6)
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT(4)
      DOUBLE PRECISION,PARAMETER :: EM5=1D-5
      INTEGER I,I1,I2,ISK
      DOUBLE PRECISION X1,Y1,Z1,X0(1),RLOC(3,1),PRVC(3,1),NRLOC(1),NPRVC(1),SKEW(6,1)
      I=1
      I1=1
      I2=2
      ISK=1
      SKEW(:,1)=SEED
#include "extracted/ReferenceChord.inc"
#include "extracted/ReferenceSeedY.inc"
#include "extracted/ReferenceSeedX.inc"
#include "extracted/ReferenceCross.inc"
      ENDIF
#include "extracted/ReferenceNormalize.inc"
      RESULT=(/RLOC(1,1),RLOC(2,1),RLOC(3,1),X0(1)/)
      END
