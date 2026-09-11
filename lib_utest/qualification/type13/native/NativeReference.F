C SPDX-License-Identifier: AGPL-3.0-or-later
C Exact R4BUF3 arithmetic and conditions for finite explicit N3 geometry.
C Only native warning/write calls and their IALIGN bookkeeping are omitted.
C BRANCH exposes the selected N3/Y/X path. No geometry recurrence is claimed.
      SUBROUTINE TYPE13_REFERENCE(X,SEED,RESULT,BRANCH)
     . BIND(C,NAME='type13_native_reference')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: X(3,3),SEED(6)
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT(4)
      INTEGER(C_INT),INTENT(OUT) :: BRANCH
      DOUBLE PRECISION,PARAMETER :: EM5=1D-5
      INTEGER I,I1,I2,I3,ISK
      DOUBLE PRECISION X1,Y1,Z1,X0(1),RLOC(3,1),PRVC(3,1),NRLOC(1),NPRVC(1),SKEW(6,1)
      I=1
      I1=1
      I2=2
      I3=3
      ISK=1
      SKEW(:,1)=SEED
      BRANCH=0
#include "extracted/ReferenceChord.inc"
#include "extracted/ReferenceN3.inc"
      BRANCH=1
#include "extracted/ReferenceSkewY.inc"
      BRANCH=2
#include "extracted/ReferenceSkewX.inc"
#include "extracted/ReferenceCross.inc"
      ENDIF
      ENDIF
#include "extracted/ReferenceNormalize.inc"
      RESULT=(/RLOC(1,1),RLOC(2,1),RLOC(3,1),X0(1)/)
      END
