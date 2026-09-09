! Adapter to pinned OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later.
! See original/LICENSE.md and source-manifest.json. Native geometry routines
! are complete unchanged extracts. The centered uniform CINMAS expression is
! adapted below from original starter/.../cinmas.F:851-874,925-932,1381-1412.
      SUBROUTINE QE_Q1_STARTUP(X, MATERIAL, OUTPUT)
     . BIND(C,NAME="qeph_q1_startup")
      USE ISO_C_BINDING, ONLY: C_DOUBLE
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "vect01_c.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,4), MATERIAL(3)
      REAL(C_DOUBLE), INTENT(OUT) :: OUTPUT(34)
      MY_REAL P(MVSIZ,3,4), F(MVSIZ,3,3), AREA(MVSIZ),
     . PX1(MVSIZ),PX2(MVSIZ),PY1(MVSIZ),PY2(MVSIZ),
     . LX(MVSIZ,4),LY(MVSIZ,4),MASS,PHYSICAL,ADDED,TOTAL
      INTEGER I,J,K
      P=ZERO
      F=ZERO
      AREA=ZERO
      LX=ZERO
      LY=ZERO
      LFT=1
      LLT=1
      DO J=1,4
        P(1,:,J)=X(:,J)
      END DO
      CALL CNEVECI(1,1,AREA,
     . P(1,1,1),P(1,1,2),P(1,1,3),P(1,1,4),
     . P(1,2,1),P(1,2,2),P(1,2,3),P(1,2,4),
     . P(1,3,1),P(1,3,2),P(1,3,3),P(1,3,4),
     . F(1,1,1),F(1,1,2),F(1,1,3),
     . F(1,2,1),F(1,2,2),F(1,2,3),
     . F(1,3,1),F(1,3,2),F(1,3,3))
      CALL CDERII(PX1,PX2,PY1,PY2,
     . P(1,1,1),P(1,1,2),P(1,1,3),P(1,1,4),
     . P(1,2,1),P(1,2,2),P(1,2,3),P(1,2,4),
     . P(1,3,1),P(1,3,2),P(1,3,3),P(1,3,4),
     . F(1,1,1),F(1,1,2),F(1,1,3),
     . F(1,2,1),F(1,2,2),F(1,2,3),
     . F(1,3,1),F(1,3,2),F(1,3,3),
     . LX(1,2),LX(1,3),LX(1,4),LY(1,2),LY(1,3),LY(1,4))
      DO I=1,3
        DO J=1,3
          OUTPUT(3*(I-1)+J)=F(1,I,J)
        END DO
      END DO
      OUTPUT(10)=AREA(1)
      OUTPUT(11:14)=[PX1(1),PX2(1),-PX1(1),-PX2(1)]
      OUTPUT(15:18)=[PY1(1),PY2(1),-PY1(1),-PY2(1)]
      DO J=1,4
        K=19+3*(J-1)
        OUTPUT(K:K+2)=[LX(1,J),LY(1,J),ZERO]
      END DO
! CINMAS selected path: uniform centered section, IHBE>=11, INER_9_12=0.
! Preserve source total-inertia operation order; do not replace it by a sum
! of the independently reported physical/artificial diagnostic partitions.
      MASS=MATERIAL(1)*MATERIAL(2)*AREA(1)*FOURTH
      PHYSICAL=MASS*MATERIAL(2)*MATERIAL(2)*ONE_OVER_12
      ADDED=MASS*(AREA(1)/12)
      TOTAL=MASS*(AREA(1)/12+MATERIAL(2)*MATERIAL(2)*(ONE_OVER_12+ZERO))
      OUTPUT(31:34)=[MASS,PHYSICAL,ADDED,TOTAL]
      END
