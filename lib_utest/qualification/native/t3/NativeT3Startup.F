! SPDX-License-Identifier: AGPL-3.0-or-later
! Adapter to OpenRadioss, Copyright (C) 2026 Siemens, pinned a62b27e6.
! See source-manifest.json and ../qeph/original/LICENSE.md.
! Only C3EVEC3 below is a complete native leaf invocation. Selected C3INMAS,
! C3DERII and SPMD_MSIN expressions are explicitly adapted in the second
! operation; unrelated full starter drivers are not executed or stubbed.
      SUBROUTINE T3_R1_FRAME(X,OUTPUT)
     . BIND(C,NAME="t3_r1_frame")
      USE ISO_C_BINDING, ONLY: C_DOUBLE
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,3)
      REAL(C_DOUBLE), INTENT(OUT) :: OUTPUT(13)
      MY_REAL P(MVSIZ,3,3),F(MVSIZ,3,3),AREA(MVSIZ),
     . X31(MVSIZ),Y31(MVSIZ),Z31(MVSIZ),
     . X2(MVSIZ),X3(MVSIZ),Y3(MVSIZ)
      INTEGER I,J
      P=ZERO
      F=ZERO
      AREA=ZERO
      X31=ZERO
      Y31=ZERO
      Z31=ZERO
      X2=ZERO
      X3=ZERO
      Y3=ZERO
      DO J=1,3
        P(1,:,J)=X(:,J)
      ENDDO
      CALL C3EVEC3(1,1,AREA,
     . P(1,1,1),P(1,1,2),P(1,1,3),
     . P(1,2,1),P(1,2,2),P(1,2,3),
     . P(1,3,1),P(1,3,2),P(1,3,3),
     . F(1,1,1),F(1,1,2),F(1,1,3),
     . F(1,2,1),F(1,2,2),F(1,2,3),
     . F(1,3,1),F(1,3,2),F(1,3,3),X31,Y31,Z31,X2,X3,Y3)
      DO I=1,3
        DO J=1,3
          OUTPUT(3*(I-1)+J)=F(1,I,J)
        ENDDO
      ENDDO
      OUTPUT(10:13)=[AREA(1),X2(1),X3(1),Y3(1)]
      END

      SUBROUTINE T3_R1_SELECTED_MASS(LOCAL,AREA,MATERIAL,
     . ACOS_LIMIT,OUTPUT,STATUS)
     . BIND(C,NAME="t3_r1_selected_mass")
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
      USE, INTRINSIC :: IEEE_ARITHMETIC, ONLY: IEEE_IS_FINITE
#include "implicit_f.inc"
      REAL(C_DOUBLE), INTENT(IN) :: LOCAL(3),AREA,MATERIAL(2),
     . ACOS_LIMIT
      REAL(C_DOUBLE), INTENT(OUT) :: OUTPUT(26)
      INTEGER(C_INT), INTENT(OUT) :: STATUS
      MY_REAL X2,X3,Y3,RHO,THK,EM,XI,PHYSICAL,ADDED,
     . AA,A2,B2,BB,C2,CC,COSINE(3),PTG(3),AL1,AL2,AL3,ALMAX
      INTEGER I
      STATUS=1
      X2=LOCAL(1)
      X3=LOCAL(2)
      Y3=LOCAL(3)
      RHO=MATERIAL(1)
      THK=MATERIAL(2)
! C3INMAS:1137-1143. Check the actual native arguments before the source
! ACOS expressions. Reject unsupported geometry; never clamp or normalize.
      AA = X2
      A2 = AA**2
      B2 = (X2-X3)**2 + Y3**2
      BB = SQRT(B2)
      C2 = X3**2 + Y3**2
      CC = SQRT(C2)
      COSINE(1)=(A2 + C2 - B2)/(TWO * AA * CC)
      COSINE(2)=(A2 + B2 - C2)/(TWO * AA * BB)
      COSINE(3)=(B2 + C2 - A2)/(TWO * BB * CC)
      DO I=1,3
        IF(.NOT.IEEE_IS_FINITE(COSINE(I))) RETURN
        IF(ABS(COSINE(I))>ACOS_LIMIT) RETURN
      ENDDO
      PTG(1) = ACOS((A2 + C2 - B2)/(TWO * AA * CC)) / PI
      PTG(2) = ACOS((A2 + B2 - C2)/(TWO * AA * BB)) / PI
      PTG(3) = ACOS((B2 + C2 - A2)/(TWO * BB * CC)) / PI
! Selected uniform C3INMAS:598,1116. Preserve native total expression order.
      EM=RHO*THK*AREA
      XI=EM*(AREA/NINE_OVER_2+THK*THK*ONE_OVER_12)
! Separate diagnostic partitions, never recombined to replace XI.
      PHYSICAL=EM*THK*THK*ONE_OVER_12
      ADDED=EM*(AREA/NINE_OVER_2)
      OUTPUT(1:3)=COSINE
      OUTPUT(4:6)=PTG
      DO I=1,3
! SPMD_MSIN:557-558, contribution to initially zero one-element node sums.
        OUTPUT(6+I)=ZERO+EM*PTG(I)
        OUTPUT(15+I)=ZERO+XI*PTG(I)
        OUTPUT(9+I)=PHYSICAL*PTG(I)
        OUTPUT(12+I)=ADDED*PTG(I)
      ENDDO
      OUTPUT(19:22)=[EM,XI,PHYSICAL,ADDED]
! C3DERII:106-109,151. Characteristic length only; no full nodal stiffness
! or native timestep diagnostic is claimed. ISMSTR=-1 slots remain zero.
      AL1 = X2 * X2
      AL2 = (X3-X2) * (X3-X2) + Y3 * Y3
      AL3 = X3 * X3 + Y3 * Y3
      ALMAX = MAX(AL1,AL2,AL3)
      OUTPUT(23)=TWO*AREA / SQRT(ALMAX)
      OUTPUT(24:26)=ZERO
      STATUS=0
      END
