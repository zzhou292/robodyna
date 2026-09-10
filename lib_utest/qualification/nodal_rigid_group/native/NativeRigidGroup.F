! Qualification wrapper around exact, manifest-verified OpenRadioss arithmetic.
! OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later.
! The small packet setup is authored here; extracted statements are unchanged.
      SUBROUTINE RBG_INERTIA(COUNT,POSITION,CENTER,MASS,INERTIA,INITIAL,OUTPUT)
     . BIND(C,NAME="nodal_rigid_native_inertia")
      USE ISO_C_BINDING, ONLY:C_INT,C_DOUBLE
      IMPLICIT NONE
      INTEGER(C_INT),INTENT(IN)::COUNT
      REAL(C_DOUBLE),INTENT(IN)::POSITION(3,COUNT),CENTER(3),MASS(COUNT),
     . INERTIA(COUNT),INITIAL(9)
      REAL(C_DOUBLE),INTENT(OUT)::OUTPUT(9)
      INTEGER NSL,I,N,M,NRB,LSN(COUNT)
      REAL(C_DOUBLE) X(3,COUNT+1),MS_LOC(COUNT),IN_LOC(COUNT),RBY(9,1),
     . XX,XY,XZ,YY,YZ,ZZ
      NSL=COUNT
      NRB=1
      M=COUNT+1
      X(:,1:COUNT)=POSITION
      X(:,M)=CENTER
      MS_LOC=MASS
      IN_LOC=INERTIA
      RBY(:,1)=INITIAL
      DO I=1,COUNT
        LSN(I)=I
      ENDDO
#include "extracted/InertiaLoop.inc"
      OUTPUT=RBY(:,1)
      END

      SUBROUTINE RBG_ISPHER2(RAW,OUTPUT,REACHED)
     . BIND(C,NAME="nodal_rigid_native_ispher2")
      USE ISO_C_BINDING, ONLY:C_INT,C_DOUBLE
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN)::RAW(3)
      REAL(C_DOUBLE),INTENT(OUT)::OUTPUT(3)
      INTEGER(C_INT),INTENT(OUT)::REACHED
      REAL(C_DOUBLE) RBY(12,1),INMIN,INMAX
      REAL(C_DOUBLE),PARAMETER::EM03=1.D-3,EM01=1.D-1
      INTEGER NRB
      NRB=1
      RBY=0.D0
      RBY(10:12,1)=RAW
      REACHED=0
#include "extracted/Ispher2.inc"
            REACHED=1
          ENDIF
      OUTPUT=RBY(10:12,1)
      END

      SUBROUTINE RBG_WRENCH(COUNT,POSITION,CENTER,FORCES,COUPLES,OUTPUT)
     . BIND(C,NAME="nodal_rigid_native_wrench")
      USE ISO_C_BINDING, ONLY:C_INT,C_DOUBLE
      IMPLICIT NONE
      INTEGER(C_INT),INTENT(IN)::COUNT
      REAL(C_DOUBLE),INTENT(IN)::POSITION(3,COUNT),CENTER(3),
     . FORCES(3,COUNT),COUPLES(3,COUNT)
      REAL(C_DOUBLE),INTENT(OUT)::OUTPUT(6)
      REAL(C_DOUBLE) X(3,COUNT+1),AF(3,COUNT+1),AM(3,COUNT+1),
     . F3(COUNT),F4(COUNT),F5(COUNT),F6(COUNT),F7(COUNT),F8(COUNT)
      INTEGER I,N,M
      M=COUNT+1
      X(:,1:COUNT)=POSITION
      X(:,M)=CENTER
      AF(:,1:COUNT)=FORCES
      AM(:,1:COUNT)=COUPLES
      DO I=1,COUNT
        N=I
#include "extracted/MemberWrench.inc"
      ENDDO
      OUTPUT=0.D0
      DO I=1,COUNT
        OUTPUT(1)=OUTPUT(1)+F3(I)
        OUTPUT(2)=OUTPUT(2)+F4(I)
        OUTPUT(3)=OUTPUT(3)+F5(I)
        OUTPUT(4)=OUTPUT(4)+F6(I)
        OUTPUT(5)=OUTPUT(5)+F7(I)
        OUTPUT(6)=OUTPUT(6)+F8(I)
      ENDDO
      END

      SUBROUTINE RBG_GYRO(PRINCIPAL,OMEGA,TORQUE,OUTPUT)
     . BIND(C,NAME="nodal_rigid_native_gyro")
      USE ISO_C_BINDING, ONLY:C_DOUBLE
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN)::PRINCIPAL(3),OMEGA(3),TORQUE(3)
      REAL(C_DOUBLE),INTENT(OUT)::OUTPUT(3)
      REAL(C_DOUBLE) RBY(12),VI(3),AM(3,1),IN(1),WA1,WA2,WA3
      INTEGER M
      M=1
      RBY=0.D0
      RBY(10:12)=PRINCIPAL
      VI=OMEGA
      AM(:,1)=TORQUE
      IN(1)=MINVAL(PRINCIPAL)
#include "extracted/GyroscopicTorque.inc"
#include "extracted/PrincipalAcceleration.inc"
      OUTPUT(1)=WA1/IN(1)
      OUTPUT(2)=WA2/IN(1)
      OUTPUT(3)=WA3/IN(1)
      END
