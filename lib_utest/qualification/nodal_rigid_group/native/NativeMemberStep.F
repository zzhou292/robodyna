! >2-member acceleration, reactions, kick and drift in separately authored
! bounded packet context. OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later.
      SUBROUTINE RBG_MEMBERS(COUNT,CENTER,BASE_V,PLUS_W,BODY_A,DURATION,
     . POSITION,VELOCITY,OMEGA,MASS,INERTIA,FORCES,COUPLES,
     . OUT_A,OUT_AR,OUT_X,OUT_V,OUT_W,OUT_R,OUT_RR)
     . BIND(C,NAME="nodal_rigid_native_member_step")
      USE ISO_C_BINDING, ONLY:C_DOUBLE,C_INT
      IMPLICIT NONE
      INTEGER(C_INT),INTENT(IN)::COUNT
      REAL(C_DOUBLE),INTENT(IN)::CENTER(3),BASE_V(3),PLUS_W(3),BODY_A(3),
     . DURATION(3),POSITION(3,COUNT),VELOCITY(3,COUNT),OMEGA(3,COUNT),
     . MASS(COUNT),INERTIA(COUNT),FORCES(3,COUNT),COUPLES(3,COUNT)
      REAL(C_DOUBLE),INTENT(OUT)::OUT_A(3,COUNT),OUT_AR(3,COUNT),
     . OUT_X(3,COUNT),OUT_V(3,COUNT),OUT_W(3,COUNT),
     . OUT_R(3,COUNT),OUT_RR(3,COUNT)
      REAL(C_DOUBLE) X(3,COUNT+1),V(3,COUNT+1),VR(3,COUNT+1),
     . A(3,COUNT+1),AR(3,COUNT+1),D(3,COUNT+1),MS(COUNT),IN(COUNT),
     . FREAC(6,COUNT),R(3,COUNT),RR(3,COUNT),VG(3),DT12,DT2,USDT,
     . AM1,AM2,AM3,V1X2,V2X1,V2X3,V3X2,V3X1,V1X3,VX1,VX2,VX3,VDT
      REAL(C_DOUBLE),PARAMETER::ZERO=0.D0,HALF=.5D0
      INTEGER M,N,I,NSN,NUMNOD,NOD(COUNT)
      M=COUNT+1
      NSN=COUNT
      NUMNOD=COUNT
      X(:,1:COUNT)=POSITION
      X(:,M)=CENTER
      V(:,1:COUNT)=VELOCITY
      V(:,M)=BASE_V
      VR(:,1:COUNT)=OMEGA
      VG=PLUS_W
      AM1=BODY_A(1)
      AM2=BODY_A(2)
      AM3=BODY_A(3)
      DT12=DURATION(2)
      DT2=DURATION(3)
      USDT=1.D0/DT12
      MS=MASS
      IN=INERTIA
      FREAC(1:3,:)=FORCES
      FREAC(4:6,:)=COUPLES
      D=ZERO
      DO I=1,COUNT
        NOD(I)=I
      ENDDO
#include "extracted/MemberAcceleration.inc"
#include "extracted/MemberReaction.inc"
      OUT_A=A(:,1:COUNT)
      OUT_AR=AR(:,1:COUNT)
      OUT_R=R
      OUT_RR=RR
#include "extracted/TranslationKick.inc"
#include "extracted/RotationKick.inc"
#include "extracted/PositionDrift.inc"
      OUT_X=X(:,1:COUNT)
      OUT_V=V(:,1:COUNT)
      OUT_W=VR(:,1:COUNT)
      END
