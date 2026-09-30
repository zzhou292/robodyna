! Qualification packet, not native engine startup or publication.
! OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later.
      SUBROUTINE RBG_PRIMARY(FRAME,PRINCIPAL,CENTER,VELOCITY,OMEGA,MASS,
     . FORCE,TORQUE,DURATION,OUT_FRAME,SAVED,ACCEL,ALPHA,OUT_X,OUT_V,OUT_W)
     . BIND(C,NAME="nodal_rigid_native_primary_step")
      USE ISO_C_BINDING, ONLY:C_DOUBLE
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN)::FRAME(9),PRINCIPAL(3),CENTER(3),
     . VELOCITY(3),OMEGA(3),MASS,FORCE(3),TORQUE(3),DURATION(3)
      REAL(C_DOUBLE),INTENT(OUT)::OUT_FRAME(9),SAVED(3),ACCEL(3),ALPHA(3),
     . OUT_X(3),OUT_V(3),OUT_W(3)
      REAL(C_DOUBLE) RBY(12),VI(3),AM(3,1),IN(1),WA1,WA2,WA3,
     . VR(3,1),V(3,1),X(3,1),D(3,1),A(3,1),AR(3,1),DT12,DT2,VDT
      REAL(C_DOUBLE),PARAMETER::ZERO=0.D0
      INTEGER M,N,NUMNOD
      M=1
      NUMNOD=1
      RBY(1:9)=FRAME
      RBY(10:12)=PRINCIPAL
      VR(:,1)=OMEGA
      V(:,1)=VELOCITY
      X(:,1)=CENTER
      D=ZERO
      DT12=DURATION(2)
      DT2=DURATION(3)
      IN(1)=MINVAL(PRINCIPAL)
      WA1=TORQUE(1)
      WA2=TORQUE(2)
      WA3=TORQUE(3)
#include "extracted/SavedBodyOmega.inc"
      SAVED=VI
      CALL NRG_ROTBMR(VI,RBY,DURATION(1))
#include "extracted/TorqueToFrame.inc"
#include "extracted/GyroscopicTorque.inc"
#include "extracted/PrincipalAcceleration.inc"
#include "extracted/AccelerationToWorld.inc"
      AR(:,1)=AM(:,1)/IN(1)
      A(:,1)=FORCE/MASS
      ACCEL=A(:,1)
      ALPHA=AR(:,1)
      OUT_FRAME=RBY(1:9)
#include "extracted/TranslationKick.inc"
#include "extracted/RotationKick.inc"
#include "extracted/PositionDrift.inc"
      OUT_X=X(:,1)
      OUT_V=V(:,1)
      OUT_W=VR(:,1)
      END
