! Authored bounded wrapper; exact RGBCOR correction fragments. DT1=0 and
! supplied velocities qualify value arithmetic only, not native collocation.
! OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later.
      SUBROUTINE RBG_KINETIC(COUNT,FRAME,PRINCIPAL,GROUP_VR,
     . MEMBER_V,MEMBER_VR,MASS,INERTIA,PROXY,OUTPUT)
     . BIND(C,NAME="nodal_rigid_native_kinetic_correction")
      USE ISO_C_BINDING, ONLY:C_DOUBLE,C_INT
      IMPLICIT NONE
      INTEGER(C_INT),INTENT(IN)::COUNT
      REAL(C_DOUBLE),INTENT(IN)::FRAME(9),PRINCIPAL(3),GROUP_VR(3),
     . MEMBER_V(3,COUNT),MEMBER_VR(3,COUNT),MASS(COUNT),
     . INERTIA(COUNT),PROXY
      REAL(C_DOUBLE),INTENT(OUT)::OUTPUT(2)
      REAL(C_DOUBLE)::RBY(12),V(3,COUNT+1),VR(3,COUNT+1),
     . A(3,COUNT+1),AR(3,COUNT+1),MS(COUNT+1),IN(COUNT+1),
     . VG(3),VI(3),DT05,MAS,WEWE2,ENROTT,ENCINT,XMASST,
     . XMOMTT,YMOMTT,ZMOMTT,ENCIN2T,ENROT2T,ENROT_T,ENCIN_T
      REAL(C_DOUBLE),PARAMETER::HALF=.5D0,ZERO=0.D0,ONE=1.D0,
     . DT1=0.D0,DY_G=1.D0
      INTEGER::NOD(COUNT),WEIGHT_MD(COUNT+1),WEIGHT(COUNT+1),M,N,I
      INTEGER,PARAMETER::IDYNA=0
      M=1
      A=ZERO
      AR=ZERO
      V=ZERO
      VR=ZERO
      MS=ZERO
      IN=ZERO
      WEIGHT_MD=1
      WEIGHT=1
      RBY(1:9)=FRAME
      RBY(10:12)=PRINCIPAL
      VR(:,1)=GROUP_VR
      IN(1)=PROXY
      DO I=1,COUNT
        NOD(I)=I+1
        V(:,I+1)=MEMBER_V(:,I)
        VR(:,I+1)=MEMBER_VR(:,I)
        MS(I+1)=MASS(I)
        IN(I+1)=INERTIA(I)
      ENDDO
      XMOMTT=ZERO
      YMOMTT=ZERO
      ZMOMTT=ZERO
      ENCIN2T=ZERO
      ENROT2T=ZERO
#include "extracted/KineticPrimaryCorrection.inc"
      DO I=1,COUNT
#include "extracted/KineticMemberCorrection.inc"
      ENDDO
      ENROT_T=ZERO
      ENCIN_T=ZERO
#include "extracted/KineticCorrectionPublish.inc"
      OUTPUT(1)=ENCIN_T
      OUTPUT(2)=ENROT_T
      END
