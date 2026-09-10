! Exact pinned INIRBY arithmetic with bounded, initialized packet setup.
      SUBROUTINE RIGID_PART_RAW(NPART,NEXTRA,POSITION,MASS,INERTIA,
     . PRIMARY,PRIMARY_MASS,PRIMARY_J,OUTPUT)
     . BIND(C,NAME='rigid_part_native_raw')
      USE ISO_C_BINDING,ONLY:C_INT,C_DOUBLE
      IMPLICIT NONE
      INTEGER(C_INT),VALUE::NPART,NEXTRA
      REAL(C_DOUBLE),INTENT(IN)::POSITION(3,NPART+NEXTRA),
     . MASS(NPART+NEXTRA),INERTIA(NPART+NEXTRA),PRIMARY(3)
      REAL(C_DOUBLE),VALUE::PRIMARY_MASS,PRIMARY_J
      REAL(C_DOUBLE),INTENT(OUT)::OUTPUT(13)
      INTEGER I,J,N,M,NRB,NSL,LSN(NPART),LSN_XTRA(NEXTRA),NPBY(17,1)
      REAL(C_DOUBLE)::X(3,NPART+NEXTRA+1),MS(NPART+NEXTRA+1),
     . MS_LOC(NPART+NEXTRA),IN_LOC(NPART+NEXTRA),RBY(9,1),XG(3),
     . XMG,MASRB,XX,XY,XZ,YY,YZ,ZZ
      M=NPART+NEXTRA+1
      NSL=NPART
      NRB=1
      X(:,1:M-1)=POSITION
      X(:,M)=PRIMARY
      MS(1:M-1)=MASS
      MS(M)=PRIMARY_MASS
      MS_LOC=MASS
      IN_LOC=INERTIA
      XMG=PRIMARY_MASS
      RBY=0.D0
      RBY(1,1)=PRIMARY_J
      RBY(5,1)=PRIMARY_J
      RBY(9,1)=PRIMARY_J
      NPBY=0
      NPBY(15,1)=NEXTRA
      DO I=1,NPART
        LSN(I)=I
      END DO
      DO I=1,NEXTRA
        LSN_XTRA(I)=NPART+I
      END DO
#include "extracted/PartCenter.inc"
#include "extracted/PartCenterDivide.inc"
      IF(NEXTRA>0)THEN
#include "extracted/ExtraCenter.inc"
      END IF
#include "extracted/PrimaryTensor.inc"
#include "extracted/PartTensor.inc"
#include "extracted/ExtraTensor.inc"
      OUTPUT(1)=MASRB
      OUTPUT(2:4)=X(:,M)
      OUTPUT(5:13)=RBY(:,1)
      END SUBROUTINE

      SUBROUTINE RIGID_MERGE_RAW(PARENT,CHILD,OUTPUT)
     . BIND(C,NAME='rigid_part_native_merge')
      USE ISO_C_BINDING,ONLY:C_DOUBLE
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN)::PARENT(13),CHILD(13)
      REAL(C_DOUBLE),INTENT(OUT)::OUTPUT(13)
      INTEGER I,J,M,M_I,NRB
      REAL(C_DOUBLE)::X(3,2),RBY(14,2),XG(3),XMG,MASRB,
     . XX,XY,XZ,YY,YZ,ZZ
      I=1
      M_I=1
      M=2
      NRB=2
      RBY=0.D0
      RBY(1:9,1)=CHILD(5:13)
      RBY(14,1)=CHILD(1)
      RBY(1:9,2)=PARENT(5:13)
      X(:,1)=CHILD(2:4)
      X(:,2)=PARENT(2:4)
      XG=X(:,2)
      XMG=PARENT(1)
      MASRB=PARENT(1)
#include "extracted/MergeCenter.inc"
#include "extracted/MergeParentShift.inc"
#include "extracted/MergeChildTensor.inc"
      OUTPUT(1)=MASRB
      OUTPUT(2:4)=X(:,M)
      OUTPUT(5:13)=RBY(1:9,2)
      END SUBROUTINE

      SUBROUTINE RIGID_POINT_MASS(MASS,INERTIA,ADDED,OUTPUT)
     . BIND(C,NAME='rigid_part_native_point_mass')
      USE ISO_C_BINDING,ONLY:C_DOUBLE
      IMPLICIT NONE
      REAL(C_DOUBLE),VALUE::MASS,INERTIA,ADDED
      REAL(C_DOUBLE),INTENT(OUT)::OUTPUT(3)
      INTEGER NOSYS,J
      REAL(C_DOUBLE)::MS(1),IN(1),AMAS_MULTI(1),TOTADDMAS
      NOSYS=1
      J=1
      MS(1)=MASS
      IN(1)=INERTIA
      AMAS_MULTI(1)=ADDED
      TOTADDMAS=0.D0
#include "extracted/PointMassAdd.inc"
      OUTPUT=[MS(1),IN(1),TOTADDMAS]
      END SUBROUTINE
