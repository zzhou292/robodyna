! SPDX-License-Identifier: AGPL-3.0-or-later
! Initial current-geometry/coefficient packet only. No material/history call.
      SUBROUTINE QBM_NATIVE_INITIAL(X,THICKNESS,A11,DM,DECLARED_DN,
     . OUTPUT) BIND(C,NAME="qbm_native_initial")
      USE ISO_C_BINDING, ONLY:C_DOUBLE
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      REAL(C_DOUBLE),INTENT(IN) :: X(3,4),THICKNESS,A11,DM,DECLARED_DN
      REAL(C_DOUBLE),INTENT(OUT) :: OUTPUT(4)
      MY_REAL V(3,4),VR(3,4),VQ(MVSIZ,3,3),VCORE(MVSIZ,3,4),
     . JAC(MVSIZ,4),BMS(MVSIZ,8,4),VXYZ(MVSIZ,3,4),RXYZ(MVSIZ,2,4),
     . VDEFS(MVSIZ,8,4),AREA(MVSIZ),LL(MVSIZ),FACN(MVSIZ,2),GEOM(84),
     . AMU(1),DN,GEO(13,1),ALDT(1),ALPE(1),VISCMX(1),
     . VOL0(1),A1(1),OFF(1),F_OSET(1),STI(1),STIR(1),DIVM
      INTEGER JFT,JLT,I,PID(1),IHBE,IMPL_S
      JFT=1
      JLT=1
      V=ZERO
      VR=ZERO
      CALL QBF_KINEMATICS(X,V,VR,ZERO,VQ,VCORE,JAC,BMS,VXYZ,
     . RXYZ,VDEFS,AREA,LL,FACN,GEOM)
      GEO=ZERO
      GEO(13,1)=DECLARED_DN
      PID=1
      IHBE=11
      IMPL_S=0
#include "extracted/CurrentDn.inc"
      ALDT=LL(1)
      ALPE=ONE
      VISCMX=DM
#include "extracted/CurrentViscosity.inc"
      VOL0=THICKNESS*AREA(1)
      A1=A11
      OFF=ONE
      F_OSET=ONE
#include "extracted/CurrentStiffness.inc"
      OUTPUT=[STI(1),STIR(1),FACN(1,1),FACN(1,2)]
      END
