! Binding to complete pinned OpenRadioss geometry/rate routines.
! Original arithmetic: (C) 2026 Siemens, AGPL-3.0-or-later; see original/LICENSE.md.
! This adapter owns only fixed scratch, context selection and packet packing.
      SUBROUTINE QE_Q1_KINEMATICS(X,V,VR,THICKNESS,STEP,
     . OUTPUT,IS_PLANAR,STATUS) BIND(C,NAME="qeph_q1_kinematics")
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
      USE ELBUFDEF_MOD, ONLY: ELBUF_STRUCT_
      USE ELEMENT_MOD, ONLY: NIXC
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "param_c.inc"
#include "parit_c.inc"
#include "scr05_c.inc"
#include "scr17_c.inc"
#include "com08_c.inc"
#include "impl1_c.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,4),V(3,4),VR(3,4),THICKNESS,STEP
      REAL(C_DOUBLE), INTENT(OUT) :: OUTPUT(80)
      INTEGER(C_INT), INTENT(OUT) :: IS_PLANAR,STATUS
      TYPE(ELBUF_STRUCT_) CONTEXT
      INTEGER IXC(NIXC,1),I,J,K
      LOGICAL PLAT(MVSIZ)
      MY_REAL AREA(MVSIZ),AREA_I(MVSIZ),LL(MVSIZ),
     . L13(MVSIZ),L24(MVSIZ),X13(MVSIZ),X24(MVSIZ),
     . Y13(MVSIZ),Y24(MVSIZ),MX13(MVSIZ),MX23(MVSIZ),MX34(MVSIZ),
     . MY13(MVSIZ),MY23(MVSIZ),MY34(MVSIZ),Z1(MVSIZ),Z2(MVSIZ),
     . PY1(MVSIZ),PX2(MVSIZ),PY2(MVSIZ),OFFG(MVSIZ),OFF(MVSIZ),
     . THK(MVSIZ),V13(MVSIZ,3),V24(MVSIZ,3),VHI(MVSIZ,3),
     . RLXYZ(MVSIZ,2,4),VQN(MVSIZ,3,4),VQ(MVSIZ,3,3),
     . COREL(MVSIZ,2,4),DI(MVSIZ,6),DB(MVSIZ,3,4),FACN(MVSIZ,2),
     . R(MVSIZ,3,3),RLZ(MVSIZ,4),DIZ(MVSIZ,3),
     . VX(MVSIZ,3,4),VRX(MVSIZ,3,4),XG(MVSIZ,3,4),
     . U(MVSIZ,2,4),XL(MVSIZ,2,4),VL(MVSIZ,3,4),
     . VDEF(MVSIZ,8),VHG(MVSIZ,6),DIR_A(MVSIZ,1),DIR_B(MVSIZ,1),PM(1,1),
     . CENTROID(3),DELTA(3),RAW_WARP
      DOUBLE PRECISION SMSTR(6)
! All context consumed by the selected branch is explicit. COMMON names and
! module symbols are privately prefixed at compile time; C++ serializes calls.
      NPROPM=1
      IVECTOR=0
      IRESP=2
      IMPL_S=0
      IMP_LR=0
      ISMDISP=0
      IKPROJ=0
      IDT1SH=0
      IDEL7NOK=0
      DT1=STEP
      CONTEXT%IDRAPE=0
      CONTEXT%IGTYP=1
      CONTEXT%NEL=MVSIZ
      CONTEXT%NLAY=1
      CONTEXT%NINTLAY=0
      CONTEXT%NPTR=1
      CONTEXT%NPTS=1
      CONTEXT%NPTT=3
      CONTEXT%IXFEM=0
      CONTEXT%NXEL=0
      NULLIFY(CONTEXT%BUFLY,CONTEXT%INTLAY,CONTEXT%XFEM,
     . CONTEXT%NLOC,CONTEXT%NLOCTS)
      IXC=1
      IXC(2:5,1)=[1,2,3,4]
      AREA=ZERO
      AREA_I=ZERO
      LL=ZERO
      L13=ZERO
      L24=ZERO
      X13=ZERO
      X24=ZERO
      Y13=ZERO
      Y24=ZERO
      MX13=ZERO
      MX23=ZERO
      MX34=ZERO
      MY13=ZERO
      MY23=ZERO
      MY34=ZERO
      Z1=ZERO
      Z2=ZERO
      PY1=ZERO
      PX2=ZERO
      PY2=ZERO
      OFFG=ONE
      OFF=ZERO
      THK=THICKNESS
      V13=ZERO
      V24=ZERO
      VHI=ZERO
      RLXYZ=ZERO
      VQN=ZERO
      VQ=ZERO
      COREL=ZERO
      DI=ZERO
      DB=ZERO
      FACN=ZERO
      R=ZERO
      RLZ=ZERO
      DIZ=ZERO
      VX=ZERO
      VRX=ZERO
      XG=ZERO
      U=ZERO
      XL=ZERO
      VL=ZERO
      VDEF=ZERO
      VHG=ZERO
      PM=ZERO
      SMSTR=0D0
      PLAT=.FALSE.
! Native CORTDIR3 is a real no-op for IREP=0, IDRAPE=0, not a stub.
      DIR_A=17.25D0
      DIR_B=-9.5D0
      CALL CZCORC1(4,1,CONTEXT,1,1,X,V,VR,IXC,PM,PLAT,AREA,
     . AREA_I,V13,V24,VHI,RLXYZ,VQN,VQ,LL,L13,L24,
     . X13,X24,Y13,Y24,MX13,MX23,MX34,MY13,MY23,MY34,
     . Z1,COREL,DI,DB,SMSTR,0,0,1,-1,DIR_A,DIR_B,OFFG,
     . RLXYZ,COREL,FACN,PY1,PX2,PY2,
     . R(1,1,1),R(1,1,2),R(1,1,3),R(1,2,1),R(1,2,2),
     . R(1,2,3),R(1,3,1),R(1,3,2),R(1,3,3),RLZ,0,0,
     . VX(1,1,1),VX(1,1,2),VX(1,1,3),VX(1,1,4),
     . VX(1,2,1),VX(1,2,2),VX(1,2,3),VX(1,2,4),
     . VX(1,3,1),VX(1,3,2),VX(1,3,3),VX(1,3,4),
     . VRX(1,1,1),VRX(1,1,2),VRX(1,1,3),VRX(1,1,4),
     . VRX(1,2,1),VRX(1,2,2),VRX(1,2,3),VRX(1,2,4),
     . VRX(1,3,1),VRX(1,3,2),VRX(1,3,3),VRX(1,3,4),
     . XG(1,1,1),XG(1,1,2),XG(1,1,3),XG(1,1,4),
     . XG(1,2,1),XG(1,2,2),XG(1,2,3),XG(1,2,4),
     . XG(1,3,1),XG(1,3,2),XG(1,3,3),XG(1,3,4),THK,DIZ,
     . U(1,1,1),U(1,1,2),U(1,1,3),U(1,1,4),
     . U(1,2,1),U(1,2,2),U(1,2,3),U(1,2,4),
     . XL(1,1,2),XL(1,1,3),XL(1,1,4),
     . XL(1,2,2),XL(1,2,3),XL(1,2,4),
     . VL(1,1,1),VL(1,1,2),VL(1,1,3),VL(1,1,4),1,Z2)
      STATUS=1
      IF(IDEL7NOK/=0.OR.OFFG(1)/=ONE) RETURN
      IF(DIR_A(1,1)/=17.25D0.OR.DIR_B(1,1)/=-9.5D0) RETURN
! Preserve the source alias: RLXYZ is passed both as physical/current and as
! the unused IVECTOR=1 view, exactly as in the selected CZFORC3 call chain.
      CALL CZDEF(1,1,AREA,AREA_I,V13,V24,VHI,RLXYZ,VDEF,VHG,
     . X13,X24,Y13,Y24,MX13,MX23,MX34,MY13,MY23,MY34,
     . Z1,STEP,OFF,OFFG,RLXYZ)
      IF(OFF(1)/=ONE) RETURN
      DO I=1,3
        DO J=1,3
          OUTPUT(3*(I-1)+J)=VQ(1,I,J)
        END DO
      END DO
! CZCORP5's Z2 is local; CZCORC1's final Z2 dummy is unused on this path.
! Reproduce its pre-projection raw Z1 diagnostic with the returned unchanged
! frame and original X, exact centroid/dot order from original czcorc.F:199-218.
! This does not feed back into the selected native geometry/rates.
      DO I=1,3
        CENTROID(I)=FOURTH*(X(I,3)+X(I,4)+X(I,1)+X(I,2))
        DELTA(I)=X(I,1)-CENTROID(I)
      END DO
      RAW_WARP=VQ(1,1,3)*DELTA(1)+VQ(1,2,3)*DELTA(2)+VQ(1,3,3)*DELTA(3)
      OUTPUT(10:16)=[AREA(1),AREA_I(1),LL(1),FACN(1,1),
     . FACN(1,2),ABS(RAW_WARP),Z1(1)]
      IS_PLANAR=0
      IF(PLAT(1)) IS_PLANAR=1
      DO J=1,4
        K=17+3*(J-1)
        OUTPUT(K:K+2)=[COREL(1,1,J),COREL(1,2,J),
     . Z1(1)*(-ONE)**(J-1)]
        K=29+3*(J-1)
        OUTPUT(K:K+2)=VQN(1,:,J)
        IF(PLAT(1)) OUTPUT(K:K+2)=[ZERO,ZERO,ONE]
        K=47+3*(J-1)
        OUTPUT(K:K+2)=DB(1,:,J)
        K=59+2*(J-1)
        OUTPUT(K:K+1)=RLXYZ(1,:,J)
      END DO
      OUTPUT(41:46)=DI(1,:)
      OUTPUT(67:74)=VDEF(1,:)
      OUTPUT(75:80)=VHG(1,:)
      STATUS=0
      END
