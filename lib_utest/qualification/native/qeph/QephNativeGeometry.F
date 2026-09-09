! Shared fixed QEPH geometry/rate context; no owner or accepted history.
! Adapter to pinned OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later.
! Exact donor routines and original notices are retained separately.
      MODULE QE_Q1_GEOMETRY_MOD
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      PRIVATE
      PUBLIC :: QE_GEOMETRY_,QE_PREPARE_GEOMETRY,QE_PACK_GEOMETRY
      TYPE QE_GEOMETRY_
        LOGICAL PLAT(MVSIZ)
        MY_REAL AREA(MVSIZ),AREA_I(MVSIZ),LL(MVSIZ),
     .  L13(MVSIZ),L24(MVSIZ),X13(MVSIZ),X24(MVSIZ),
     .  Y13(MVSIZ),Y24(MVSIZ),MX13(MVSIZ),MX23(MVSIZ),MX34(MVSIZ),
     .  MY13(MVSIZ),MY23(MVSIZ),MY34(MVSIZ),Z1(MVSIZ),Z2(MVSIZ),
     .  PY1(MVSIZ),PX2(MVSIZ),PY2(MVSIZ),OFFG(MVSIZ),OFF(MVSIZ),
     .  THK(MVSIZ),V13(MVSIZ,3),V24(MVSIZ,3),VHI(MVSIZ,3),
     .  RLXYZ(MVSIZ,2,4),VQN(MVSIZ,3,4),VQ(MVSIZ,3,3),
     .  COREL(MVSIZ,2,4),DI(MVSIZ,6),DB(MVSIZ,3,4),FACN(MVSIZ,2),
     .  RLZ(MVSIZ,4),DIZ(MVSIZ,3),VDEF(MVSIZ,8),VHG(MVSIZ,6)
      END TYPE
      CONTAINS

! Same explicit context, arithmetic, aliases and source call sequence as Q1.
! Q2 consumes these actual intermediate arrays rather than rebuilding them
! from public diagnostics or running a second frame/projection calculation.
      SUBROUTINE QE_PREPARE_GEOMETRY(X,V,VR,THICKNESS,STEP,G,STATUS)
      USE ELBUFDEF_MOD, ONLY: ELBUF_STRUCT_
      USE ELEMENT_MOD, ONLY: NIXC
#include "param_c.inc"
#include "parit_c.inc"
#include "scr05_c.inc"
#include "scr17_c.inc"
#include "com08_c.inc"
#include "impl1_c.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,4),V(3,4),VR(3,4),THICKNESS,STEP
      TYPE(QE_GEOMETRY_), INTENT(OUT) :: G
      INTEGER(C_INT), INTENT(OUT) :: STATUS
      TYPE(ELBUF_STRUCT_) CONTEXT
      INTEGER IXC(NIXC,1)
      MY_REAL R(MVSIZ,3,3),VX(MVSIZ,3,4),VRX(MVSIZ,3,4),XG(MVSIZ,3,4),
     . U(MVSIZ,2,4),XL(MVSIZ,2,4),VL(MVSIZ,3,4),
     . DIR_A(MVSIZ,1),DIR_B(MVSIZ,1),PM(1,1)
      DOUBLE PRECISION SMSTR(6)
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
      G%AREA=ZERO
      G%AREA_I=ZERO
      G%LL=ZERO
      G%L13=ZERO
      G%L24=ZERO
      G%X13=ZERO
      G%X24=ZERO
      G%Y13=ZERO
      G%Y24=ZERO
      G%MX13=ZERO
      G%MX23=ZERO
      G%MX34=ZERO
      G%MY13=ZERO
      G%MY23=ZERO
      G%MY34=ZERO
      G%Z1=ZERO
      G%Z2=ZERO
      G%PY1=ZERO
      G%PX2=ZERO
      G%PY2=ZERO
      G%OFFG=ONE
      G%OFF=ZERO
      G%THK=THICKNESS
      G%V13=ZERO
      G%V24=ZERO
      G%VHI=ZERO
      G%RLXYZ=ZERO
      G%VQN=ZERO
      G%VQ=ZERO
      G%COREL=ZERO
      G%DI=ZERO
      G%DB=ZERO
      G%FACN=ZERO
      R=ZERO
      G%RLZ=ZERO
      G%DIZ=ZERO
      VX=ZERO
      VRX=ZERO
      XG=ZERO
      U=ZERO
      XL=ZERO
      VL=ZERO
      G%VDEF=ZERO
      G%VHG=ZERO
      PM=ZERO
      SMSTR=0D0
      G%PLAT=.FALSE.
! Native CORTDIR3 is a real no-op for IREP=0, IDRAPE=0, not a stub.
      DIR_A=17.25D0
      DIR_B=-9.5D0
      CALL CZCORC1(4,1,CONTEXT,1,1,X,V,VR,IXC,PM,G%PLAT,G%AREA,
     . G%AREA_I,G%V13,G%V24,G%VHI,G%RLXYZ,G%VQN,G%VQ,G%LL,G%L13,G%L24,
     . G%X13,G%X24,G%Y13,G%Y24,G%MX13,G%MX23,G%MX34,G%MY13,G%MY23,G%MY34,
     . G%Z1,G%COREL,G%DI,G%DB,SMSTR,0,0,1,-1,DIR_A,DIR_B,G%OFFG,
     . G%RLXYZ,G%COREL,G%FACN,G%PY1,G%PX2,G%PY2,
     . R(1,1,1),R(1,1,2),R(1,1,3),R(1,2,1),R(1,2,2),
     . R(1,2,3),R(1,3,1),R(1,3,2),R(1,3,3),G%RLZ,0,0,
     . VX(1,1,1),VX(1,1,2),VX(1,1,3),VX(1,1,4),
     . VX(1,2,1),VX(1,2,2),VX(1,2,3),VX(1,2,4),
     . VX(1,3,1),VX(1,3,2),VX(1,3,3),VX(1,3,4),
     . VRX(1,1,1),VRX(1,1,2),VRX(1,1,3),VRX(1,1,4),
     . VRX(1,2,1),VRX(1,2,2),VRX(1,2,3),VRX(1,2,4),
     . VRX(1,3,1),VRX(1,3,2),VRX(1,3,3),VRX(1,3,4),
     . XG(1,1,1),XG(1,1,2),XG(1,1,3),XG(1,1,4),
     . XG(1,2,1),XG(1,2,2),XG(1,2,3),XG(1,2,4),
     . XG(1,3,1),XG(1,3,2),XG(1,3,3),XG(1,3,4),G%THK,G%DIZ,
     . U(1,1,1),U(1,1,2),U(1,1,3),U(1,1,4),
     . U(1,2,1),U(1,2,2),U(1,2,3),U(1,2,4),
     . XL(1,1,2),XL(1,1,3),XL(1,1,4),
     . XL(1,2,2),XL(1,2,3),XL(1,2,4),
     . VL(1,1,1),VL(1,1,2),VL(1,1,3),VL(1,1,4),1,G%Z2)
      STATUS=1
      IF(IDEL7NOK/=0.OR.G%OFFG(1)/=ONE) RETURN
      IF(DIR_A(1,1)/=17.25D0.OR.DIR_B(1,1)/=-9.5D0) RETURN
! Preserve the source alias: G%RLXYZ is passed both as physical/current and as
! the unused IVECTOR=1 view, exactly as in the selected CZFORC3 call chain.
      CALL CZDEF(1,1,G%AREA,G%AREA_I,G%V13,G%V24,G%VHI,G%RLXYZ,G%VDEF,G%VHG,
     . G%X13,G%X24,G%Y13,G%Y24,G%MX13,G%MX23,G%MX34,G%MY13,G%MY23,G%MY34,
     . G%Z1,STEP,G%OFF,G%OFFG,G%RLXYZ)
      IF(G%OFF(1)/=ONE) RETURN
      STATUS=0
      END SUBROUTINE

      SUBROUTINE QE_PACK_GEOMETRY(X,G,OUTPUT,IS_PLANAR)
      REAL(C_DOUBLE), INTENT(IN) :: X(3,4)
      TYPE(QE_GEOMETRY_), INTENT(IN) :: G
      REAL(C_DOUBLE), INTENT(OUT) :: OUTPUT(80)
      INTEGER(C_INT), INTENT(OUT) :: IS_PLANAR
      INTEGER I,J,K
      MY_REAL CENTROID(3),DELTA(3),RAW_WARP
      DO I=1,3
        DO J=1,3
          OUTPUT(3*(I-1)+J)=G%VQ(1,I,J)
        END DO
      END DO
! CZCORP5's G%Z2 is local; CZCORC1's final G%Z2 dummy is unused on this path.
! Reproduce its pre-projection raw G%Z1 diagnostic with the returned unchanged
! frame and original X, exact centroid/dot order from original czcorc.F:199-218.
! This does not feed back into the selected native geometry/rates.
      DO I=1,3
        CENTROID(I)=FOURTH*(X(I,3)+X(I,4)+X(I,1)+X(I,2))
        DELTA(I)=X(I,1)-CENTROID(I)
      END DO
      RAW_WARP=G%VQ(1,1,3)*DELTA(1)+G%VQ(1,2,3)*DELTA(2)+G%VQ(1,3,3)*DELTA(3)
      OUTPUT(10:16)=[G%AREA(1),G%AREA_I(1),G%LL(1),G%FACN(1,1),
     . G%FACN(1,2),ABS(RAW_WARP),G%Z1(1)]
      IS_PLANAR=0
      IF(G%PLAT(1)) IS_PLANAR=1
      DO J=1,4
        K=17+3*(J-1)
        OUTPUT(K:K+2)=[G%COREL(1,1,J),G%COREL(1,2,J),
     . G%Z1(1)*(-ONE)**(J-1)]
        K=29+3*(J-1)
        OUTPUT(K:K+2)=G%VQN(1,:,J)
        IF(G%PLAT(1)) OUTPUT(K:K+2)=[ZERO,ZERO,ONE]
        K=47+3*(J-1)
        OUTPUT(K:K+2)=G%DB(1,:,J)
        K=59+2*(J-1)
        OUTPUT(K:K+1)=G%RLXYZ(1,:,J)
      END DO
      OUTPUT(41:46)=G%DI(1,:)
      OUTPUT(67:74)=G%VDEF(1,:)
      OUTPUT(75:80)=G%VHG(1,:)
      END SUBROUTINE
      END MODULE
