! SPDX-License-Identifier: AGPL-3.0-or-later
! Adapter to pinned OpenRadioss (C) 2026 Siemens, a62b27e6.
! The complete unchanged leaves retain their notices in extracted/.
! Shared private engine work, no material state or accepted physical clock.
      MODULE T3_ENGINE_GEOMETRY_MOD
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      PRIVATE
      PUBLIC :: T3_GEOMETRY_,T3_PREPARE_GEOMETRY,T3_PACK_GEOMETRY
      PUBLIC :: T3_PREPARE_FRAME,T3_PREPARE_RATES
      TYPE T3_GEOMETRY_
        MY_REAL :: F(MVSIZ,3,3)=ZERO,F0(MVSIZ,3,3)=ZERO,
     . XG(MVSIZ,3,3)=ZERO,V(MVSIZ,3,3)=ZERO,VR(MVSIZ,3,3)=ZERO,
     . EDGE(MVSIZ,3,2)=ZERO,LOCAL(MVSIZ,2,2)=ZERO,
     . U(MVSIZ,2,3)=ZERO,D(MVSIZ,3)=ZERO,
     . AREA(MVSIZ)=ZERO,LENGTH(MVSIZ)=ZERO,ALPE(MVSIZ)=ZERO,
     . OFFG(MVSIZ)=ONE,OFF(MVSIZ)=ZERO,DT1C(MVSIZ)=ZERO,
     . SIGY(MVSIZ)=ZERO,DEF(MVSIZ,8)=ZERO,VDIFF(MVSIZ,3)=ZERO,
     . WXY(MVSIZ)=ZERO,ECOS(MVSIZ)=ZERO,ESIN(MVSIZ)=ZERO
      END TYPE
      CONTAINS

      SUBROUTINE T3_PREPARE_GEOMETRY(X,V,VR,STEP,G,STATUS)
      REAL(C_DOUBLE), INTENT(IN) :: X(3,3),V(3,3),VR(3,3),STEP
      TYPE(T3_GEOMETRY_), INTENT(OUT) :: G
      INTEGER(C_INT), INTENT(OUT) :: STATUS
      CALL T3_PREPARE_FRAME(X,V,VR,STEP,G,STATUS)
      IF(STATUS/=0) RETURN
      CALL T3_PREPARE_RATES(G,STATUS)
      END SUBROUTINE

! R3 inserts the complete C3COEF3 between these two private stages, matching
! C3FORC3. R2 still calls both consecutively, without arithmetic changes.
      SUBROUTINE T3_PREPARE_FRAME(X,V,VR,STEP,G,STATUS)
      USE ELBUFDEF_MOD, ONLY: ELBUF_STRUCT_
      USE ELEMENT_MOD, ONLY: NIXTG
      USE, INTRINSIC :: IEEE_ARITHMETIC, ONLY: IEEE_IS_FINITE
#include "com08_c.inc"
#include "scr05_c.inc"
#include "impl1_c.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,3),V(3,3),VR(3,3),STEP
      TYPE(T3_GEOMETRY_), INTENT(OUT) :: G
      INTEGER(C_INT), INTENT(OUT) :: STATUS
      TYPE(ELBUF_STRUCT_) CONTEXT
      INTEGER IXTG(NIXTG,1)
      MY_REAL DIR_A(MVSIZ,1),DIR_B(MVSIZ,1),RAW_Y3
      DOUBLE PRECISION SMSTR(3*MVSIZ)
      G=T3_GEOMETRY_()
      IRESP=2
      DT1=STEP
      IMPL_S=0
      IMP_LR=0
      CONTEXT%IDRAPE=0
      CONTEXT%IGTYP=1
      CONTEXT%NEL=MVSIZ
      CONTEXT%NLAY=1
      CONTEXT%NINTLAY=0
      CONTEXT%NPTR=1
      CONTEXT%NPTS=1
      CONTEXT%NPTT=0
      CONTEXT%IXFEM=0
      CONTEXT%NXEL=0
      NULLIFY(CONTEXT%BUFLY,CONTEXT%INTLAY,CONTEXT%XFEM,
     . CONTEXT%NLOC,CONTEXT%NLOCTS)
      IXTG=1
      IXTG(2:4,1)=[1,2,3]
      SMSTR=0D0
      DIR_A=17.25D0
      DIR_B=-9.5D0
! IRESP2 gathers the supplied endpoint positions and midpoint velocities.
! No quarter-step correction is applied by this gather operation.
      CALL C3COOR3(1,1,X,IXTG,G%OFFG,G%OFF,G%DT1C,V,VR,
     . G%V(1,1,1),G%V(1,1,2),G%V(1,1,3),
     . G%VR(1,1,1),G%VR(1,1,2),G%VR(1,1,3),G%SIGY,
     . G%XG(1,1,1),G%XG(1,1,2),G%XG(1,1,3),
     . G%XG(1,2,1),G%XG(1,2,2),G%XG(1,2,3),
     . G%XG(1,3,1),G%XG(1,3,2),G%XG(1,3,3),X)
! ISH3N2 selects IFRAM_OLD1, passed as ISH3NFRAM=1 to this leaf.
! IREP0/IDRAPE0 leaves material directions unchanged; CLSKEW3 is linked but
! its ISH3NFRAM0 call is not selected. Both actual returned frames are retained.
      CALL C3EVEC3(CONTEXT,DIR_A,DIR_B,1,1,0,
     . G%F0(1,1,1),G%F0(1,2,1),G%F0(1,3,1),
     . G%F0(1,1,2),G%F0(1,2,2),G%F0(1,3,2),
     . G%F0(1,1,3),G%F0(1,2,3),G%F0(1,3,3),
     . G%F(1,1,1),G%F(1,2,1),G%F(1,3,1),
     . G%F(1,1,2),G%F(1,2,2),G%F(1,3,2),
     . G%F(1,1,3),G%F(1,2,3),G%F(1,3,3),
     . 1,G%OFFG,G%ECOS,G%ESIN,1,MVSIZ,G%AREA,
     . G%EDGE(1,1,1),G%EDGE(1,2,1),G%EDGE(1,3,1),
     . G%EDGE(1,1,2),G%EDGE(1,2,2),G%EDGE(1,3,2),
     . G%XG(1,1,1),G%XG(1,1,2),G%XG(1,1,3),
     . G%XG(1,2,1),G%XG(1,2,2),G%XG(1,2,3),
     . G%XG(1,3,1),G%XG(1,3,2),G%XG(1,3,3))
      STATUS=1
      IF(.NOT.IEEE_IS_FINITE(G%AREA(1))) RETURN
      IF(G%AREA(1)<=ZERO) RETURN
! Same dot-product order as the complete C3DERI3 below. This is a guard only:
! the source leaf computes its own value, retains its own floor expression,
! and its actual floor branch must stay inactive in this named domain.
      RAW_Y3=G%F(1,1,2)*G%EDGE(1,1,2)
     .      +G%F(1,2,2)*G%EDGE(1,2,2)
     .      +G%F(1,3,2)*G%EDGE(1,3,2)
      IF(.NOT.IEEE_IS_FINITE(RAW_Y3)) RETURN
      IF(RAW_Y3<=THIRTY2*EM15) RETURN
      IF(DIR_A(1,1)/=17.25D0.OR.DIR_B(1,1)/=-9.5D0) RETURN
      CALL C3DERI3(1,1,G%D(1,1),G%D(1,2),G%D(1,3),
     . SMSTR,G%OFFG,-1,G%ALPE,G%LENGTH,
     . G%U(1,1,1),G%U(1,1,2),G%U(1,1,3),
     . G%U(1,2,1),G%U(1,2,2),G%U(1,2,3),MVSIZ,G%AREA,
     . G%EDGE(1,1,1),G%EDGE(1,2,1),G%EDGE(1,3,1),
     . G%EDGE(1,1,2),G%EDGE(1,2,2),G%EDGE(1,3,2),
     . G%LOCAL(1,1,1),G%LOCAL(1,2,1),G%LOCAL(1,1,2),G%LOCAL(1,2,2),
     . G%F(1,1,1),G%F(1,2,1),G%F(1,3,1),
     . G%F(1,1,2),G%F(1,2,2),G%F(1,3,2),
     . G%F(1,1,3),G%F(1,2,3),G%F(1,3,3))
      IF(G%LOCAL(1,2,2)/=RAW_Y3) RETURN
      STATUS=0
      END SUBROUTINE

      SUBROUTINE T3_PREPARE_RATES(G,STATUS)
      USE ELEMENT_MOD, ONLY: NIXTG
      TYPE(T3_GEOMETRY_), INTENT(INOUT) :: G
      INTEGER(C_INT), INTENT(OUT) :: STATUS
      INTEGER IXTG(NIXTG,1)
      IXTG=1
      IXTG(2:4,1)=[1,2,3]
      STATUS=1
! Keep the native quarter-step and ISH3N2 in-plane corrections. No copied
! equations are substituted for the complete donor leaf.
      CALL C3DEFO3(1,1,G%V(1,1,1),G%V(1,1,2),G%V(1,1,3),
     . IXTG,2,G%D(1,1),G%D(1,2),G%D(1,3),
     . G%DEF(1,1),G%DEF(1,2),G%DEF(1,3),G%DEF(1,4),G%DEF(1,5),
     . G%VDIFF(1,1),G%VDIFF(1,2),G%VDIFF(1,3),
     . G%F(1,1,1),G%F(1,2,1),G%F(1,3,1),
     . G%F(1,1,2),G%F(1,2,2),G%F(1,3,2),
     . G%F(1,1,3),G%F(1,2,3),G%F(1,3,3))
      CALL C3CURV3(1,1,G%VR(1,1,1),G%VR(1,1,2),G%VR(1,1,3),
     . IXTG,G%WXY,-1,G%DEF(1,6),G%DEF(1,7),G%DEF(1,8),
     . G%D(1,1),G%D(1,2),G%D(1,3),G%DEF(1,4),G%DEF(1,5),
     . G%F(1,1,1),G%F(1,2,1),G%F(1,3,1),
     . G%F(1,1,2),G%F(1,2,2),G%F(1,3,2),
     . G%F(1,1,3),G%F(1,2,3),G%F(1,3,3))
      IF(G%OFFG(1)/=ONE.OR.G%OFF(1)/=ONE) RETURN
      STATUS=0
      END SUBROUTINE

      SUBROUTINE T3_PACK_GEOMETRY(G,OUTPUT)
      TYPE(T3_GEOMETRY_), INTENT(IN) :: G
      REAL(C_DOUBLE), INTENT(OUT) :: OUTPUT(38)
      INTEGER I,J
      DO I=1,3
        DO J=1,3
          OUTPUT(3*(I-1)+J)=G%F(1,I,J)
        ENDDO
      ENDDO
      OUTPUT(10:12)=[G%AREA(1),G%LENGTH(1),G%ALPE(1)]
      OUTPUT(13:16)=[G%LOCAL(1,1,1),G%LOCAL(1,2,1),
     . G%LOCAL(1,1,2),G%LOCAL(1,2,2)]
      OUTPUT(17:19)=G%D(1,:)
      OUTPUT(20:27)=G%DEF(1,:)
      OUTPUT(28:30)=G%VDIFF(1,:)
! Only public diagnostics are normalized here. R3 uses G%DEF directly and
! calls C3STRA3 once in the actual native order, never normalized_rate*h.
      OUTPUT(31:38)=G%DEF(1,:)/G%AREA(1)
      END SUBROUTINE
      END MODULE
