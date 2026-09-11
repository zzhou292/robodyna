! SPDX-License-Identifier: AGPL-3.0-or-later
! Private calls to complete pinned T3 leaves; actual ISMSTR2/NPTT1/OFF.
      MODULE ONE_POINT_GEOMETRY
      USE ISO_C_BINDING,ONLY:C_DOUBLE,C_INT
      USE T3_ENGINE_GEOMETRY_MOD,ONLY:T3_GEOMETRY_
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      CONTAINS
      SUBROUTINE ONE_POINT_FRAME(X,V,VR,STEP,OLD_OFF,G,STATUS)
      USE ELBUFDEF_MOD, ONLY: ELBUF_STRUCT_
      USE ELEMENT_MOD, ONLY: NIXTG
      USE, INTRINSIC :: IEEE_ARITHMETIC, ONLY: IEEE_IS_FINITE
#include "com08_c.inc"
#include "scr05_c.inc"
#include "impl1_c.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,3),V(3,3),VR(3,3),STEP,OLD_OFF
      TYPE(T3_GEOMETRY_), INTENT(OUT) :: G
      INTEGER(C_INT), INTENT(OUT) :: STATUS
      TYPE(ELBUF_STRUCT_) CONTEXT
      INTEGER IXTG(NIXTG,1)
      MY_REAL DIR_A(MVSIZ,1),DIR_B(MVSIZ,1),RAW_Y3
      DOUBLE PRECISION SMSTR(3*MVSIZ)
      G=T3_GEOMETRY_()
      G%OFFG=OLD_OFF
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
      CONTEXT%NPTT=1
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
     . SMSTR,G%OFFG,2,G%ALPE,G%LENGTH,
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

      SUBROUTINE ONE_POINT_RATES(G,STATUS)
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
     . IXTG,G%WXY,2,G%DEF(1,6),G%DEF(1,7),G%DEF(1,8),
     . G%D(1,1),G%D(1,2),G%D(1,3),G%DEF(1,4),G%DEF(1,5),
     . G%F(1,1,1),G%F(1,2,1),G%F(1,3,1),
     . G%F(1,1,2),G%F(1,2,2),G%F(1,3,2),
     . G%F(1,1,3),G%F(1,2,3),G%F(1,3,3))
      IF(G%OFFG(1)/=G%OFF(1)) RETURN
      STATUS=0
      END SUBROUTINE

      END MODULE
