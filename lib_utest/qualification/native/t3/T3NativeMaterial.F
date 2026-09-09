! SPDX-License-Identifier: AGPL-3.0-or-later
! OpenRadioss (C) 2026 Siemens, complete C3COEF3 plus selected PM setup.
! Property expressions: borrowed hm_read_mat01.F:116-140 and hm_read_mat.F90:
! 1612-1618, as retained in QEPH source-manifest. Explicit T3 GEO37=0/38=5/6.
! This setup adapter is not a complete starter/material-driver invocation.
      MODULE T3_ENGINE_MATERIAL_MOD
      USE ISO_C_BINDING, ONLY: C_DOUBLE
      USE T3_ENGINE_GEOMETRY_MOD, ONLY: T3_GEOMETRY_
      USE T3_ENGINE_HISTORY_MOD, ONLY: T3_HISTORY_
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      PRIVATE
      PUBLIC :: T3_MATERIAL_,T3_PREPARE_MATERIAL
      TYPE T3_MATERIAL_
        MY_REAL :: PM(200,1)=ZERO,GEO(200,1)=ZERO,
     . THK0(MVSIZ)=ZERO,THK02(MVSIZ)=ZERO,NU(MVSIZ)=ZERO,
     . G(MVSIZ)=ZERO,YM(MVSIZ)=ZERO,A11(MVSIZ)=ZERO,A12(MVSIZ)=ZERO,
     . SSP(MVSIZ)=ZERO,RHO(MVSIZ)=ZERO,VOL0(MVSIZ)=ZERO,
     . VOL00(MVSIZ)=ZERO,GS(MVSIZ)=ZERO,SHF(MVSIZ)=ZERO,
     . STI(MVSIZ)=ZERO,STIR(MVSIZ)=ZERO,A11R(MVSIZ)=ZERO,
     . ZOFFSET(MVSIZ)=ZERO,DM=ZERO
      END TYPE
      CONTAINS
      SUBROUTINE T3_PREPARE_MATERIAL(INPUT,G,H,M)
#include "param_c.inc"
#include "com04_c.inc"
      REAL(C_DOUBLE),INTENT(IN) :: INPUT(4)
      TYPE(T3_GEOMETRY_),INTENT(IN) :: G
      TYPE(T3_HISTORY_),INTENT(IN) :: H
      TYPE(T3_MATERIAL_),INTENT(OUT) :: M
      INTEGER MAT(MVSIZ),PID(MVSIZ),IGEO(100,1)
      MY_REAL PM_STACK(20,1),THK_COPY(MVSIZ)
      NPROPM=200
      NPROPG=200
      NPROPGI=100
      NPSAV=8
      NUMGEO=1
      NUMMAT=1
      M=T3_MATERIAL_()
      MAT=1
      PID=1
      IGEO=0
      IGEO(11,1)=1
      PM_STACK=ZERO
      M%GEO(37,1)=ZERO
      M%GEO(38,1)=FIVE_OVER_6
      M%GEO(199,1)=ZERO
      M%PM(1,1)=INPUT(1)
      M%PM(20,1)=INPUT(2)
      M%PM(21,1)=INPUT(3)
      M%PM(22,1)=INPUT(2)/(TWO*(ONE+INPUT(3)))
      M%PM(24,1)=INPUT(2)/(ONE-INPUT(3)**2)
      M%PM(25,1)=INPUT(3)*M%PM(24,1)
      M%PM(26,1)=FIVE_OVER_6
      M%PM(27,1)=SQRT(INPUT(2)/MAX(INPUT(1),EM20))
      M%DM=ZEP01+FIVEEM3
      M%THK0=INPUT(4)
      THK_COPY=H%THK
      CALL C3COEF3(1,1,M%PM,MAT,M%GEO,PID,G%OFF,G%AREA,
     . M%STI,M%STIR,M%SHF,M%THK0,M%THK02,M%NU,M%G,M%YM,
     . M%A11,M%A12,THK_COPY,M%SSP,M%RHO,M%VOL0,M%GS,
     . 1,0,0,-1,M%VOL00,IGEO,M%A11R,1,PM_STACK,1,M%ZOFFSET)
      END SUBROUTINE
      END MODULE
