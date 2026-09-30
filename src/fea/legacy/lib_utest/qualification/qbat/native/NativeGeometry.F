! SPDX-License-Identifier: AGPL-3.0-or-later
! One active NPTT1 geometry packet. Exact CBACOOR/CBADEF source fragments;
! no velocity correction, material driver or engine history is substituted.
      SUBROUTINE QBAT_GEOMETRY(X,OUTPUT,FLAT)
     . BIND(C,NAME="qbat_native_geometry")
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,4)
      REAL(C_DOUBLE), INTENT(OUT) :: OUTPUT(84)
      INTEGER(C_INT), INTENT(OUT) :: FLAT
      MY_REAL RX(MVSIZ),RY(MVSIZ),RZ(MVSIZ),SX(MVSIZ),SY(MVSIZ),SSZ(MVSIZ),
     . R11(MVSIZ),R12(MVSIZ),R13(MVSIZ),R21(MVSIZ),R22(MVSIZ),R23(MVSIZ),
     . R31(MVSIZ),R32(MVSIZ),R33(MVSIZ),DETA1(MVSIZ),OFFG(MVSIZ),
     . AREA(MVSIZ),AREA_I(MVSIZ),VQ(MVSIZ,3,3),VCORE(MVSIZ,3,4),
     . XL2(MVSIZ),YL2(MVSIZ),XL3(MVSIZ),YL3(MVSIZ),XL4(MVSIZ),YL4(MVSIZ),
     . ZL1(MVSIZ),X13_T(MVSIZ),X24_T(MVSIZ),Y13_T(MVSIZ),Y24_T(MVSIZ),
     . L13(MVSIZ),L24(MVSIZ),LL(MVSIZ),LM(MVSIZ),
     . JAC(MVSIZ,4),HX(MVSIZ,4),HY(MVSIZ,4),VXYZ(MVSIZ,12),
     . BM(MVSIZ,36),VDEF(MVSIZ,8),
     . LXYZ0(3),XX1,YY1,ZZ1,XX2,YY2,ZZ2,XX3,YY3,ZZ3,XX4,YY4,ZZ4,
     . C1,C2,Z2,TOL,X13,X24,Y13,Y24,MX13,MY13,MX23,MX34,MY23,MY34,
     . GAMA1,GAMA2,J0,J1,J2,PG
      INTEGER I,J,K,L,M,EP,JFT,JLT,IXC(5,1),NPT,NPINCH,
     . NPLAT,SPLAT,IPLAT(MVSIZ),JPLAT(MVSIZ),NG,Q,OFFSET
! The original DATA initialization is deliberately preserved as REAL32 literal.
      DATA PG/.577350269189626/
      JFT=1
      JLT=1
      IXC(:,1)=[1,1,2,3,4]
      OFFG=ONE
      VCORE=ZERO
      VXYZ=ZERO
      BM=ZERO
      VDEF=ZERO
      NPT=1
      NPINCH=0
      NPLAT=0
      TOL=EM12
#define CLSKEW3 QE_Q1_ENGINE_CLSKEW3
#include "extracted/CurrentFrame.inc"
#undef CLSKEW3
#include "extracted/CenteredGeometry.inc"
      DO I=1,3
        DO J=1,3
          OUTPUT((I-1)*3+J)=VQ(1,I,J)
        ENDDO
      ENDDO
      OUTPUT(10:12)=[AREA(1),AREA_I(1),ZL1(1)]
      DO J=1,4
        OUTPUT(13+(J-1)*3:15+(J-1)*3)=VCORE(1,:,J)
      ENDDO
#include "extracted/SurfaceBranches.inc"
#include "extracted/FlatPointPrelude.inc"
#include "extracted/FlatPointGeometry.inc"
      FLAT=NPLAT
      DO J=1,4
        OUTPUT(25+(J-1)*3:27+(J-1)*3)=VCORE(1,:,J)
      ENDDO
      DO NG=1,4
        CALL QBAT_CBADEF1(JFT,JLT,NG,VCORE,VXYZ,VDEF,
     .                   HX,HY,BM,NPLAT,IPLAT,0)
        OFFSET=37+(NG-1)*11
        OUTPUT(OFFSET:OFFSET+2)=[JAC(1,NG),HX(1,NG),HY(1,NG)]
        OUTPUT(OFFSET+3:OFFSET+10)=BM(1,1:8)
      ENDDO
! Independently query CBADEFSH with four velocity basis vectors to expose its
! geometric constant-shear coefficients; this is not a physical velocity step.
      DO Q=1,4
        VXYZ=ZERO
        SELECT CASE(Q)
        CASE(1)
          VXYZ(1,2)=ONE
        CASE(2)
          VXYZ(1,5)=ONE
        CASE(3)
          VXYZ(1,1)=ONE
        CASE(4)
          VXYZ(1,4)=ONE
        END SELECT
        CALL QBAT_CBADEFSH(JFT,JLT,X13_T,X24_T,Y13_T,Y24_T,
     .                    BM,VDEF,VXYZ,NPLAT,IPLAT)
        OUTPUT(80+Q)=VDEF(1,3)
      ENDDO
      END
