! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete selected current CBACOOR including actual velocity correction,
! local spin, native constant shear and point derivatives. Independent inputs.
      SUBROUTINE QBF_KINEMATICS(X,V,VR,DT1,VQ,VCORE,JAC,
     . BMS,VXYZ,RXYZ,VDEFS,AREA,LL,FACN,OUTPUT)
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,4)
      REAL(C_DOUBLE), INTENT(OUT) :: OUTPUT(84)
      REAL(C_DOUBLE), INTENT(IN) :: V(3,4),VR(3,4),DT1
      MY_REAL RXYZ(MVSIZ,2,4),FACN(MVSIZ,2),BMS(MVSIZ,8,4),
     . VDEFS(MVSIZ,8,4),VL1(MVSIZ,3),VL2(MVSIZ,3),
     . VL3(MVSIZ,3),VL4(MVSIZ,3),V13(MVSIZ,3),
     . V24(MVSIZ,3),VHI(MVSIZ,3),VG13(3),VG24(3),VGHI(3),
     . RR(3,4),DT05,DT025,EXZ,EYZ,DDRY,DDRX,DDRZ1,DDRZ2,
     . V13X,V24X,VHIX,S1,FAC1,FAC2,FACI,VSAVE(MVSIZ,3,4)
      INTEGER NN(4),IMPL_S,IMP_LR
      MY_REAL RX(MVSIZ),RY(MVSIZ),RZ(MVSIZ),SX(MVSIZ),SY(MVSIZ),SSZ(MVSIZ),
     . R11(MVSIZ),R12(MVSIZ),R13(MVSIZ),R21(MVSIZ),R22(MVSIZ),R23(MVSIZ),
     . R31(MVSIZ),R32(MVSIZ),R33(MVSIZ),DETA1(MVSIZ),OFFG(MVSIZ),
     . AREA(MVSIZ),AREA_I(MVSIZ),VQ(MVSIZ,3,3),VCORE(MVSIZ,3,4),
     . XL2(MVSIZ),YL2(MVSIZ),XL3(MVSIZ),YL3(MVSIZ),XL4(MVSIZ),YL4(MVSIZ),
     . ZL1(MVSIZ),X13_T(MVSIZ),X24_T(MVSIZ),Y13_T(MVSIZ),Y24_T(MVSIZ),
     . L13(MVSIZ),L24(MVSIZ),LL(MVSIZ),LM(MVSIZ),
     . JAC(MVSIZ,4),HX(MVSIZ,4),HY(MVSIZ,4),VXYZ(MVSIZ,3,4),
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
      IMPL_S=0
      IMP_LR=0
      RXYZ=ZERO
      VDEFS=ZERO
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
#include "extracted/VelocityCorrection.inc"
#include "extracted/SurfaceBranches.inc"
#include "extracted/FlatPacket.inc"
      CALL QBAT_CBADEFSH(JFT,JLT,X13_T,X24_T,Y13_T,Y24_T,
     .                  BM,VDEF,VXYZ,NPLAT,IPLAT)
      DO J=1,4
        OUTPUT(25+(J-1)*3:27+(J-1)*3)=VCORE(1,:,J)
      ENDDO
      DO NG=1,4
        CALL QBAT_CBADEF1(JFT,JLT,NG,VCORE,VXYZ,VDEF,
     .                   HX,HY,BM,NPLAT,IPLAT,0)
        BMS(:,:,NG)=BM(:,1:8)
        VDEFS(:,:,NG)=VDEF
        OFFSET=37+(NG-1)*11
        OUTPUT(OFFSET:OFFSET+2)=[JAC(1,NG),HX(1,NG),HY(1,NG)]
        OUTPUT(OFFSET+3:OFFSET+10)=BM(1,1:8)
      ENDDO
! Independently query CBADEFSH with four velocity basis vectors to expose its
! geometric constant-shear coefficients; this is not a physical velocity step.
      VSAVE=VXYZ
      DO Q=1,4
        VXYZ=ZERO
        SELECT CASE(Q)
        CASE(1)
          VXYZ(1,2,1)=ONE
        CASE(2)
          VXYZ(1,2,2)=ONE
        CASE(3)
          VXYZ(1,1,1)=ONE
        CASE(4)
          VXYZ(1,1,2)=ONE
        END SELECT
        CALL QBAT_CBADEFSH(JFT,JLT,X13_T,X24_T,Y13_T,Y24_T,
     .                    BM,VDEF,VXYZ,NPLAT,IPLAT)
        OUTPUT(80+Q)=VDEF(1,3)
      ENDDO
      VXYZ=VSAVE
#include "extracted/CurrentLength.inc"
      END
