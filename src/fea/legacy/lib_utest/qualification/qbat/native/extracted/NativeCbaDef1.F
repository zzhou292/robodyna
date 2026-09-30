      SUBROUTINE CBADEF1(JFT,JLT,NG,VCORE,VXYZ,VDEF,
     1                  HX,HY,BM,NPLAT,IPLAT,ISROT)
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
C        CALCULATION OF GENERALIZED DEFORMATIONS AT GAUSS POINTS AND [B] (MEMBRANE ONLY)
C        ENTREES :  NEL,NG,VCORE,VXYZ
C        SORTIES : VDEF,BM (MEMBRANE)
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
C-----------------------------------------------
C   I M P L I C I T   T Y P E S
C-----------------------------------------------
#include      "implicit_f.inc"
C-----------------------------------------------
C   G l o b a l   P a r a m e t e r s
C-----------------------------------------------
#include      "mvsiz_p.inc"
C-----------------------------------------------
C   D U M M Y   A R G U M E N T S
C-----------------------------------------------
      INTEGER NG,JFT,JLT,NPLAT,IPLAT(*),ISROT
      MY_REAL 
     .   VCORE(MVSIZ,12),VXYZ(MVSIZ,12),BM(MVSIZ,36),HX(MVSIZ,4),HY(MVSIZ,4)
      MY_REAL 
     .   VDEF(MVSIZ,8)
C-----------------------------------------------
C   L O C A L   V A R I A B L E S
C-----------------------------------------------
      INTEGER I, EP
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
#include "vectorize.inc"
       DO I=JFT,NPLAT 
        EP=IPLAT(I)
C-------VCORE(12):BX0(2),BY0(2),GAMA(2),MX23,MY23,MX34,MY34,MX13,MY13
C-------V13(I)->VXYZ(j,1,I),V24(I)->VXYZ(j,2,I),VHI(I)->VXYZ(j,3,I)------------
        BM(EP,1)=VCORE(EP,1)+HX(EP,NG)*VCORE(EP,5)
        BM(EP,2)=VCORE(EP,2)+HX(EP,NG)*VCORE(EP,6)
        BM(EP,3)=HX(EP,NG)*FOURTH
        BM(EP,4)=-BM(EP,3)
        BM(EP,5)=VCORE(EP,3)+HY(EP,NG)*VCORE(EP,5)
        BM(EP,6)=VCORE(EP,4)+HY(EP,NG)*VCORE(EP,6)
        BM(EP,7)=HY(EP,NG)*FOURTH
        BM(EP,8)=-BM(EP,7)
C
        VDEF(EP,1)=BM(EP,1)*VXYZ(EP,1)+BM(EP,2)*VXYZ(EP,4)
     1          +BM(EP,3)*VXYZ(EP,7)
        VDEF(EP,2)=BM(EP,5)*VXYZ(EP,2)+BM(EP,6)*VXYZ(EP,5)
     1          +BM(EP,7)*VXYZ(EP,8)
        VDEF(EP,4)=ZERO
        VDEF(EP,5)=ZERO
        VDEF(EP,6)=ZERO
        VDEF(EP,7)=ZERO
        VDEF(EP,8)=ZERO
       ENDDO
       IF (ISROT>0) THEN
#include "vectorize.inc"
        DO I=JFT,NPLAT
         EP=IPLAT(I)
C----------no more constant shear-----------
         VDEF(EP,3)=BM(EP,1)*VXYZ(EP,2)+BM(EP,2)*VXYZ(EP,5)
     1             +BM(EP,3)*VXYZ(EP,8)
     1             +BM(EP,5)*VXYZ(EP,1)+BM(EP,6)*VXYZ(EP,4)
     1             +BM(EP,7)*VXYZ(EP,7)
        END DO
       END IF !(ISROT>0) THEN
C       
       RETURN
       END
