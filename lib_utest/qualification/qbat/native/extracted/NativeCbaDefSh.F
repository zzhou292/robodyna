        SUBROUTINE CBADEFSH(JFT  ,JLT  ,X13  ,X24  ,Y13  ,Y24  ,
     1                                 BM, VDEF, VXYZ , NPLAT, IPLAT)
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
C        CALCUL 'membrane shear traitement' use only PART CONSTANTE
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
       INTEGER JFT ,JLT,NPLAT,IPLAT(*)
       my_real 
     .     X13(*)   ,X24(*)    ,Y13(*)     ,Y24(*)    ,
     .     BM(MVSIZ,36),VDEF(MVSIZ,8) ,VXYZ(MVSIZ,3,4)  
C-----------------------------------------------
C   L O C A L   V A R I A B L E S
C-----------------------------------------------
        INTEGER I, EP
C
       DO EP=JFT,NPLAT 
        I=IPLAT(EP)
          VDEF(I,3)=Y24(I)*VXYZ(I,2,1)-Y13(I)*VXYZ(I,2,2)
     .             -X24(I)*VXYZ(I,1,1)+X13(I)*VXYZ(I,1,2)
       ENDDO
       DO EP=NPLAT+1,JLT 
        I=IPLAT(EP)
          BM(I,3)=-X24(I)
          BM(I,6)=Y24(I)
          BM(I,12)=X13(I)
          BM(I,15)=-Y13(I)
          BM(I,21)=-BM(I,3)
          BM(I,24)=-BM(I,6)
          BM(I,30)=-BM(I,12)
          BM(I,33)=-BM(I,15)
          VDEF(I,3)=Y24(I)*(VXYZ(I,2,1)-VXYZ(I,2,3))+
     1              Y13(I)*(-VXYZ(I,2,2)+VXYZ(I,2,4))
     2             -X24(I)*(VXYZ(I,1,1)-VXYZ(I,1,3))
     3             +X13(I)*(VXYZ(I,1,2)-VXYZ(I,1,4))
       ENDDO
       RETURN
       END
