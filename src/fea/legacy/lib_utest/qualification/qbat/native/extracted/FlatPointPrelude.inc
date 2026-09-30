      DO I=JFT,NPLAT
        EP =IPLAT(I)
        X13 =X13_T(EP)
        X24 =X24_T(EP)
        Y13 =Y13_T(EP)
        Y24 =Y24_T(EP)
C
        MX13=(VCORE(EP,1,1)+VCORE(EP,1,3))*HALF
        MY13=(VCORE(EP,2,1)+VCORE(EP,2,3))*HALF
        MX23=(VCORE(EP,1,2)+VCORE(EP,1,3))*HALF
        MX34=(VCORE(EP,1,3)+VCORE(EP,1,4))*HALF
        MY23=(VCORE(EP,2,2)+VCORE(EP,2,3))*HALF
        MY34=(VCORE(EP,2,3)+VCORE(EP,2,4))*HALF
        X13_T(EP) =X13*AREA_I(EP)
        X24_T(EP) =X24*AREA_I(EP)
        Y13_T(EP) =Y13*AREA_I(EP)
        Y24_T(EP) =Y24*AREA_I(EP)
C--------GAMA(2)
        GAMA1=-MX13*Y24+MY13*X24
        GAMA2= MX13*Y13-MY13*X13
        Z2=ZL1(EP)*ZL1(EP)
