C-------ARE STORED IN VCORE---
        VCORE(EP,1,1)=Y24_T(EP)
        VCORE(EP,2,1)=-Y13_T(EP)
        VCORE(EP,3,1)=-X24_T(EP)
        VCORE(EP,1,2)= X13_T(EP)
        VCORE(EP,2,2)=GAMA1*AREA_I(EP)
        VCORE(EP,3,2)=GAMA2*AREA_I(EP)
        VCORE(EP,1,3)=MX23
        VCORE(EP,2,3)=MY23
        VCORE(EP,3,3)=MX34
        VCORE(EP,1,4)=MY34
        VCORE(EP,2,4)=MX13
        VCORE(EP,3,4)=MY13
C
        J1=(MX23*MY13-MX13*MY23)*PG
        J2=-(MX13*MY34-MX34*MY13)*PG
        J0=AREA(EP)*FOURTH
C----   JAC(jacobiens aux points d'integration) peut etre negative------        
        JAC(EP,1)=ABS(J0+J2-J1)
        JAC(EP,2)=ABS(J0+J2+J1)
        JAC(EP,3)=ABS(J0-J2+J1)
        JAC(EP,4)=ABS(J0-J2-J1)
        J1=(MY23-MY34)*PG
        J2=-(MY23+MY34)*PG
        HX(EP,1)=J1/JAC(EP,1)
        HX(EP,2)=J2/JAC(EP,2)
        HX(EP,3)=-J1/JAC(EP,3)
        HX(EP,4)=-J2/JAC(EP,4)
        J1=(MX34-MX23)*PG
        J2=(MX34+MX23)*PG
        HY(EP,1)=J1/JAC(EP,1)
        HY(EP,2)=J2/JAC(EP,2)
        HY(EP,3)=-J1/JAC(EP,3)
        HY(EP,4)=-J2/JAC(EP,4)
C
      ENDDO
