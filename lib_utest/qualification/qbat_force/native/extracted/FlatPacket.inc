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
        NN(1)=IXC(2,EP)
        NN(2)=IXC(3,EP)
        NN(3)=IXC(4,EP)
        NN(4)=IXC(5,EP)
C-------TRANSMET 3DDL ROTATIONS ----
        RR(1,1) =VQ(EP,1,1)*VR(1,NN(1))+VQ(EP,2,1)*VR(2,NN(1))+VQ(EP,3,1)*VR(3,NN(1))
        RR(1,2) =VQ(EP,1,1)*VR(1,NN(2))+VQ(EP,2,1)*VR(2,NN(2))+VQ(EP,3,1)*VR(3,NN(2))
        RR(1,3) =VQ(EP,1,1)*VR(1,NN(3))+VQ(EP,2,1)*VR(2,NN(3))+VQ(EP,3,1)*VR(3,NN(3))
        RR(1,4) =VQ(EP,1,1)*VR(1,NN(4))+VQ(EP,2,1)*VR(2,NN(4))+VQ(EP,3,1)*VR(3,NN(4))
        RR(2,1) =VQ(EP,1,2)*VR(1,NN(1))+VQ(EP,2,2)*VR(2,NN(1))+VQ(EP,3,2)*VR(3,NN(1))
        RR(2,2) =VQ(EP,1,2)*VR(1,NN(2))+VQ(EP,2,2)*VR(2,NN(2))+VQ(EP,3,2)*VR(3,NN(2))
        RR(2,3) =VQ(EP,1,2)*VR(1,NN(3))+VQ(EP,2,2)*VR(2,NN(3))+VQ(EP,3,2)*VR(3,NN(3))
        RR(2,4) =VQ(EP,1,2)*VR(1,NN(4))+VQ(EP,2,2)*VR(2,NN(4))+VQ(EP,3,2)*VR(3,NN(4))
C-------V13(I)->VXYZ(I,1),V24(I)->VXYZ(I,2),VHI(I)->VXYZ(I,3)------------
        VXYZ(EP,1,1)=V13(EP,1)
        VXYZ(EP,2,1)=V13(EP,2)
        VXYZ(EP,3,1)=V13(EP,3)
C
        VXYZ(EP,1,2)=V24(EP,1)
        VXYZ(EP,2,2)=V24(EP,2)
        VXYZ(EP,3,2)=V24(EP,3)
C
        VXYZ(EP,1,3)=VHI(EP,1)
        VXYZ(EP,2,3)=VHI(EP,2)
        VXYZ(EP,3,3)=VHI(EP,3)
C-------R13(I)->RXYZ(I,1),R24(I)->RXYZ(I,2),RHI(I)->RXYZ(I,3),RTI(I)->RXYZ(I,4)------------
        RXYZ(EP,1,1)=RR(1,1)-RR(1,3)
        RXYZ(EP,2,1)=RR(2,1)-RR(2,3)
        RXYZ(EP,1,2)=RR(1,2)-RR(1,4)
        RXYZ(EP,2,2)=RR(2,2)-RR(2,4)
        RXYZ(EP,1,3)=RR(1,1)-RR(1,2)+RR(1,3)-RR(1,4)
        RXYZ(EP,2,3)=RR(2,1)-RR(2,2)+RR(2,3)-RR(2,4)
        RXYZ(EP,1,4)=RR(1,1)+RR(1,2)+RR(1,3)+RR(1,4)
        RXYZ(EP,2,4)=RR(2,1)+RR(2,2)+RR(2,3)+RR(2,4)
C-------BX0(2),BY0(2),GAMA(2),MX23,MY23,MX34,MY34,MX13,MY13
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
