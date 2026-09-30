      DO EP=JFT,JLT

        NN(1)=IXC(2,EP)
        NN(2)=IXC(3,EP)
        NN(3)=IXC(4,EP)
        NN(4)=IXC(5,EP)

        VL1(EP,1)=V(1,NN(1))
        VL1(EP,2)=V(2,NN(1))
        VL1(EP,3)=V(3,NN(1))
        VL2(EP,1)=V(1,NN(2))
        VL2(EP,2)=V(2,NN(2))
        VL2(EP,3)=V(3,NN(2))
        VL3(EP,1)=V(1,NN(3))
        VL3(EP,2)=V(2,NN(3))
        VL3(EP,3)=V(3,NN(3))
        VL4(EP,1)=V(1,NN(4))
        VL4(EP,2)=V(2,NN(4))
        VL4(EP,3)=V(3,NN(4))

        VG13(1)=VL1(EP,1)-VL3(EP,1)
        VG24(1)=VL2(EP,1)-VL4(EP,1)
        VGHI(1)=VL1(EP,1)-VL2(EP,1)+VL3(EP,1)-VL4(EP,1)
C
        VG13(2)=VL1(EP,2)-VL3(EP,2)
        VG24(2)=VL2(EP,2)-VL4(EP,2)
        VGHI(2)=VL1(EP,2)-VL2(EP,2)+VL3(EP,2)-VL4(EP,2)
C
        VG13(3)=VL1(EP,3)-VL3(EP,3)
        VG24(3)=VL2(EP,3)-VL4(EP,3)
        VGHI(3)=VL1(EP,3)-VL2(EP,3)+VL3(EP,3)-VL4(EP,3)
C
        V13(EP,1)=(VQ(EP,1,1)*VG13(1)+VQ(EP,2,1)*VG13(2)+VQ(EP,3,1)*VG13(3))
        V24(EP,1)=(VQ(EP,1,1)*VG24(1)+VQ(EP,2,1)*VG24(2)+VQ(EP,3,1)*VG24(3))
        VHI(EP,1)=(VQ(EP,1,1)*VGHI(1)+VQ(EP,2,1)*VGHI(2)+VQ(EP,3,1)*VGHI(3))
        V13(EP,2)=(VQ(EP,1,2)*VG13(1)+VQ(EP,2,2)*VG13(2)+VQ(EP,3,2)*VG13(3))
        V24(EP,2)=(VQ(EP,1,2)*VG24(1)+VQ(EP,2,2)*VG24(2)+VQ(EP,3,2)*VG24(3))
        VHI(EP,2)=(VQ(EP,1,2)*VGHI(1)+VQ(EP,2,2)*VGHI(2)+VQ(EP,3,2)*VGHI(3))
        V13(EP,3)=(VQ(EP,1,3)*VG13(1)+VQ(EP,2,3)*VG13(2)+VQ(EP,3,3)*VG13(3))
        V24(EP,3)=(VQ(EP,1,3)*VG24(1)+VQ(EP,2,3)*VG24(2)+VQ(EP,3,3)*VG24(3))
        VHI(EP,3)=(VQ(EP,1,3)*VGHI(1)+VQ(EP,2,3)*VGHI(2)+VQ(EP,3,3)*VGHI(3))
      ENDDO 
C
       IF (IMPL_S==0.OR.IMP_LR>0) THEN
        DT05 = HALF*DT1
        DT025 =FOURTH*DT1
        DO I=JFT,JLT
         EXZ = Y24_T(I)*V13(I,3)-Y13_T(I)*V24(I,3)
         EYZ = -X24_T(I)*V13(I,3)+X13_T(I)*V24(I,3)
         DDRY=DT05*EXZ*AREA_I(I)
         DDRX=DT05*EYZ*AREA_I(I)
         V13X = V13(I,1)
         V24X = V24(I,1)
         VHIX = VHI(I,1)
         DDRZ1=DT025*(V13(I,2)-V24(I,2))/(X13_T(I)-X24_T(I))
         IF (ABS(X13_T(I)-X24_T(I))<EM10) DDRZ1 = ZERO
         V13(I,1) = V13(I,1)-DDRY*V13(I,3)-DDRZ1*V13(I,2)
         V24(I,1) = V24(I,1)-DDRY*V24(I,3)-DDRZ1*V24(I,2)
         VHI(I,1) = VHI(I,1)-DDRY*VHI(I,3)-DDRZ1*VHI(I,2)
         DDRZ2=DT025*(V13X+V24X)/(Y13_T(I)+Y24_T(I))
         IF (ABS(Y13_T(I)+Y24_T(I))<EM10) DDRZ2 = ZERO
         V13(I,2) = V13(I,2)-DDRX*V13(I,3)-DDRZ2*V13X
         V24(I,2) = V24(I,2)-DDRX*V24(I,3)-DDRZ2*V24X
         VHI(I,2) = VHI(I,2)-DDRX*VHI(I,3)-DDRZ2*VHIX
        ENDDO
       ENDIF 
