      DO I=JFT,JLT
        RX(I)=X(1,IXC(3,I))+X(1,IXC(4,I))-X(1,IXC(2,I))-X(1,IXC(5,I))
        SX(I)=X(1,IXC(4,I))+X(1,IXC(5,I))-X(1,IXC(2,I))-X(1,IXC(3,I))
        RY(I)=X(2,IXC(3,I))+X(2,IXC(4,I))-X(2,IXC(2,I))-X(2,IXC(5,I))
        SY(I)=X(2,IXC(4,I))+X(2,IXC(5,I))-X(2,IXC(2,I))-X(2,IXC(3,I))
        RZ(I)=X(3,IXC(3,I))+X(3,IXC(4,I))-X(3,IXC(2,I))-X(3,IXC(5,I))
        SSZ(I)=X(3,IXC(4,I))+X(3,IXC(5,I))-X(3,IXC(2,I))-X(3,IXC(3,I))
      ENDDO 
C----------------------------
C     LOCAL SYSTEM
C----------------------------
      K = 0
      CALL CLSKEW3(JFT,JLT,K,
     .   RX, RY, RZ, 
     .   SX, SY, SSZ, 
     .   R11,R12,R13,R21,R22,R23,R31,R32,R33,DETA1,OFFG )
      DO I=JFT,JLT
        AREA(I)=FOURTH*DETA1(I)
        AREA_I(I)=ONE/AREA(I)
        VQ(I,1,1)=R11(I)
        VQ(I,2,1)=R21(I)
        VQ(I,3,1)=R31(I)
        VQ(I,1,2)=R12(I)
        VQ(I,2,2)=R22(I)
        VQ(I,3,2)=R32(I)
        VQ(I,1,3)=R13(I)
        VQ(I,2,3)=R23(I)
        VQ(I,3,3)=R33(I)
      ENDDO 
c
      DO I=JFT,JLT

        J=IXC(2,I)
        K=IXC(3,I)
        L=IXC(4,I)
        M=IXC(5,I)

        LXYZ0(1)=FOURTH*(X(1,L)+X(1,M)+X(1,J)+X(1,K))
        LXYZ0(2)=FOURTH*(X(2,L)+X(2,M)+X(2,J)+X(2,K))
        LXYZ0(3)=FOURTH*(X(3,L)+X(3,M)+X(3,J)+X(3,K))

        XX1=X(1,K)-X(1,J)
        YY1=X(2,K)-X(2,J)
        ZZ1=X(3,K)-X(3,J)

        XL2(I)=R11(I)*XX1+R21(I)*YY1+R31(I)*ZZ1
        YL2(I)=R12(I)*XX1+R22(I)*YY1+R32(I)*ZZ1

        XX2=X(1,J)-LXYZ0(1)
        YY2=X(2,J)-LXYZ0(2)
        ZZ2=X(3,J)-LXYZ0(3)
        ZL1(I)=R13(I)*XX2+R23(I)*YY2+R33(I)*ZZ2

        XX3=X(1,L)-X(1,J)
        YY3=X(2,L)-X(2,J)
        ZZ3=X(3,L)-X(3,J)
        XL3(I)=R11(I)*XX3+R21(I)*YY3+R31(I)*ZZ3
        YL3(I)=R12(I)*XX3+R22(I)*YY3+R32(I)*ZZ3

        XX4=X(1,M)-X(1,J)
        YY4=X(2,M)-X(2,J)
        ZZ4=X(3,M)-X(3,J)
        XL4(I)=R11(I)*XX4+R21(I)*YY4+R31(I)*ZZ4
        YL4(I)=R12(I)*XX4+R22(I)*YY4+R32(I)*ZZ4
