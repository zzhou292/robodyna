      FAC11=FIVE_OVER_4
      IF (IHBE == 11) FAC11=FOUR_OVER_3
      DO I=LFT,LLT
        LXYZ0(1)=FOURTH*(X2L(I)+X3L(I)+X4L(I))
        LXYZ0(2)=FOURTH*(Y2L(I)+Y3L(I)+Y4L(I))
        COREL(1,1)=-LXYZ0(1)
        COREL(1,2)=X2L(I)-LXYZ0(1)
        COREL(1,3)=X3L(I)-LXYZ0(1)
        COREL(1,4)=X4L(I)-LXYZ0(1)
        COREL(2,1)=-LXYZ0(2)
        COREL(2,2)=Y2L(I)-LXYZ0(2)
        COREL(2,3)=Y3L(I)-LXYZ0(2)
        COREL(2,4)=Y4L(I)-LXYZ0(2)
        X13=(COREL(1,1)-COREL(1,3))*HALF
        X24=(COREL(1,2)-COREL(1,4))*HALF
        Y13=(COREL(2,1)-COREL(2,3))*HALF
        Y24=(COREL(2,2)-COREL(2,4))*HALF

C
        L13=X13*X13+Y13*Y13
        L24=X24*X24+Y24*Y24
        AL1(I)=MAX(L13,L24)
        C1 =COREL(1,2)*COREL(2,4)-COREL(2,2)*COREL(1,4)
        C2 =COREL(1,1)*COREL(2,3)-COREL(2,1)*COREL(1,3)
        AL2(I) =MAX(ABS(C1),ABS(C2))/AREA(I)
        RX=X24-X13
        RY=Y24-Y13
        SX=-X24-X13
        SY=-Y24-Y13
        C1=SQRT(RX*RX+RY*RY)
        C2=SQRT(SX*SX+SY*SY)
        S1=FOURTH*(MAX(C1,C2)/MIN(C1,C2)-ONE)
        FAC1=MIN(HALF,S1)+ONE
        FAC2=AREA(I)/(C1*C2)
        FAC2=3.413*MAX(ZERO,FAC2-0.7071)
        FAC2=0.78+0.22*FAC2*FAC2*FAC2
        FACI=TWO*FAC1*FAC2
        S1 = SQRT(FACI*(FAC11+AL2(I))*AL1(I))
        S1 = MAX(S1,EM20)
        AL1(I)=S1
      ENDDO 
