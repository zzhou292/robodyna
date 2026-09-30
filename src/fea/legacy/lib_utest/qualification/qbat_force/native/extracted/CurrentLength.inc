       DO I=JFT,JLT
        RX(I)=XL2(I)+XL3(I)-XL4(I)
        RY(I)=YL2(I)+YL3(I)-YL4(I)
        SX(I)=-XL2(I)+XL3(I)+XL4(I)
        SY(I)=-YL2(I)+YL3(I)+YL4(I)
        C1=SQRT(RX(I)*RX(I)+RY(I)*RY(I))
        C2=SQRT(SX(I)*SX(I)+SY(I)*SY(I))
        S1=FOURTH*(MAX(C1,C2)/MIN(C1,C2)-ONE)
        FAC1=MIN(HALF,S1)+ONE
        FAC2=FOUR*AREA(I)/(C1*C2)
        FAC2=3.413*MAX(ZERO,FAC2-0.7071)
        FAC2=0.78+0.22*FAC2*FAC2*FAC2
        FACI=TWO*FAC1*FAC2
C
        FACN(I,1)=SQRT(L24(I)/LL(I))
        FACN(I,2)=SQRT(L13(I)/LL(I))
        S1 = SQRT(FACI*(FOUR_OVER_3+LM(I)*AREA_I(I))*LL(I))
        S1 = MAX(S1,EM20)
        LL(I) = AREA(I)/S1
       ENDDO
