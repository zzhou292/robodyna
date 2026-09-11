      DO I=JFT,JLT
        DIVM=MAX(ALDT(I)*ALDT(I),EM20)
        STI(I) = HALF*F_OSET(I)*VOL0(I) * A1(I)* OFF(I) / DIVM
        STIR(I)= ZERO
      ENDDO
