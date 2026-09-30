      DO I=JFT,JLT
        VISCMX(I) = MAX(VISCMX(I), AMU(I))
        VISCMX(I) = SQRT(ONE + VISCMX(I)*VISCMX(I)) - VISCMX(I)
        ALDT(I)   = ALDT(I) * VISCMX(I) / SQRT(ALPE(I))
      ENDDO
