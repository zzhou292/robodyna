      IF (TT == ZERO) THEN
        DO I=1,NEL
          X0(I)= ALDP(I)                   ! cast double to My_real
        ENDDO
      ENDIF
C
      IF (SCODVER >= 101) THEN
        IF (TT == ZERO) THEN
          DO I=1,NEL
            X0_ERR(I)= ALDP(I)-X0(I)         ! difference between double and my_real
          ENDDO
        ENDIF
      ENDIF
