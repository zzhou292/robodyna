      DO I=1,NEL
        X0DP(I)= X0(I)                     ! cast double to My_real
      ENDDO
C
      IF (SCODVER >= 101) THEN
        DO I=1,NEL
          X0DP(I)= X0(I) + X0_ERR(I)         ! difference between double and my_real
        ENDDO
      ENDIF
