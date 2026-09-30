      IF(ITHERM_FE == 0 ) THEN
       DO J=1,NUMELS
        I = INDEX(J)
        DO K=1,8
          N = IXS(K+1,I)
          MS(N) = MS(N) + MSS(K,I)
        ENDDO
       ENDDO
