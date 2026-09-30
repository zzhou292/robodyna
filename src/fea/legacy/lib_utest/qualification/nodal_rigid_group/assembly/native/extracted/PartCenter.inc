        MASRB=MS(M)
        DO J=1,3
          XG(J)=X(J,M)
          X(J,M)=X(J,M)*MS(M)
        ENDDO
        DO I=1,NSL
         N=LSN(I)
         DO J=1,3
          X(J,M)    = X(J,M)+X(J,N)*MS_LOC(I)
         ENDDO
         MASRB   = MASRB+MS_LOC(I)
        ENDDO
