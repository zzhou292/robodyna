        DO J=1,3
          XG(J)=X(J,M)
          X(J,M)=X(J,M)*MASRB
        ENDDO
        XMG=MASRB
C
        DO I=1,NPBY(15,NRB)
          N=LSN_XTRA(I)
          DO J=1,3
            X(J,M)    = X(J,M)+X(J,N)*MS_LOC(NSL+I)
          ENDDO
          MASRB   = MASRB+MS_LOC(NSL+I)
        ENDDO
C    
        DO J=1,3
          X(J,M)=X(J,M)/MASRB
          XG(J)=X(J,M)          
        ENDDO
