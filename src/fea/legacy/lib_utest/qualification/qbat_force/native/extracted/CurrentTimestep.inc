        DO I=JFT,JLT
          DTEL(I)=F_DTE(I)*ALDT(I)/SSP(I)     ! DT=ALDT(I)/SSP(I) & DT=DT*DTFAC1 does not lead to same digits, so operation is made again here.
        ENDDO
