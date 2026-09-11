      IF(NPT==1) THEN
        DO I=JFT,JLT
         SHF(I)=ZERO
        ENDDO
      ELSE
        DO I=JFT,JLT
          SHF(I)=GEO(38,PID(I))
        ENDDO
      ENDIF
      DO I=JFT,JLT
         GS(I)=G(I)*SHF(I)
      ENDDO
      IF(MTN>=24)THEN
        DO I=JFT,JLT
          A12(I)  =NU(I)*A11(I)
        ENDDO
      ENDIF
