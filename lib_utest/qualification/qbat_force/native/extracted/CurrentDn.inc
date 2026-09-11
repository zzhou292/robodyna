      IF (IMPL_S == 1) THEN
        DN = ZERO
      ELSE
        DN = GEO(13,PID(1))
        IF(DN == ZERO ) THEN
            IF (IHBE == 11)THEN
              DN = EM3
            ELSEIF(IHBE == 30)THEN
              DN = EM4
            ENDIF
        ENDIF    
      ENDIF
      AMU(JFT:JLT) = DN
