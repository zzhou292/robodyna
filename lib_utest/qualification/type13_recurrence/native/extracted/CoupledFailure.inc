      DO I=1,NEL
        ISRATE = INT (GEO(127,MGN(I)))
C---- smoothing factor alpha = 2PI*fc*dt/(2PI*fc*dt+1) ---
        ASRATE = (2*PI*GEO(128,MGN(I))*DT1)/(ONE+2*PI*GEO(128,MGN(I))*DT1)
        IF (ISRATE /= 0) THEN
          IF (CRIT_NEW(I) < ONE) THEN 
            CRIT(I) = MIN(CRIT(I),ONE+EM3)
            CRIT(I) = ASRATE*CRIT(I) + (ONE - ASRATE)*CRIT_NEW(I)
            CRIT_NEW(I) = MIN(CRIT(I),ONE)
          ELSE
            CRIT_NEW(I) = ONE
          ENDIF
        ELSE
          IF (CRIT_NEW(I) < ONE) THEN 
            CRIT_NEW(I) = MIN(CRIT(I),ONE)
          ELSE
            CRIT_NEW(I) = ONE
          ENDIF
        ENDIF
        IF (OFF(I) == ONE) THEN
           IF (CRIT(I) >= ONE) THEN
            OFF(I)=ZERO
            NINDX = NINDX + 1
            INDX(NINDX) = I
            IDEL7NOK = 1
          ENDIF
        ENDIF
      ENDDO
