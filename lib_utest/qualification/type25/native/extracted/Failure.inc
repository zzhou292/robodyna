      DO I=1,NEL
        IF (IFAIL(I) == 0) THEN 
          IF (CRITNEW(I) < ONE) THEN 
            CRITNEW(I) = MIN(CRIT(I),ONE)
          ELSE 
            CRITNEW(I) = ONE
          ENDIF
        ELSEIF (IFAIL(I) == 1) THEN 
          IF (CRITNEW(I) < ONE) THEN 
            CRITNEW(I) = MIN(CRIT(I)/(XL0(I)*XL0(I)),ONE)
          ELSE 
            CRITNEW(I) = ONE
          ENDIF
        ENDIF
        IF (OFF(I) == ONE .AND. IFAIL(I) == 1) THEN
          IF (CRIT(I)/(XL0(I)*XL0(I)) > ONE) THEN
            OFF(I)=ZERO
            NINDX = NINDX + 1
            INDX(NINDX) = I
            IDEL7NOK = 1
            CRITNEW(I) = ONE
          ENDIF
        ENDIF
      ENDDO
