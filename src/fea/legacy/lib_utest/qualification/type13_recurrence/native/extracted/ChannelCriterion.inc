      DLIM = ZERO
      DO I=1,NEL
        CC = GEO(103,MGN(I))
        CN = GEO(109,MGN(I))
        XA = GEO(115,MGN(I))
        XB = GEO(121,MGN(I))
        IF (OFF(I)  == ONE .AND. DMX(I) /= ZERO .AND. DMN(I)/= ZERO) THEN
          IF (IFAIL2(I) == 0) THEN
            XA = ONE
            XB = TWO
            IF (DX(I) > ZERO) THEN
              DLIM = DX(I) / (XL0(I)*DMX(I))
            ELSE
              DLIM = DX(I) / (XL0(I)*DMN(I)) 
            ENDIF
          ELSE
            VFAIL = CC * (ABS(DV(I)/VRT(I)))**CN
            IF (IFAIL2(I) == 1) THEN
              IF (DX(I) > ZERO) THEN
                DLIM = DX(I) / (XL0(I)*DMX(I) + VFAIL)
              ELSE
                DLIM = DX(I) / (XL0(I)*DMN(I) - VFAIL)
              ENDIF
            ELSEIF (IFAIL2(I) == 2) THEN
              IF (FX(I) > ZERO) THEN
                DLIM = FX(I) / (DMX(I) + VFAIL)
              ELSE
                DLIM = FX(I) / (DMN(I) - VFAIL)
              ENDIF
            ELSEIF (IFAIL2(I) == 3) THEN
              DLIM = MAX(ZERO,E6(I,1)) / (DMX(I) + VFAIL)
            ENDIF
          ENDIF
          IF (IFAIL(I) == 0) THEN
!         Uniaxial failure
            CRIT(I) = MAX(CRIT(I),XA*DLIM)
          ELSE
!         Multiaxial failure
            CRIT(I)= CRIT(I) + XA*DLIM**XB
          ENDIF
        ENDIF
      ENDDO
