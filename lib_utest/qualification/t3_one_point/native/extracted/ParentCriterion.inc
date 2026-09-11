          DO IEL=1,NEL                                                          
            IF (OFF(IEL) == ONE) THEN                                        
              IF (((THFACT(IEL) >= PTHKF(IL,IFL)).AND.(PTHKF(IL,IFL) > ZERO)).OR.
     .            ((NPFAIL(IEL) >= ABS(PTHKF(IL,IFL))).AND.(PTHKF(IL,IFL) < ZERO))) THEN
                OFF(IEL) = FOUR_OVER_5
                PRINT_FAIL(IEL) = .FALSE.
