        IF (PTHK == ZERO) PTHK = ONE-EM06
        PTHK = MIN(PTHK, ONE)
        PTHK = MAX(PTHK,-ONE) 
        GEO(42) = PTHK
