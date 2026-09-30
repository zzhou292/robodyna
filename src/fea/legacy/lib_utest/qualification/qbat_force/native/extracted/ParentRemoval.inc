        ! Check element failure (when IPG = NPG = 4)
        IF (IPG == NPG) THEN
          DO IEL=1,NEL
            IF (OFF(IEL) == ONE) THEN
              COUNTPG = 0
              DO IG=1,NPG
                JPG  = (IG-1)*NEL
                COUNTPG = COUNTPG + ELBUF_STR%BUFLY(IL)%OFFPG(JPG+IEL)
              ENDDO ! |-> IG
              IF (COUNTPG == 0) THEN
                OFF(IEL)  = FOUR_OVER_5  
                PRINT_FAIL(IEL) = .FALSE.
