        ! Check in-plane Gauss point failure
        NPTT = ELBUF_STR%BUFLY(IL)%NPTT
        OFFPG => ELBUF_STR%BUFLY(IL)%OFFPG(JPG+1:JPG+NEL)
        DO IEL=1,NEL
          IF (OFF(IEL) == ONE) THEN 
            DO IFL = 1,NFAIL
              THFACT = ZERO
              NPFAIL = ZERO
              DO IPT=1,NPTT
                FOFF => ELBUF_STR%BUFLY(IL)%FAIL(IR,IS,IPT)%FLOC(IFL)%OFF
                ! Computation of broken fraction of thickness / 
                ! ratio of intg. points                
                IF (FOFF(IEL) < 1)  THEN
                  IPOS = (IPT-1)*NEL + IEL
                  THFACT = THFACT + THKLY(IPOS)
                  NPFAIL = NPFAIL + ONE/NPTT
                ENDIF
                ! Comparison with critical value PTHICKFAIL
                IF (((THFACT >= PTHKF(IL,IFL)).AND.(PTHKF(IL,IFL) > ZERO)).OR.
     .              ((NPFAIL >= ABS(PTHKF(IL,IFL))).AND.(PTHKF(IL,IFL) < ZERO))) THEN
                  OFFPG(IEL) = 0
                  FAIL_NUM(IEL) = IFL
                ENDIF 
              ENDDO ! |-> IPT    
            ENDDO ! |-> IFL
          ENDIF
        ENDDO ! |-> IEL
