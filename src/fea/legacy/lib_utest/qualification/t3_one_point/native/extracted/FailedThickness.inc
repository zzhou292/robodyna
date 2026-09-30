            DO IEL=1,NEL                                                       
              IF (OFF(IEL) == ONE) THEN                                        
                IF (FOFF(IEL) < ONE)  THEN                
                  IPOS = (IPT-1)*NEL + IEL                                      
                  THFACT(IEL) = THFACT(IEL) + THKLY(IPOS) ! -> Percentage of broken thickness (accounting for integration weight)
                  NPFAIL(IEL) = NPFAIL(IEL) + ONE/NPTT    ! -> Ratio of broken integration points
                ENDIF                                                           
              ENDIF                                               
            ENDDO ! |-> IEL                                          
