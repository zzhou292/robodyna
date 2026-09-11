            IF (IS1==-1) THEN 
               NSN_MULTI_CONNEC = NSN_MULTI_CONNEC + NSN
            ELSE
              DO I=1,IGRNOD(ISU1)%NENTITY
                ISL = IGRNOD(ISU1)%ENTITY(I)
                T2_NB_CONNEC(ISL) = T2_NB_CONNEC(ISL) + 1
                IF (T2_NB_CONNEC(ISL) == 2) NSN_MULTI_CONNEC = NSN_MULTI_CONNEC + 1
              ENDDO
            END IF            
