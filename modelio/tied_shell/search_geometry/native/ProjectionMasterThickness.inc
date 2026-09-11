          IF(NELTG/=0) THEN
            MG=IXTG(5,NELTG)
              IP = IPARTTG(NELTG) 
            IF ( THK_PART(IP) /= ZERO) THEN  
              THKMAIN = THK_PART(IP)
              ELSEIF ( THK(NUMELC+NELTG) /= ZERO) THEN
              THKMAIN = THK(NUMELC+NELTG)
              ELSE
              THKMAIN = GEO(1,MG)
              ENDIF
          ELSEIF(NELC/=0) THEN
            MG=IXC(6,NELC)
              IP = IPARTC(NELC) 
            IF ( THK_PART(IP) /= ZERO) THEN  
              THKMAIN = THK_PART(IP)
              ELSEIF ( THK(NELC) /= ZERO) THEN
              THKMAIN = THK(NELC)
              ELSE
              THKMAIN = GEO(1,MG)
              ENDIF
