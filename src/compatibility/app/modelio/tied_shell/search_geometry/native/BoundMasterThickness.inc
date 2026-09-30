          local_thkmain = zero 
          IF (NELC /= 0) THEN
            MG=IXC(6,NELC)
            IP = IPARTC(NELC) 
            IF ( THK_PART(IP) /= ZERO) THEN  
              local_thkmain = MAX(local_thkmain,THK_PART(IP))
            ELSEIF ( THK(NELC) /= ZERO) THEN
              local_thkmain = MAX(local_thkmain,THK(NELC))
            ELSE
              local_thkmain = MAX(local_thkmain,GEO(1,MG))
            ENDIF
          ELSEIF (NELTG /= 0)THEN
            MG=IXTG(5,NELTG)
            IP = IPARTTG(NELTG) 
            IF ( THK_PART(IP) /= ZERO) THEN  
              local_thkmain = MAX(local_thkmain,THK_PART(IP))
            ELSEIF ( THK(NUMELC+NELTG) /= ZERO) THEN
              local_thkmain = MAX(local_thkmain,THK(NUMELC+NELTG))
            ELSE
              local_thkmain = MAX(local_thkmain,GEO(1,MG))
            ENDIF
