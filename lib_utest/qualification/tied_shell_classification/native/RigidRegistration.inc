        IF(IDDLEVEL==0)THEN
         IF(IKREM == 0)THEN
          DO J=1,NSL
           DO IDIR=1,6
             CALL KINSET(8,ITAB(LPBY(J+K)),IKINE(LPBY(J+K)),IDIR,0,
     .                   IKINE1(LPBY(J+K)))
           ENDDO
          ENDDO
         ELSE
          DO J=1,NSL
           DO IDIR=1,6
             CALL KINSET(128,ITAB(LPBY(J+K)),IKINE(LPBY(J+K)),IDIR,0,
     .                   IKINE1(LPBY(J+K)))
           ENDDO
          ENDDO
         ENDIF
        ENDIF
