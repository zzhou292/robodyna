          DO I=LFT,LLT
            RHO  =PM(1,IMAT)
            YOUNG=PM(20,IMAT)
            NU   =PM(21,IMAT)
            SSP(I)=SQRT(YOUNG/(ONE-NU*NU)/RHO)
          ENDDO
