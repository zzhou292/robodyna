      SPLAT= 0
C [PM] in order to make all shells as non-flat shells for pinching
C [PM] flat shells algorithm not implemented for pinching
      DO EP=JFT,JLT
       Z2=ZL1(EP)*ZL1(EP)
       IF ((Z2<TOL*LL(EP).OR.NPT==1).AND.NPINCH==0) THEN
        NPLAT=NPLAT+1
        IPLAT(NPLAT)=EP
       ELSE
        SPLAT=SPLAT+1
        JPLAT(SPLAT)=EP
       ENDIF
      ENDDO 
      DO EP=NPLAT+1,JLT
        IPLAT(EP)=JPLAT(EP-NPLAT)
      ENDDO 
