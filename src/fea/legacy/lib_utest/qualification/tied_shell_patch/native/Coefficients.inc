        INX=INS + XMSI*(XS*XS+YS*YS+ZS*ZS)
        MRX = (B1+C3+C2)
        MRY = (B2+C1+C3)
        MRZ = (B3+C2+C1)
        MR=DET*INX*MAX(MRX,MRY,MRZ)
C
C------------------------------------------------------
C     MASSES TRANSMISES AUX NODES MAINS
C------------------------------------------------------
C
        FACT = ONE
        IF (IRODDL==1) THEN
        IF (MINER(J1)>ZERO.AND.MINER(J2)>ZERO.AND.MINER(J3)>ZERO.AND.MINER(J4)>ZERO) THEN
C- inertia transmitted in the form of inertia
          FACT = ZERO
        ENDIF
        ENDIF
C
        XMSI=FOURTH*XMSI+MR*FACT
C
        DMAST = DMAST + FOUR*XMSI - MS(I)
        IF (ANIM_N(2)+OUTP_N(2)+H3D_DATA%N_SCAL_DMAS >0) THEN
         ADM(J1) = ADM(J1) + XMSI - FOURTH*MS(I)
         ADM(J2) = ADM(J2) + XMSI - FOURTH*MS(I)
         ADM(J3) = ADM(J3) + XMSI - FOURTH*MS(I)
         ADM(J4) = ADM(J4) + XMSI - FOURTH*MS(I)
        ENDIF
C
        MS(J1)=MS(J1)+XMSI
        MS(J2)=MS(J2)+XMSI
        MS(J3)=MS(J3)+XMSI
        MS(J4)=MS(J4)+XMSI
C
        IF (IRODDL == 1) THEN
          STF = (FOURTH*STIFN(I)
     .         + DET*MAX(MRX,MRY,MRZ)*(STIFR(I)+STIFN(I)*(XS*XS+YS*YS+ZS*ZS)))*WEIGHT(I)
        ELSE
          STF = (FOURTH*STIFN(I)
     .         + DET*MAX(MRX,MRY,MRZ)*(STIFN(I)*(XS*XS+YS*YS+ZS*ZS)))*WEIGHT(I)
        ENDIF
C
        STIFN(J1)=STIFN(J1) + STF
        STIFN(J2)=STIFN(J2) + STF
        STIFN(J3)=STIFN(J3) + STF
        STIFN(J4)=STIFN(J4) + STF
C
        IF (IRODDL==1) THEN
          IN(J1)=IN(J1)+INX*FOURTH*(ONE-FACT)
          IN(J2)=IN(J2)+INX*FOURTH*(ONE-FACT)
          IN(J3)=IN(J3)+INX*FOURTH*(ONE-FACT)
          IN(J4)=IN(J4)+INX*FOURTH*(ONE-FACT)
        ENDIF
C
        IF(IDEL2/=0.AND.MS(I)/=0.)SMASS(II)=MS(I)
        MS(I)=ZERO
        STIFN(I)=EM20
C  
        IF (IRODDL==1) THEN
          IF(IDEL2/=0.AND.IN(I)/=0.)SINER(II)=IN(I)
          IN(I)=ZERO
          STIFR(I)=EM20
        ENDIF
