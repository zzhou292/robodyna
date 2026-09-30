      DO I=JFT,JLT
        STI(1,I) = ZERO
        STI(2,I) = ZERO
        STIR(1,I) = ZERO
        STIR(2,I) = ZERO
        IF(OFF(I)>ZERO)THEN
C        
          IRB1 = NINT(RBY_NODE(2*(I-1)+1))
          IRB2 = NINT(RBY_NODE(2*(I-1)+2))
C        
          IF(MS(NC1(I))/=ZERO)THEN
            MASS2 = TWO * MS(NC1(I))
            IF (IRB1 > 0) THEN
              IF (MS(IRB1) > MS(NC1(I))) MASS2 =  TWO * MS(IRB1)
            ENDIF
          ELSE
           MASS2 = XM(I)
          ENDIF
C        
          IF(XKM(I)/=ZERO.AND.XCM(I)/=ZERO.AND.
     .      MASS2/=ZERO)THEN
            STI(1,I) = (XCM(I) + SQRT(XCM(I)**2+XKM(I)*MASS2))**2/MASS2
          ELSEIF(XKM(I)/=ZERO)THEN
            STI(1,I) = XKM(I)
          ELSEIF(XCM(I)/=ZERO.AND.MASS2/=ZERO)THEN
            A = FOUR * XCM(I)**2 
            STI(1,I) = A / MASS2
          ENDIF
C
          IF(MS(NC2(I))/=ZERO)THEN
            MASS2 = TWO * MS(NC2(I))
            IF (IRB2 > 0) THEN
              IF (MS(IRB2) > MS(NC2(I))) MASS2 =  TWO * MS(IRB2)
            ENDIF
          ELSE
            MASS2 = XM(I)
          ENDIF
C        
          IF(XKM(I)/=ZERO.AND.XCM(I)/=ZERO.AND.
     .      MASS2/=ZERO)THEN
            STI(2,I) = (XCM(I) + SQRT(XCM(I)**2+XKM(I)*MASS2))**2/MASS2
          ELSEIF(XKM(I)/=ZERO)THEN
            STI(2,I) = XKM(I)
          ELSEIF(XCM(I)/=ZERO.AND.MASS2/=ZERO)THEN
            A = FOUR * XCM(I)**2 
            STI(2,I) = A / MASS2
          ENDIF
C
          IF(IN(NC1(I))/=ZERO)THEN
            IN2 = TWO * IN(NC1(I))
            IF (IRB1 > 0) THEN
              IF (IN(IRB1) > IN(NC1(I))) IN2 =  TWO * IN(IRB1)
            ENDIF
          ELSE
            IN2 = XIN(I)
          ENDIF
          IF(XKR(I)/=ZERO.AND.XCR(I)/=ZERO.AND.
     .      IN2/=ZERO)THEN
            STIR(1,I) = (XCR(I) + SQRT(XCR(I)**2+XKR(I)*IN2))**2/IN2
          ELSEIF(XKR(I)/=ZERO)THEN
            STIR(1,I) = XKR(I)
          ELSEIF(XCR(I)/=ZERO.AND.IN2/=ZERO)THEN
            A = FOUR * XCR(I)**2 
            STIR(1,I) = A / IN2
          ENDIF
C
          IF(IN(NC2(I))/=ZERO)THEN
            IN2 = TWO * IN(NC2(I))
            IF (IRB2 > 0) THEN
              IF (IN(IRB2) > IN(NC2(I))) IN2 =  TWO * IN(IRB2)
            ENDIF
          ELSE
            IN2 = XIN(I)
          ENDIF
C          
          IF(XKR(I)/=ZERO.AND.XCR(I)/=ZERO.AND.
     .      IN2/=ZERO)THEN
            STIR(2,I) = (XCR(I) + SQRT(XCR(I)**2+XKR(I)*IN2))**2/IN2
          ELSEIF(XKR(I)/=ZERo)THEN
            STIR(2,I) = XKR(I)
          ELSEIF(XCR(I)/=ZERO.AND.IN2/=ZERO)THEN
            A = FOUR * XCR(I)**2 
            STIR(2,I) = A / IN2
          ENDIF
        ENDIF
      ENDDO ! DO I=JFT,JLT
