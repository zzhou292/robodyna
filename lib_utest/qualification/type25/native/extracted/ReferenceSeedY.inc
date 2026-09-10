            RLOC(1,I)=SKEW(4,ISK)
            RLOC(2,I)=SKEW(5,ISK)
            RLOC(3,I)=SKEW(6,ISK)
            NRLOC(I)=RLOC(1,I)**2+RLOC(2,I)**2+RLOC(3,I)**2
            PRVC(1,I)=Y1*RLOC(3,I)-Z1*RLOC(2,I)
            PRVC(2,I)=Z1*RLOC(1,I)-X1*RLOC(3,I)
            PRVC(3,I)=X1*RLOC(2,I)-Y1*RLOC(1,I)
            NPRVC(I)=PRVC(1,I)**2+PRVC(2,I)**2+PRVC(3,I)**2
            IF (SQRT(NPRVC(I)/NRLOC(I))/X0(I) < EM5) THEN
