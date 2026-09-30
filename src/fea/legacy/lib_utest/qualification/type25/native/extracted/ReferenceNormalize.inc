        RLOC(1,I)=PRVC(2,I)*Z1-PRVC(3,I)*Y1
        RLOC(2,I)=PRVC(3,I)*X1-PRVC(1,I)*Z1
        RLOC(3,I)=PRVC(1,I)*Y1-PRVC(2,I)*X1
        NRLOC(I)=SQRT(RLOC(1,I)**2+RLOC(2,I)**2+RLOC(3,I)**2)
        RLOC(1,I)=RLOC(1,I)/NRLOC(I)
        RLOC(2,I)=RLOC(2,I)/NRLOC(I)
        RLOC(3,I)=RLOC(3,I)/NRLOC(I)
