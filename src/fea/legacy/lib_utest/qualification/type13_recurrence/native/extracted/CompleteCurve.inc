      SUBROUTINE VINTER2(TF,IAD,IPOS ,ILEN,NEL0,X,DYDX,Y)
C same as vinter with possibility to decrease x
C-----------------------------------------------
C   I m p l i c i t   T y p e s
C-----------------------------------------------
#include      "implicit_f.inc"
C
      INTEGER ILEN(*),IPOS(*),IAD(*),NEL0
      my_real X(*),DYDX(*),Y(*),TF(2,*)
      INTEGER I,J1,J,ICONT,J2

      my_real TF2J2,TF2J1,TF1J2,TF1J1,
     .  XDP,YYDP, DYDXDP
      LOGICAL COND1, COND2
C
      J = 0
      ICONT = 1
      DO WHILE(ICONT==1)
C 
       J = J+1
       ICONT = 0
       DO I=1,NEL0
         J1 = IPOS(I)+IAD(I)+1
         COND1 = J<=ILEN(I)-1
         IF(COND1) COND1 = X(I)>TF(1,J1)
         COND2 = IPOS(I)>=1
         IF(COND2) COND2 = X(I)<TF(1,J1-1)
         IF(COND1)THEN
           IPOS(I)=IPOS(I)+1
           ICONT = 1
         ELSEIF(COND2)THEN
           IPOS(I)=IPOS(I)-1
           ICONT = 1
         ENDIF
       ENDDO
C
      ENDDO
C
      DO I=1,NEL0
        J1   =IPOS(I)+IAD(I)
        J2   = J1+1

        TF2J2 = TF(2,J2)
        TF2J1 = TF(2,J1)
        TF1J2 = TF(1,J2)
        TF1J1 = TF(1,J1)
        XDP = X(I)

ccc        DYDX(I)=(TF(2,J2)-TF(2,J1))/(TF(1,J2)-TF(1,J1))

        DYDXDP = (TF2J2-TF2J1)/(TF1J2-TF1J1)
        DYDX(I) = DYDXDP
ccc        Y(I)   = TF(2,J1) + DYDX(I)*(X(I)-TF(1,J1))
        YYDP = TF2J1 + DYDXDP * (XDP - TF1J1)
        Y(I) = YYDP

      ENDDO
C
      RETURN
      END
