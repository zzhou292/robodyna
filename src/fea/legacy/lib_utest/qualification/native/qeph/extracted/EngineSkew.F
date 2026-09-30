Copyright>        OpenRadioss
Copyright>        Copyright (C) 2026 Siemens
Copyright>
Copyright>        This program is free software: you can redistribute it and/or modify
Copyright>        it under the terms of the GNU Affero General Public License as published by
Copyright>        the Free Software Foundation, either version 3 of the License, or
Copyright>        (at your option) any later version.
Copyright>
Copyright>        This program is distributed in the hope that it will be useful,
Copyright>        but WITHOUT ANY WARRANTY; without even the implied warranty of
Copyright>        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
Copyright>        GNU Affero General Public License for more details.
Copyright>
Copyright>        You should have received a copy of the GNU Affero General Public License
Copyright>        along with this program.  If not, see <https://www.gnu.org/licenses/>.
Copyright>
Copyright>
Copyright>        Commercial Alternative: Simcenter Radioss Software
Copyright>
Copyright>        As an alternative to this open-source version, Siemens also offers Simcenter(TM) Radioss(R)
Copyright>        software under a commercial license.  Contact Siemens to discuss further if the
Copyright>        commercial version may interest you: 
Copyright>        https://www.siemens.com/en-us/products/simcenter/mechanical-simulation/radioss/.

! Complete pinned routines, extracted without arithmetic edits.
      SUBROUTINE CLSKEW3(JFT,JLT,IREP,
     .   RX, RY, RZ, SX, SY, SZ, 
     .   E1X, E2X, E3X, E1Y, E2Y, E3Y, E1Z, E2Z, E3Z,
     .   DET,OFF )
C-----------------------------------------------
C   I m p l i c i t   T y p e s
C-----------------------------------------------
#include      "implicit_f.inc"
C-----------------------------------------------
C   G l o b a l   P a r a m e t e r s
C-----------------------------------------------
#include      "mvsiz_p.inc"
#include      "scr17_c.inc"
C-----------------------------------------------
C   D u m m y   A r g u m e n t s
C-----------------------------------------------
      INTEGER JFT,JLT,IREP
      my_real
     .   RX(*) , RY(*) , RZ(*), 
     .   SX(*) , SY(*) , SZ(*),
     .   E1X(*), E1Y(*), E1Z(*),
     .   E2X(*), E2Y(*), E2Z(*),
     .   E3X(*), E3Y(*), E3Z(*), DET(*), OFF(*)
C-----------------------------------------------
C   L o c a l   V a r i a b l e s
C-----------------------------------------------
      INTEGER I
      MY_REAL C1,C2,CC,C1C1,C2C2,C1_1(MVSIZ),C2_1(MVSIZ)
      my_real :: OFF_LOC
C~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
C IREP=0 ->QEPH IREP=1 ->Q4, IREP=2-> E1=R(ISHFRAM=1)
C~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      DO I=JFT,JLT
C---------E3------------
        E3X(I) = RY(I) * SZ(I) - RZ(I) * SY(I) 
        E3Y(I) = RZ(I) * SX(I) - RX(I) * SZ(I) 
        E3Z(I) = RX(I) * SY(I) - RY(I) * SX(I) 
        DET(I) = SQRT(E3X(I)*E3X(I) + E3Y(I)*E3Y(I) + E3Z(I)*E3Z(I))
        IF (DET(I) < EM20 .AND. OFF(I) /= ZERO) THEN
          OFF(I)=ZERO
          IDEL7NOK = 1
        ENDIF
        OFF_LOC = ZERO
        IF(ABS(OFF(I))/=ZERO) OFF_LOC = ONE
        DET(I)=MAX(EM20,DET(I))
        CC = OFF_LOC/DET(I)
        CC = MAX(CC,EM20)
        E3X(I) = E3X(I) * CC 
        E3Y(I) = E3Y(I) * CC 
        E3Z(I) = E3Z(I) * CC 
      ENDDO 
C~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      IF (IREP==2) THEN
       DO I=JFT,JLT
         E1X(I) = RX(I)
         E1Y(I) = RY(I)
         E1Z(I) = RZ(I)
       ENDDO 
      ELSEIF (IREP==1) THEN
       DO I=JFT,JLT
         C2 = SQRT(SX(I)*SX(I) + SY(I)*SY(I) + SZ(I)*SZ(I))
         E1X(I) = RX(I)*C2+(SY(I)*E3Z(I)-SZ(I)*E3Y(I))
         E1Y(I) = RY(I)*C2+(SZ(I)*E3X(I)-SX(I)*E3Z(I))
         E1Z(I) = RZ(I)*C2+(SX(I)*E3Y(I)-SY(I)*E3X(I))
       ENDDO 
      ELSE
       DO I=JFT,JLT
         C1C1 = RX(I)*RX(I) + RY(I)*RY(I) + RZ(I)*RZ(I)
         C2C2 = SX(I)*SX(I) + SY(I)*SY(I) + SZ(I)*SZ(I)
         IF(C1C1 /= ZERO) THEN
           C2_1(I) = SQRT(C2C2/MAX(EM20,C1C1))
           C1_1(I) = ONE
         ELSEIF(C2C2 /= ZERO)THEN
           C2_1(I) = ONE
           C1_1(I) = SQRT(C1C1/MAX(EM20,C2C2))
         END IF 
       ENDDO
       DO I=JFT,JLT
         E1X(I) = RX(I)*C2_1(I)+(SY(I)*E3Z(I)-SZ(I)*E3Y(I))*C1_1(I)
         E1Y(I) = RY(I)*C2_1(I)+(SZ(I)*E3X(I)-SX(I)*E3Z(I))*C1_1(I) 
         E1Z(I) = RZ(I)*C2_1(I)+(SX(I)*E3Y(I)-SY(I)*E3X(I))*C1_1(I)
       ENDDO 
      ENDIF 
C~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
       DO I=JFT,JLT
         C1 = SQRT(E1X(I)*E1X(I) + E1Y(I)*E1Y(I) + E1Z(I)*E1Z(I))
         IF(C1 /= ZERO) C1 = ONE / MAX(EM20,C1)
         E1X(I) = E1X(I)*C1
         E1Y(I) = E1Y(I)*C1
         E1Z(I) = E1Z(I)*C1
         E2X(I) = E3Y(I) * E1Z(I) - E3Z(I) * E1Y(I)
         E2Y(I) = E3Z(I) * E1X(I) - E3X(I) * E1Z(I)
         E2Z(I) = E3X(I) * E1Y(I) - E3Y(I) * E1X(I)
       ENDDO 
c-----------
      RETURN
      END
