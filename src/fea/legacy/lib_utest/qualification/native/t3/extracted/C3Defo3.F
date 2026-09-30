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

! Complete pinned routine, extracted without arithmetic edits.
      SUBROUTINE C3DEFO3(JFT,JLT,VL1,VL2,VL3,
     .                   IXTG,ISH3N,PX1  ,PY1    ,PY2   ,
     .                   EXX  ,EYY  ,EXY  ,EYZ   ,EZX   ,
     .                   VX13 ,VX23  ,VY12  ,
     .                   E1X      ,E1Y      ,E1Z     ,E2X      ,           
     .                   E2Y      ,E2Z      ,E3X     ,E3Y      ,E3Z      ) 
      use element_mod , only : nixtg
C-----------------------------------------------
C   I m p l i c i t   T y p e s
C-----------------------------------------------
#include      "implicit_f.inc"
C-----------------------------------------------
C   G l o b a l   P a r a m e t e r s
C-----------------------------------------------
#include      "mvsiz_p.inc"
C-----------------------------------------------
C   C o m m o n   B l o c k s
C-----------------------------------------------
#include      "com08_c.inc"
#include      "impl1_c.inc"
C-----------------------------------------------
C   D u m m y   A r g u m e n t s
C-----------------------------------------------
      INTEGER JFT, JLT,IXTG(NIXTG,*), ISH3N
      my_real VL1(MVSIZ,3),VL2(MVSIZ,3),VL3(MVSIZ,3),
     .     EXX(MVSIZ), EYY(MVSIZ), EXY(MVSIZ), EYZ(MVSIZ),EZX(MVSIZ), 
     .     E1X(MVSIZ), E1Y(MVSIZ), E1Z(MVSIZ),
     .     E2X(MVSIZ), E2Y(MVSIZ), E2Z(MVSIZ),
     .     E3X(MVSIZ), E3Y(MVSIZ), E3Z(MVSIZ),
     .     PX1(MVSIZ), PY1(MVSIZ), PY2(MVSIZ), VX13(MVSIZ), VX23(MVSIZ),
     .     VY12(MVSIZ)
C-----------------------------------------------
C   L o c a l   V a r i a b l e s
C-----------------------------------------------
      INTEGER I
      my_real
     .     VX1(MVSIZ),  VX2(MVSIZ),  VX3(MVSIZ),
     .     VY1(MVSIZ), VY2(MVSIZ), VY3(MVSIZ),
     .     VZ1(MVSIZ), VZ2(MVSIZ), VZ3(MVSIZ),
     .     VX12(MVSIZ), VY13(MVSIZ), VY23(MVSIZ),
     .     VZ12(MVSIZ), VZ13(MVSIZ), VZ23(MVSIZ)
      my_real  DT1V4B,TMP11,TMP22,VX10,VX20,VX30,DT1V4,TMP1,TMP2
C=======================================================================
C
      DO I=JFT,JLT
        VX1(I)=E1X(I)*VL1(I,1)+E1Y(I)*VL1(I,2)+E1Z(I)*VL1(I,3)
        VX2(I)=E1X(I)*VL2(I,1)+E1Y(I)*VL2(I,2)+E1Z(I)*VL2(I,3)
        VX3(I)=E1X(I)*VL3(I,1)+E1Y(I)*VL3(I,2)+E1Z(I)*VL3(I,3)
C
        VY3(I)=E2X(I)*VL3(I,1)+E2Y(I)*VL3(I,2)+E2Z(I)*VL3(I,3)
        VY2(I)=E2X(I)*VL2(I,1)+E2Y(I)*VL2(I,2)+E2Z(I)*VL2(I,3)
        VY1(I)=E2X(I)*VL1(I,1)+E2Y(I)*VL1(I,2)+E2Z(I)*VL1(I,3)
C
        VZ1(I)=E3X(I)*VL1(I,1)+E3Y(I)*VL1(I,2)+E3Z(I)*VL1(I,3)
        VZ2(I)=E3X(I)*VL2(I,1)+E3Y(I)*VL2(I,2)+E3Z(I)*VL2(I,3)
        VZ3(I)=E3X(I)*VL3(I,1)+E3Y(I)*VL3(I,2)+E3Z(I)*VL3(I,3)
      ENDDO
c
      DT1V4 = FOURTH*DT1
      IF (IMPL_S > 0 .AND. IMP_LR == 0 ) DT1V4=ZERO
      DT1V4B= DT1V4
      IF(ISH3N < 2) DT1V4B=ZERO
C
      DO 40 I=JFT,JLT
      VZ12(I)=VZ1(I) - VZ2(I)
      VZ13(I)=VZ1(I) - VZ3(I)
      VZ23(I)=VZ2(I) - VZ3(I)
C
      TMP1 = DT1V4 * VZ12(I) / (PY1(I)+PY2(I))
      TMP2 = (PY1(I) * VZ1(I) + PY2(I) * VZ2(I)) / (PY1(I)+PY2(I))
      TMP2 = DT1V4 * (TMP2 - VZ3(I)) / PX1(I)
      VY12(I)=VY1(I) - VY2(I)
      TMP11 = DT1V4B * VY12(I) / (PY1(I)+PY2(I))
      TMP22 = (PY1(I) * VX1(I) + PY2(I) * VX2(I)) / (PY1(I)+PY2(I))
      TMP22 = DT1V4B * (TMP22 - VX3(I)) / PX1(I)
      VX10 = VX1(I)
      VX20 = VX2(I)
      VX30 = VX3(I)
      VX1(I) = VX1(I) - VZ1(I) * TMP1 - VY1(I) * TMP11 
      VX2(I) = VX2(I) - VZ2(I) * TMP1 - VY2(I) * TMP11
      VX3(I) = VX3(I) - VZ3(I) * TMP1 - VY3(I) * TMP11 
      VY1(I) = VY1(I) - VZ1(I) * TMP2 - VX10 * TMP22 
      VY2(I) = VY2(I) - VZ2(I) * TMP2 - VX20 * TMP22 
      VY3(I) = VY3(I) - VZ3(I) * TMP2 - VX30 * TMP22 
C
      VX12(I)=VX1(I) - VX2(I)
      VY12(I)=VY1(I) - VY2(I)
      VX13(I)=VX1(I) - VX3(I)
      VY13(I)=VY1(I) - VY3(I)
      VX23(I)=VX2(I) - VX3(I)
      VY23(I)=VY2(I) - VY3(I)
C
      EXX(I)=PX1(I)*VX12(I)
      EYY(I)=PY1(I)*VY13(I) + PY2(I)*VY23(I)
C
      EXY(I)=PY1(I)*VX13(I) + PY2(I)*VX23(I) + PX1(I)*VY12(I)
      EYZ(I)=PY1(I)*VZ13(I) + PY2(I)*VZ23(I)
      EZX(I)=PX1(I)*VZ12(I)
   40 CONTINUE
C-----------
      RETURN
      END
