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
      SUBROUTINE C3FINT3(JFT  ,JLT  ,FOR  ,MOM  ,THK,
     2                   PX1  ,PY1  ,PY2  ,FX1  ,FX2,
     3                   FX3  ,FY1  ,FY2  ,FY3  ,FZ1,
     4                   FZ2  ,FZ3  ,MX1  ,MX2  ,MX3,
     5                   MY1  ,MY2  ,MY3  ,NEL  )
C-----------------------------------------------
C   I m p l i c i t   T y p e s
C-----------------------------------------------
#include      "implicit_f.inc"
C-----------------------------------------------
C   G l o b a l   P a r a m e t e r s
C-----------------------------------------------
#include      "mvsiz_p.inc"
C-----------------------------------------------
C   D u m m y   A r g u m e n t s
C-----------------------------------------------
      INTEGER JFT, JLT, NEL
C     REAL
      my_real
     .   FOR(NEL,5), MOM(NEL,3), THK(*),
     .   PX1(*), PY1(*), PY2(*),
     .   FX1(MVSIZ), FX2(MVSIZ), FX3(MVSIZ),
     .   FY1(MVSIZ), FY2(MVSIZ), FY3(MVSIZ),
     .   FZ1(MVSIZ), FZ2(MVSIZ), FZ3(MVSIZ),
     .   MX1(MVSIZ), MX2(MVSIZ), MX3(MVSIZ),
     .   MY1(MVSIZ), MY2(MVSIZ), MY3(MVSIZ)
C-----------------------------------------------
C   L o c a l   V a r i a b l e s
C-----------------------------------------------
      INTEGER I
C     REAL
      my_real
     .   F1, F2, F3, F4, F5, M1, M2, M3, M4, M5, TH2
C-----------------------------------------------
C
      DO I=JFT,JLT
C
      F1 = FOR(I,1)*THK(I)
      F3 = FOR(I,3)*THK(I)
      FX1(I) =  F1 * PX1(I) + F3 * PY1(I)
      FX2(I) = -F1 * PX1(I) + F3 * PY2(I)
      FX3(I) = -FX1(I) - FX2(I)
C
      F2 = FOR(I,2)*THK(I)
      FY1(I) =  F2 * PY1(I) + F3 * PX1(I)
      FY2(I) =  F2 * PY2(I) - F3 * PX1(I)
      FY3(I) = -FY1(I) - FY2(I)
C
      F4 = FOR(I,4)*THK(I)
      F5 = FOR(I,5)*THK(I)
      FZ1(I) =  F5 * PX1(I) + F4 * PY1(I)
      FZ2(I) = -F5 * PX1(I) + F4 * PY2(I)
      FZ3(I) = -FZ1(I) - FZ2(I)
C
      TH2 = THK(I)*THK(I)
      M2 = MOM(I,2)*TH2
      M3 = MOM(I,3)*TH2
C
      MX1(I) = -M2 * PY1(I) - M3 * PX1(I)
      MX2(I) = -M2 * PY2(I) + M3 * PX1(I)
      MX3(I) = -MX1(I) - MX2(I)
C
      M1 = MOM(I,1)*TH2
      MY1(I) =  M1 * PX1(I) + M3 * PY1(I)
      MY2(I) = -M1 * PX1(I) + M3 * PY2(I)
      MY3(I) = -MY1(I) - MY2(I)
C
      M4 = F4 * THIRD
      M5 = F5 * THIRD
      M5 = M5 * PX1(I)
C
      MY1(I) = MY1(I) + M5 * (TWO*PY1(I)+THREE*PY2(I))
     .                + M4 * PY1(I) * (PY1(I)+TWO*PY2(I))
      MY2(I) = MY2(I) + M5 * (THREE*PY1(I)+TWO*PY2(I))
     .                - M4 * PY2(I) * (TWO*PY1(I)+PY2(I))
      MY3(I) = MY3(I) + M5 * (PY1(I)+PY2(I))
     .                + M4 * (PY2(I)**2 - PY1(I)**2)
C
      M5 = M5 * PX1(I)
      M4 = M4 * PX1(I)
C
      MX1(I) = MX1(I) - M5 - M4 * (TWO*PY1(I)+PY2(I)) 
      MX2(I) = MX2(I) + M5 - M4 * (PY1(I)+TWO*PY2(I)) 
      MX3(I) = MX3(I)      - M4 * THREE*(PY1(I)+PY2(I))       
      ENDDO
C
      RETURN
      END
