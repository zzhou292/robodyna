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
      SUBROUTINE C3SROTO3(JFT  ,JLT  ,ECOS ,ESIN      ,FOR,
     .                    MOM  ,NFOR ,NMOM ,IFRAM_OLD ,NEL)
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
      INTEGER JFT, JLT,IFRAM_OLD,NEL
C     REAL
      my_real
     .   FOR(NEL,5), MOM(NEL,3),NFOR(NEL,5), NMOM(NEL,3),
     .   ECOS(MVSIZ),ESIN(MVSIZ)
C-----------------------------------------------
C   L o c a l   V a r i a b l e s
C-----------------------------------------------
      INTEGER I
C     REAL
      my_real
     .  M2(MVSIZ),N2(MVSIZ),MN(MVSIZ),MN2(MVSIZ),T1
C-----------------------------------------------
      IF(IFRAM_OLD ==0 ) THEN
       DO I=JFT,JLT
        M2(I)= ECOS(I)*ECOS(I)
        N2(I)= ESIN(I)*ESIN(I)
        MN(I)= ECOS(I)*ESIN(I)
        MN2(I)= TWO*MN(I)
       ENDDO
C------
       DO I=JFT,JLT
        T1 = FOR(I,3)*MN2(I)
        NFOR(I,1)=M2(I)*FOR(I,1)+N2(I)*FOR(I,2)+T1
        NFOR(I,2)=N2(I)*FOR(I,1)+M2(I)*FOR(I,2)-T1
        NFOR(I,3)=(-FOR(I,1)+FOR(I,2))*MN(I)+FOR(I,3)*(M2(I)-N2(I))
        NFOR(I,4)=ECOS(I)*FOR(I,4)-ESIN(I)*FOR(I,5)
        NFOR(I,5)=ECOS(I)*FOR(I,5)+ESIN(I)*FOR(I,4)
       ENDDO
       DO I=JFT,JLT
        T1 = MOM(I,3)*MN2(I)
        NMOM(I,1)=M2(I)*MOM(I,1)+N2(I)*MOM(I,2)+T1
        NMOM(I,2)=N2(I)*MOM(I,1)+M2(I)*MOM(I,2)-T1
        NMOM(I,3)=(-MOM(I,1)+MOM(I,2))*MN(I)+MOM(I,3)*(M2(I)-N2(I))
       ENDDO
      ELSE
       DO I=JFT,JLT
        NFOR(I,1) = FOR(I,1)
        NFOR(I,2) = FOR(I,2)
        NFOR(I,3) = FOR(I,3)
        NFOR(I,4) = FOR(I,4)
        NFOR(I,5) = FOR(I,5)
        NMOM(I,1) = MOM(I,1)
        NMOM(I,2) = MOM(I,2)
        NMOM(I,3) = MOM(I,3)
       ENDDO
      END IF
C
      RETURN
      END
