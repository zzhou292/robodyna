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
      SUBROUTINE C3COOR3(JFT      ,JLT      ,X       ,IXTG     ,
     .                   OFFG     ,OFF      ,DT1C    ,
     .                   V        ,VR       ,VL1     ,VL2      ,VL3      ,
     .                   VRL1     ,VRL2     ,VRL3    ,SIGY     ,
     .                   X1       ,X2       ,X3      ,Y1       ,Y2       ,
     .                   Y3       ,Z1       ,Z2      ,Z3       ,XDP      )
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
#include      "scr05_c.inc"
C-----------------------------------------------
C   D u m m y   A r g u m e n t s
C-----------------------------------------------
      INTEGER JFT, JLT
      INTEGER IXTG(NIXTG,*)
      my_real
     .   OFFG(*), OFF(*),DT1C(*),X(3,*),
     .   V(3,*),VR(3,*),VL1(MVSIZ,3),VL2(MVSIZ,3),VL3(MVSIZ,3),
     .   VRL1(MVSIZ,3),VRL2(MVSIZ,3),VRL3(MVSIZ,3),SIGY(*)
!       SP issue :
      REAL(kind=8), DIMENSION(3,*), INTENT(in) :: XDP
      REAL(kind=8), DIMENSION(MVSIZ), INTENT(out) ::X1,X2,X3
      REAL(kind=8), DIMENSION(MVSIZ), INTENT(out) ::Y1,Y2,Y3
      REAL(kind=8), DIMENSION(MVSIZ), INTENT(out) ::Z1,Z2,Z3


C-----------------------------------------------
C   L o c a l   V a r i a b l e s
C-----------------------------------------------
      INTEGER I, NC1, NC2, NC3
      my_real OFF_L    
C=======================================================================
      IF(IRESP == 1)THEN
       DO I=JFT,JLT
        NC1     = IXTG(2,I) 
        NC2     = IXTG(3,I) 
        NC3     = IXTG(4,I) 
C----------------------------
C       COORDONNEES
C----------------------------
        X1(I)=XDP(1,NC1)
        Y1(I)=XDP(2,NC1)
        Z1(I)=XDP(3,NC1)
        X2(I)=XDP(1,NC2)
        Y2(I)=XDP(2,NC2)
        Z2(I)=XDP(3,NC2)
        X3(I)=XDP(1,NC3)
        Y3(I)=XDP(2,NC3)
        Z3(I)=XDP(3,NC3)
        VL1(I,1)=V(1,NC1)
        VL1(I,2)=V(2,NC1)
        VL1(I,3)=V(3,NC1)
        VL2(I,1)=V(1,NC2)
        VL2(I,2)=V(2,NC2)
        VL2(I,3)=V(3,NC2)
        VL3(I,1)=V(1,NC3)
        VL3(I,2)=V(2,NC3)
        VL3(I,3)=V(3,NC3)
        VRL1(I,1)=VR(1,NC1)
        VRL1(I,2)=VR(2,NC1)
        VRL1(I,3)=VR(3,NC1)
        VRL2(I,1)=VR(1,NC2)
        VRL2(I,2)=VR(2,NC2)
        VRL2(I,3)=VR(3,NC2)
        VRL3(I,1)=VR(1,NC3)
        VRL3(I,2)=VR(2,NC3)
        VRL3(I,3)=VR(3,NC3)
       ENDDO
      ELSE
       DO I=JFT,JLT
        NC1     = IXTG(2,I) 
        NC2     = IXTG(3,I) 
        NC3     = IXTG(4,I) 
C----------------------------
C       COORDONNEES
C----------------------------
        X1(I)=X(1,NC1)
        Y1(I)=X(2,NC1)
        Z1(I)=X(3,NC1)
        X2(I)=X(1,NC2)
        Y2(I)=X(2,NC2)
        Z2(I)=X(3,NC2)
        X3(I)=X(1,NC3)
        Y3(I)=X(2,NC3)
        Z3(I)=X(3,NC3)
        VL1(I,1)=V(1,NC1)
        VL1(I,2)=V(2,NC1)
        VL1(I,3)=V(3,NC1)
        VL2(I,1)=V(1,NC2)
        VL2(I,2)=V(2,NC2)
        VL2(I,3)=V(3,NC2)
        VL3(I,1)=V(1,NC3)
        VL3(I,2)=V(2,NC3)
        VL3(I,3)=V(3,NC3)
        VRL1(I,1)=VR(1,NC1)
        VRL1(I,2)=VR(2,NC1)
        VRL1(I,3)=VR(3,NC1)
        VRL2(I,1)=VR(1,NC2)
        VRL2(I,2)=VR(2,NC2)
        VRL2(I,3)=VR(3,NC2)
        VRL3(I,1)=VR(1,NC3)
        VRL3(I,2)=VR(2,NC3)
        VRL3(I,3)=VR(3,NC3)
       ENDDO
      END IF!(IRESP == 1)THEN
C
      OFF_L = ZERO
      DO I=JFT,JLT
        DT1C(I) = DT1
        OFF(I) = MIN(ONE,ABS(OFFG(I)))
        OFF_L  = MIN(OFF_L,OFFG(I))
        SIGY(I) = EP30
      ENDDO
      IF (OFF_L < ZERO) THEN
        DO I=JFT,JLT
         IF (OFFG(I) < ZERO) THEN
           VL1(I,1)=ZERO
           VL1(I,2)=ZERO
           VL1(I,3)=ZERO
           VL2(I,1)=ZERO
           VL2(I,2)=ZERO
           VL2(I,3)=ZERO
           VL3(I,1)=ZERO
           VL3(I,2)=ZERO
           VL3(I,3)=ZERO
           VRL1(I,1)=ZERO
           VRL1(I,2)=ZERO
           VRL1(I,3)=ZERO
           VRL2(I,1)=ZERO
           VRL2(I,2)=ZERO
           VRL2(I,3)=ZERO
           VRL3(I,1)=ZERO
           VRL3(I,2)=ZERO
           VRL3(I,3)=ZERO
         ENDIF
        ENDDO
      ENDIF
C-----------
      RETURN
      END
