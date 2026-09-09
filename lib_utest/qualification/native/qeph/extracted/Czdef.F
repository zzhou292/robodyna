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
        SUBROUTINE CZDEF(JFT ,JLT  ,AREA ,AA  ,V13   ,V24 ,
     2                   VHI ,RLXYZ,VDEF ,VHG ,X13   ,
     3                   X24 ,Y13  ,Y24  ,MX13,MX23  ,MX34,
     4                   MY13,MY23 ,MY34 ,Z1  ,DT1   , OFF, 
     5                   OFFG,RLXYZV )
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
#include      "implicit_f.inc"
#include      "mvsiz_p.inc" 
#include      "parit_c.inc"
C-----------------------------------------------
C   D U M M Y   A R G U M E N T S
C-----------------------------------------------
      INTEGER JFT,JLT
      my_real 
     .   RLXYZ(MVSIZ,2,4),V13(MVSIZ,3),V24(MVSIZ,3),
     .   X13(*),X24(*),Y13(*),Y24(*),
     .   MX13(*),MX23(*),MX34(*),MY13(*),MY23(*),MY34(*)
      my_real 
     .   AREA(*),Z1(*),VDEF(MVSIZ,8),
     .   AA(*),VHG(MVSIZ,6),VHI(MVSIZ,3),DT1,OFF(*),OFFG(*),OFF_L
      my_real
     .  RLXYZV(MVSIZ,2,4)
C-----------------------------------------------
C   L O C A L   V A R I A B L E S
C-----------------------------------------------
      INTEGER I
      my_real 
     .   BCX,BCY,BCXY,R13(2),R24(2),DETA1,
     .   RSOM(2),RHI(2),
     .   BXV2,BYV1,BXR1,BYR2
C-----------------------------------------------
      IF (IVECTOR==1) THEN
       DO I=JFT,JLT
        R13(1) =(RLXYZV(I,1,1)-RLXYZV(I,1,3))*AA(I)
        R24(1) =(RLXYZV(I,1,2)-RLXYZV(I,1,4))*AA(I)
        RSOM(1)=(RLXYZV(I,1,4)+RLXYZV(I,1,3)+RLXYZV(I,1,1)
     .         +RLXYZV(I,1,2))*AA(I)
        RHI(1) =(RLXYZV(I,1,1)-RLXYZV(I,1,2)+RLXYZV(I,1,3)
     .         -RLXYZV(I,1,4))*FOURTH
        R13(2) =(RLXYZV(I,2,1)-RLXYZV(I,2,3))*AA(I)
        R24(2) =(RLXYZV(I,2,2)-RLXYZV(I,2,4))*AA(I)
        RSOM(2)=(RLXYZV(I,2,4)+RLXYZV(I,2,3)+RLXYZV(I,2,1)
     .         +RLXYZV(I,2,2))*AA(I)
        RHI(2) =(RLXYZV(I,2,1)-RLXYZV(I,2,2)+RLXYZV(I,2,3)
     .         - RLXYZV(I,2,4))*FOURTH
C---------------
C  MEMBRANE
C---------------
        VDEF(I,1)= Y24(I)*V13(I,1)-Y13(I)*V24(I,1)
        VDEF(I,2)=-X24(I)*V13(I,2)+X13(I)*V24(I,2)
        BXV2= Y24(I)*V13(I,2)-Y13(I)*V24(I,2)
        BYV1=-X24(I)*V13(I,1)+X13(I)*V24(I,1)
        VDEF(I,3)=BXV2+BYV1
C---------
C  FLEXION
C---------
        VDEF(I,6)=Y24(I)*R13(2)-Y13(I)*R24(2)
        VDEF(I,7)=X24(I)*R13(1)-X13(I)*R24(1)
        BXR1= Y13(I)*R24(1)-Y24(I)*R13(1)
        BYR2=-X24(I)*R13(2)+X13(I)*R24(2)
        VDEF(I,8)=BXR1+BYR2
C--------
C  C.T
C--------
        BCXY=AREA(I)*FOURTH
        BCX=V13(I,3)-MY13(I)*R13(1)+MX13(I)*R13(2)
        BCY=V24(I,3)+MY13(I)*R24(1)-MX13(I)*R24(2)
        VDEF(I,4)=Y24(I)*BCX-Y13(I)*BCY+BCXY*RSOM(2)
        VDEF(I,5)=X13(I)*BCY-X24(I)*BCX-BCXY*RSOM(1)
C----------------------------------
C  hourglass strain rate
C----------------------------------
        VHG(I,1)=VHI(I,1)-MX13(I)*VDEF(I,1)-MY13(I)*BYV1     
        VHG(I,2)=VHI(I,2)-MX13(I)*BXV2     -MY13(I)*VDEF(I,2)
        VHG(I,3)= RHI(2)-MX13(I)*VDEF(I,6)-MY13(I)*BYR2     
        VHG(I,4)=-RHI(1)-MX13(I)*BXR1     -MY13(I)*VDEF(I,7)
        VHG(I,5)=(VHI(I,3)*4.-(MY13(I)*RSOM(1)-MY23(I)*(R13(1)+R24(1))
     1         +MX23(I)*(R13(2)+R24(2))-MX13(I)*RSOM(2))*AREA(I))*FOUR
        VHG(I,6)=(VHI(I,3)*4.-(MY13(I)*RSOM(1)-MY34(I)*(R13(1)-R24(1))
     1         +MX34(I)*(R13(2)-R24(2))-MX13(I)*RSOM(2))*AREA(I))*FOUR
C 
        VHG(I,1)=VHG(I,1)+( Y24(I)*V13(I,3)-Y13(I)*V24(I,3))*Z1(I)
        VHG(I,2)=VHG(I,2)+(-X24(I)*V13(I,3)+X13(I)*V24(I,3))*Z1(I)
       ENDDO
      ELSE
      DO I=JFT,JLT
        R13(1) =(RLXYZ(I,1,1)-RLXYZ(I,1,3))*AA(I)
        R24(1) =(RLXYZ(I,1,2)-RLXYZ(I,1,4))*AA(I)
        RSOM(1)=(RLXYZ(I,1,4)+RLXYZ(I,1,3)+RLXYZ(I,1,1)+RLXYZ(I,1,2))
     .         *AA(I)
        RHI(1) =(RLXYZ(I,1,1)-RLXYZ(I,1,2)+RLXYZ(I,1,3)-RLXYZ(I,1,4))
     .         *FOURTH
        R13(2) =(RLXYZ(I,2,1)-RLXYZ(I,2,3))*AA(I)
        R24(2) =(RLXYZ(I,2,2)-RLXYZ(I,2,4))*AA(I)
        RSOM(2)=(RLXYZ(I,2,4)+RLXYZ(I,2,3)+RLXYZ(I,2,1)+RLXYZ(I,2,2))
     .         *AA(I)
        RHI(2) =(RLXYZ(I,2,1)-RLXYZ(I,2,2)+RLXYZ(I,2,3)-RLXYZ(I,2,4))
     .         *FOURTH
C---------------
C  MEMBRANE
C---------------
        VDEF(I,1)= Y24(I)*V13(I,1)-Y13(I)*V24(I,1)
        VDEF(I,2)=-X24(I)*V13(I,2)+X13(I)*V24(I,2)
        BXV2= Y24(I)*V13(I,2)-Y13(I)*V24(I,2)
        BYV1=-X24(I)*V13(I,1)+X13(I)*V24(I,1)
        VDEF(I,3)=BXV2+BYV1
C---------
C  FLEXION
C---------
        VDEF(I,6)=Y24(I)*R13(2)-Y13(I)*R24(2)
        VDEF(I,7)=X24(I)*R13(1)-X13(I)*R24(1)
        BXR1= Y13(I)*R24(1)-Y24(I)*R13(1)
        BYR2=-X24(I)*R13(2)+X13(I)*R24(2)
        VDEF(I,8)=BXR1+BYR2
C--------
C  C.T
C--------
        BCXY=AREA(I)*FOURTH
        BCX=V13(I,3)-MY13(I)*R13(1)+MX13(I)*R13(2)
        BCY=V24(I,3)+MY13(I)*R24(1)-MX13(I)*R24(2)
        VDEF(I,4)=Y24(I)*BCX-Y13(I)*BCY+BCXY*RSOM(2)
        VDEF(I,5)=X13(I)*BCY-X24(I)*BCX-BCXY*RSOM(1)
C----------------------------------
C  hourglass strain rate
C----------------------------------
        VHG(I,1)=VHI(I,1)-MX13(I)*VDEF(I,1)-MY13(I)*BYV1     
        VHG(I,2)=VHI(I,2)-MX13(I)*BXV2     -MY13(I)*VDEF(I,2)
        VHG(I,3)= RHI(2)-MX13(I)*VDEF(I,6)-MY13(I)*BYR2     
        VHG(I,4)=-RHI(1)-MX13(I)*BXR1     -MY13(I)*VDEF(I,7)
        VHG(I,5)=(VHI(I,3)*4.-(MY13(I)*RSOM(1)-MY23(I)*(R13(1)+R24(1))
     1         +MX23(I)*(R13(2)+R24(2))-MX13(I)*RSOM(2))*AREA(I))*FOUR
        VHG(I,6)=(VHI(I,3)*4.-(MY13(I)*RSOM(1)-MY34(I)*(R13(1)-R24(1))
     1         +MX34(I)*(R13(2)-R24(2))-MX13(I)*RSOM(2))*AREA(I))*FOUR
C 
        VHG(I,1)=VHG(I,1)+( Y24(I)*V13(I,3)-Y13(I)*V24(I,3))*Z1(I)
        VHG(I,2)=VHG(I,2)+(-X24(I)*V13(I,3)+X13(I)*V24(I,3))*Z1(I)
      ENDDO
      ENDIF
      DO I=JFT,JLT
       DETA1=Z1(I)*FOUR*AA(I)
       VDEF(I,6)=VDEF(I,6)+(X13(I)*V13(I,1)-X24(I)*V24(I,1))*DETA1
       VDEF(I,7)=VDEF(I,7)+(Y13(I)*V13(I,2)-Y24(I)*V24(I,2))*DETA1
       VDEF(I,8)=VDEF(I,8)+(X13(I)*V13(I,2)-X24(I)*V24(I,2)+
     1                      Y13(I)*V13(I,1)-Y24(I)*V24(I,1))*DETA1
      ENDDO
      OFF_L = ZERO
      DO I=JFT,JLT
        OFF(I) = MIN(ONE,ABS(OFFG(I)))
        OFF_L  = MIN(OFF_L,OFFG(I))
      ENDDO
      IF(OFF_L<ZERO)THEN
        DO I=JFT,JLT
         IF(OFFG(I)<ZERO)THEN
          VDEF(I,1)=ZERO
          VDEF(I,2)=ZERO
          VDEF(I,3)=ZERO
          VDEF(I,4)=ZERO
          VDEF(I,5)=ZERO
          VDEF(I,6)=ZERO
          VDEF(I,7)=ZERO
          VDEF(I,8)=ZERO
C
          VHG(I,1)=ZERO    
          VHG(I,2)=ZERO
          VHG(I,3)=ZERO     
          VHG(I,4)=ZERO
          VHG(I,5)=ZERO
          VHG(I,6)=ZERO  
         ENDIF
        ENDDO
      ENDIF
C
      RETURN
      END
