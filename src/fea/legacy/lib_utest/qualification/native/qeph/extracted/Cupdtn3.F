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
      SUBROUTINE CUPDTN3(JFT  ,JLT  ,F   ,M   ,NVC  ,
     2                  OFFG ,OFF  ,STI ,STIR,STIFN,
     3                  STIFR,IXC  ,PM  ,AREA ,THK  ,
     4                  F11  ,F12  ,F13 ,F14 ,F21  ,
     5                  F22  ,F23  ,F24 ,F31 ,F32  ,
     6                  F33  ,F34  ,M11 ,M12 ,M13  ,
     7                  M14  ,M21  ,M22 ,M23 ,M24  ,
     8                  M31  ,M32  ,M33 ,M34 ,EINT,
     A                  PARTSAV,MAT,IPARTC,FAC,JTHE,
     B                  THEM ,FTHE ,CONDN,CONDE,NODADT_THERM)
      use element_mod , only : nixc
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
#include      "param_c.inc"
#include      "scr18_c.inc"
C-----------------------------------------------
C   D u m m y   A r g u m e n t s
C-----------------------------------------------
      INTEGER ,INTENT(IN) :: NODADT_THERM
      INTEGER JFT, JLT, NVC,JTHE
      INTEGER IXC(NIXC,MVSIZ),MAT(MVSIZ),IPARTC(*)
C     REAL
      my_real
     .   F(3,*), M(3,*), OFFG(*), OFF(*), STI(*), STIR(*), 
     .   STIFN(*), STIFR(*),PM(NPROPM,*),
     .   F11(MVSIZ), F12(MVSIZ), F13(MVSIZ), F14(MVSIZ),
     .   F21(MVSIZ), F22(MVSIZ), F23(MVSIZ), F24(MVSIZ),
     .   F31(MVSIZ), F32(MVSIZ), F33(MVSIZ), F34(MVSIZ),
     .   M11(MVSIZ), M12(MVSIZ), M13(MVSIZ), M14(MVSIZ),
     .   M21(MVSIZ), M22(MVSIZ), M23(MVSIZ), M24(MVSIZ),
     .   M31(MVSIZ), M32(MVSIZ), M33(MVSIZ), M34(MVSIZ),
     .   EINT(JLT,2),PARTSAV(NPSAV,*),AREA(*) ,THK(*),FAC(MVSIZ,2),
     .   THEM(MVSIZ,4)  ,FTHE(*),CONDN(*),CONDE(*)
CMasParINCLUDE 'cupdt3.intmap.inc'
C-----------------------------------------------
C   L o c a l   V a r i a b l e s
C-----------------------------------------------
      INTEGER NVC1, NVC2, NVC3, NVC4, I
      my_real
     .   OFF_L
C-----------------------------------------------
C
C cumulative energy of deleted elements at the moment of deletion
            
      OFF_L = ZERO
      DO I=JFT,JLT
        IF(OFF(I)<ONE)OFFG(I) = OFF(I)
        OFF_L = MIN(OFF_L,OFFG(I))
      ENDDO
      IF(OFF_L<ZERO)THEN
        DO I=JFT,JLT
         IF(OFFG(I)<ZERO)THEN
           F11(I)=ZERO
           F21(I)=ZERO
           F31(I)=ZERO
           M11(I)=ZERO
           M21(I)=ZERO
           M31(I)=ZERO
           F12(I)=ZERO
           F22(I)=ZERO
           F32(I)=ZERO
           M12(I)=ZERO
           M22(I)=ZERO
           M32(I)=ZERO
           F13(I)=ZERO
           F23(I)=ZERO
           F33(I)=ZERO
           M13(I)=ZERO
           M23(I)=ZERO
           M33(I)=ZERO
           F14(I)=ZERO
           F24(I)=ZERO
           F34(I)=ZERO
           M14(I)=ZERO
           M24(I)=ZERO
           M34(I)=ZERO
           STI(I)=ZERO
           STIR(I)=ZERO
           CONDE(I)=ZERO
         ENDIF
        ENDDO
      ENDIF
C  140---
C
      NVC1= NVC/8
      NVC2=(NVC-NVC1*8)/4
      NVC3=(NVC-NVC1*8-NVC2*4)/2
      NVC4=(NVC-NVC1*8-NVC2*4-NVC3*2)
C
      IF(NVC1==0)THEN
       IF(JTHE == 0 ) THEN
#include "vectorize.inc"
          DO I=JFT,JLT
             F(1,IXC(2,I))=F(1,IXC(2,I))-F11(I)
             F(2,IXC(2,I))=F(2,IXC(2,I))-F21(I)
             F(3,IXC(2,I))=F(3,IXC(2,I))-F31(I)
             M(1,IXC(2,I))=M(1,IXC(2,I))-M11(I)
             M(2,IXC(2,I))=M(2,IXC(2,I))-M21(I)
             M(3,IXC(2,I))=M(3,IXC(2,I))-M31(I)
             STIFN(IXC(2,I))=STIFN(IXC(2,I))+STI(I)*FAC(I,1)
             STIFR(IXC(2,I))=STIFR(IXC(2,I))+STIR(I)*FAC(I,1)
          ENDDO
       ELSE
          IF(NODADT_THERM == 1 ) THEN  
#include "vectorize.inc"
           DO I=JFT,JLT
             F(1,IXC(2,I))=F(1,IXC(2,I))-F11(I)
             F(2,IXC(2,I))=F(2,IXC(2,I))-F21(I)
             F(3,IXC(2,I))=F(3,IXC(2,I))-F31(I)
             M(1,IXC(2,I))=M(1,IXC(2,I))-M11(I)
             M(2,IXC(2,I))=M(2,IXC(2,I))-M21(I)
             M(3,IXC(2,I))=M(3,IXC(2,I))-M31(I)
             STIFN(IXC(2,I))=STIFN(IXC(2,I))+STI(I)*FAC(I,1)
             STIFR(IXC(2,I))=STIFR(IXC(2,I))+STIR(I)*FAC(I,1)
             FTHE(IXC(2,I))=FTHE(IXC(2,I)) + THEM(I,1)
             CONDN(IXC(2,I))=CONDN(IXC(2,I))+CONDE(I)
           ENDDO  
          ELSE
#include "vectorize.inc"
           DO I=JFT,JLT
             F(1,IXC(2,I))=F(1,IXC(2,I))-F11(I)
             F(2,IXC(2,I))=F(2,IXC(2,I))-F21(I)
             F(3,IXC(2,I))=F(3,IXC(2,I))-F31(I)
             M(1,IXC(2,I))=M(1,IXC(2,I))-M11(I)
             M(2,IXC(2,I))=M(2,IXC(2,I))-M21(I)
             M(3,IXC(2,I))=M(3,IXC(2,I))-M31(I)
             STIFN(IXC(2,I))=STIFN(IXC(2,I))+STI(I)*FAC(I,1)
             STIFR(IXC(2,I))=STIFR(IXC(2,I))+STIR(I)*FAC(I,1)
             FTHE(IXC(2,I))=FTHE(IXC(2,I)) + THEM(I,1)
           ENDDO 
          ENDIF       
        ENDIF

      ELSE
C
       IF(JTHE == 0 ) THEN
          DO I=JFT,JLT
            F(1,IXC(2,I))=F(1,IXC(2,I))-F11(I)
            F(2,IXC(2,I))=F(2,IXC(2,I))-F21(I)
            F(3,IXC(2,I))=F(3,IXC(2,I))-F31(I)
            M(1,IXC(2,I))=M(1,IXC(2,I))-M11(I)
            M(2,IXC(2,I))=M(2,IXC(2,I))-M21(I)
            M(3,IXC(2,I))=M(3,IXC(2,I))-M31(I)
            STIFN(IXC(2,I))=STIFN(IXC(2,I))+STI(I)*FAC(I,1)
            STIFR(IXC(2,I))=STIFR(IXC(2,I))+STIR(I)*FAC(I,1)
          ENDDO
       ELSE
          IF(NODADT_THERM == 1 ) THEN  
            DO I=JFT,JLT
              F(1,IXC(2,I))=F(1,IXC(2,I))-F11(I)
              F(2,IXC(2,I))=F(2,IXC(2,I))-F21(I)
              F(3,IXC(2,I))=F(3,IXC(2,I))-F31(I)
              M(1,IXC(2,I))=M(1,IXC(2,I))-M11(I)
              M(2,IXC(2,I))=M(2,IXC(2,I))-M21(I)
              M(3,IXC(2,I))=M(3,IXC(2,I))-M31(I)
              STIFN(IXC(2,I))=STIFN(IXC(2,I))+STI(I)*FAC(I,1)
              STIFR(IXC(2,I))=STIFR(IXC(2,I))+STIR(I)*FAC(I,1)        
              FTHE(IXC(2,I))=FTHE(IXC(2,I)) + THEM(I,1)
              CONDN(IXC(2,I))=CONDN(IXC(2,I))+CONDE(I)
            ENDDO  
          ELSE
            DO I=JFT,JLT
              F(1,IXC(2,I))=F(1,IXC(2,I))-F11(I)
              F(2,IXC(2,I))=F(2,IXC(2,I))-F21(I)
              F(3,IXC(2,I))=F(3,IXC(2,I))-F31(I)
              M(1,IXC(2,I))=M(1,IXC(2,I))-M11(I)
              M(2,IXC(2,I))=M(2,IXC(2,I))-M21(I)
              M(3,IXC(2,I))=M(3,IXC(2,I))-M31(I)
              STIFN(IXC(2,I))=STIFN(IXC(2,I))+STI(I)*FAC(I,1)
              STIFR(IXC(2,I))=STIFR(IXC(2,I))+STIR(I)*FAC(I,1)        
              FTHE(IXC(2,I))=FTHE(IXC(2,I)) + THEM(I,1)
            ENDDO  
          ENDIF       
       ENDIF
      ENDIF
C
      IF(NVC2==0)THEN
       IF(JTHE == 0 ) THEN
#include "vectorize.inc"
         DO I=JFT,JLT
            F(1,IXC(3,I))=F(1,IXC(3,I))-F12(I)
            F(2,IXC(3,I))=F(2,IXC(3,I))-F22(I)
            F(3,IXC(3,I))=F(3,IXC(3,I))-F32(I)
            M(1,IXC(3,I))=M(1,IXC(3,I))-M12(I)
            M(2,IXC(3,I))=M(2,IXC(3,I))-M22(I)
            M(3,IXC(3,I))=M(3,IXC(3,I))-M32(I)
            STIFN(IXC(3,I))=STIFN(IXC(3,I))+STI(I)*FAC(I,2)
            STIFR(IXC(3,I))=STIFR(IXC(3,I))+STIR(I)*FAC(I,2)
         ENDDO  
        ELSE
         IF(NODADT_THERM == 1 ) THEN  
#include "vectorize.inc"
          DO I=JFT,JLT
            F(1,IXC(3,I))=F(1,IXC(3,I))-F12(I)
            F(2,IXC(3,I))=F(2,IXC(3,I))-F22(I)
            F(3,IXC(3,I))=F(3,IXC(3,I))-F32(I)
            M(1,IXC(3,I))=M(1,IXC(3,I))-M12(I)
            M(2,IXC(3,I))=M(2,IXC(3,I))-M22(I)
            M(3,IXC(3,I))=M(3,IXC(3,I))-M32(I)
            STIFN(IXC(3,I))=STIFN(IXC(3,I))+STI(I)*FAC(I,2)
            STIFR(IXC(3,I))=STIFR(IXC(3,I))+STIR(I)*FAC(I,2)        
            FTHE(IXC(3,I))=FTHE(IXC(3,I)) + THEM(I,2) 
            CONDN(IXC(3,I))=CONDN(IXC(3,I))+CONDE(I)         
          ENDDO  
         ELSE
#include "vectorize.inc"
          DO I=JFT,JLT
            F(1,IXC(3,I))=F(1,IXC(3,I))-F12(I)
            F(2,IXC(3,I))=F(2,IXC(3,I))-F22(I)
            F(3,IXC(3,I))=F(3,IXC(3,I))-F32(I)
            M(1,IXC(3,I))=M(1,IXC(3,I))-M12(I)
            M(2,IXC(3,I))=M(2,IXC(3,I))-M22(I)
            M(3,IXC(3,I))=M(3,IXC(3,I))-M32(I)
            STIFN(IXC(3,I))=STIFN(IXC(3,I))+STI(I)*FAC(I,2)
            STIFR(IXC(3,I))=STIFR(IXC(3,I))+STIR(I)*FAC(I,2)        
            FTHE(IXC(3,I))=FTHE(IXC(3,I)) + THEM(I,2) 
          ENDDO  
         ENDIF       
        ENDIF
C   
      ELSE
       IF(JTHE == 0 ) THEN
         DO I=JFT,JLT
            F(1,IXC(3,I))=F(1,IXC(3,I))-F12(I)
            F(2,IXC(3,I))=F(2,IXC(3,I))-F22(I)
            F(3,IXC(3,I))=F(3,IXC(3,I))-F32(I)
            M(1,IXC(3,I))=M(1,IXC(3,I))-M12(I)
            M(2,IXC(3,I))=M(2,IXC(3,I))-M22(I)
            M(3,IXC(3,I))=M(3,IXC(3,I))-M32(I)
            STIFN(IXC(3,I))=STIFN(IXC(3,I))+STI(I)*FAC(I,2)
            STIFR(IXC(3,I))=STIFR(IXC(3,I))+STIR(I)*FAC(I,2)
         ENDDO
        ELSE
         IF(NODADT_THERM == 1 ) THEN  
          DO I=JFT,JLT
            F(1,IXC(3,I))=F(1,IXC(3,I))-F12(I)
            F(2,IXC(3,I))=F(2,IXC(3,I))-F22(I)
            F(3,IXC(3,I))=F(3,IXC(3,I))-F32(I)
            M(1,IXC(3,I))=M(1,IXC(3,I))-M12(I)
            M(2,IXC(3,I))=M(2,IXC(3,I))-M22(I)
            M(3,IXC(3,I))=M(3,IXC(3,I))-M32(I)
            STIFN(IXC(3,I))=STIFN(IXC(3,I))+STI(I)*FAC(I,2)
            STIFR(IXC(3,I))=STIFR(IXC(3,I))+STIR(I)*FAC(I,2)        
            FTHE(IXC(3,I))=FTHE(IXC(3,I)) + THEM(I,2) 
            CONDN(IXC(3,I))=CONDN(IXC(3,I))+CONDE(I)         
          ENDDO  
         ELSE
          DO I=JFT,JLT
            F(1,IXC(3,I))=F(1,IXC(3,I))-F12(I)
            F(2,IXC(3,I))=F(2,IXC(3,I))-F22(I)
            F(3,IXC(3,I))=F(3,IXC(3,I))-F32(I)
            M(1,IXC(3,I))=M(1,IXC(3,I))-M12(I)
            M(2,IXC(3,I))=M(2,IXC(3,I))-M22(I)
            M(3,IXC(3,I))=M(3,IXC(3,I))-M32(I)
            STIFN(IXC(3,I))=STIFN(IXC(3,I))+STI(I)*FAC(I,2)
            STIFR(IXC(3,I))=STIFR(IXC(3,I))+STIR(I)*FAC(I,2)        
            FTHE(IXC(3,I))=FTHE(IXC(3,I)) + THEM(I,2) 
          ENDDO  
         ENDIF        
        ENDIF
C     
      ENDIF
C
      IF(NVC3==0)THEN
        IF(JTHE == 0) THEN
#include "vectorize.inc"
         DO I=JFT,JLT
           F(1,IXC(4,I))=F(1,IXC(4,I))-F13(I)
           F(2,IXC(4,I))=F(2,IXC(4,I))-F23(I)
           F(3,IXC(4,I))=F(3,IXC(4,I))-F33(I)
           M(1,IXC(4,I))=M(1,IXC(4,I))-M13(I)
           M(2,IXC(4,I))=M(2,IXC(4,I))-M23(I)
           M(3,IXC(4,I))=M(3,IXC(4,I))-M33(I)
           STIFN(IXC(4,I))=STIFN(IXC(4,I))+STI(I)*FAC(I,1)
           STIFR(IXC(4,I))=STIFR(IXC(4,I))+STIR(I)*FAC(I,1)
         ENDDO
        ELSE
         IF(NODADT_THERM == 1 ) THEN  
#include "vectorize.inc"
          DO I=JFT,JLT
           F(1,IXC(4,I))=F(1,IXC(4,I))-F13(I)
           F(2,IXC(4,I))=F(2,IXC(4,I))-F23(I)
           F(3,IXC(4,I))=F(3,IXC(4,I))-F33(I)
           M(1,IXC(4,I))=M(1,IXC(4,I))-M13(I)
           M(2,IXC(4,I))=M(2,IXC(4,I))-M23(I)
           M(3,IXC(4,I))=M(3,IXC(4,I))-M33(I)
           STIFN(IXC(4,I))=STIFN(IXC(4,I))+STI(I)*FAC(I,1)
           STIFR(IXC(4,I))=STIFR(IXC(4,I))+STIR(I)*FAC(I,1)        
           FTHE(IXC(4,I))=FTHE(IXC(4,I)) + THEM(I,3)
           CONDN(IXC(4,I))=CONDN(IXC(4,I))+CONDE(I)
          ENDDO 
         ELSE
#include "vectorize.inc"
          DO I=JFT,JLT
           F(1,IXC(4,I))=F(1,IXC(4,I))-F13(I)
           F(2,IXC(4,I))=F(2,IXC(4,I))-F23(I)
           F(3,IXC(4,I))=F(3,IXC(4,I))-F33(I)
           M(1,IXC(4,I))=M(1,IXC(4,I))-M13(I)
           M(2,IXC(4,I))=M(2,IXC(4,I))-M23(I)
           M(3,IXC(4,I))=M(3,IXC(4,I))-M33(I)
           STIFN(IXC(4,I))=STIFN(IXC(4,I))+STI(I)*FAC(I,1)
           STIFR(IXC(4,I))=STIFR(IXC(4,I))+STIR(I)*FAC(I,1)        
           FTHE(IXC(4,I))=FTHE(IXC(4,I)) + THEM(I,3)
          ENDDO
         ENDIF        
        ENDIF
C    
      ELSE
       IF(JTHE == 0 ) THEN
          DO I=JFT,JLT
             F(1,IXC(4,I))=F(1,IXC(4,I))-F13(I)
             F(2,IXC(4,I))=F(2,IXC(4,I))-F23(I)
             F(3,IXC(4,I))=F(3,IXC(4,I))-F33(I)
             M(1,IXC(4,I))=M(1,IXC(4,I))-M13(I)
             M(2,IXC(4,I))=M(2,IXC(4,I))-M23(I)
             M(3,IXC(4,I))=M(3,IXC(4,I))-M33(I)
             STIFN(IXC(4,I))=STIFN(IXC(4,I))+STI(I)*FAC(I,1)
             STIFR(IXC(4,I))=STIFR(IXC(4,I))+STIR(I)*FAC(I,1)
          ENDDO
        ELSE
         IF(NODADT_THERM == 1 ) THEN  
           DO I=JFT,JLT
             F(1,IXC(4,I))=F(1,IXC(4,I))-F13(I)
             F(2,IXC(4,I))=F(2,IXC(4,I))-F23(I)
             F(3,IXC(4,I))=F(3,IXC(4,I))-F33(I)
             M(1,IXC(4,I))=M(1,IXC(4,I))-M13(I)
             M(2,IXC(4,I))=M(2,IXC(4,I))-M23(I)
             M(3,IXC(4,I))=M(3,IXC(4,I))-M33(I)
             STIFN(IXC(4,I))=STIFN(IXC(4,I))+STI(I)*FAC(I,1)
             STIFR(IXC(4,I))=STIFR(IXC(4,I))+STIR(I)*FAC(I,1)        
             FTHE(IXC(4,I))=FTHE(IXC(4,I)) + THEM(I,3)
             CONDN(IXC(4,I))=CONDN(IXC(4,I))+CONDE(I)   
           ENDDO
         ELSE
           DO I=JFT,JLT
             F(1,IXC(4,I))=F(1,IXC(4,I))-F13(I)
             F(2,IXC(4,I))=F(2,IXC(4,I))-F23(I)
             F(3,IXC(4,I))=F(3,IXC(4,I))-F33(I)
             M(1,IXC(4,I))=M(1,IXC(4,I))-M13(I)
             M(2,IXC(4,I))=M(2,IXC(4,I))-M23(I)
             M(3,IXC(4,I))=M(3,IXC(4,I))-M33(I)
             STIFN(IXC(4,I))=STIFN(IXC(4,I))+STI(I)*FAC(I,1)
             STIFR(IXC(4,I))=STIFR(IXC(4,I))+STIR(I)*FAC(I,1)        
             FTHE(IXC(4,I))=FTHE(IXC(4,I)) + THEM(I,3)
           ENDDO
         ENDIF         
        ENDIF
C       
      ENDIF
C
      IF(NVC4==0)THEN
       IF(JTHE == 0) THEN
#include "vectorize.inc"
           DO I=JFT,JLT
              F(1,IXC(5,I))=F(1,IXC(5,I))-F14(I)
              F(2,IXC(5,I))=F(2,IXC(5,I))-F24(I)
              F(3,IXC(5,I))=F(3,IXC(5,I))-F34(I)
              M(1,IXC(5,I))=M(1,IXC(5,I))-M14(I)
              M(2,IXC(5,I))=M(2,IXC(5,I))-M24(I)
              M(3,IXC(5,I))=M(3,IXC(5,I))-M34(I)
              STIFN(IXC(5,I))=STIFN(IXC(5,I))+STI(I)*FAC(I,2)
              STIFR(IXC(5,I))=STIFR(IXC(5,I))+STIR(I)*FAC(I,2)
           ENDDO
         ELSE
         IF(NODADT_THERM == 1 ) THEN  
#include "vectorize.inc"
           DO I=JFT,JLT
            F(1,IXC(5,I))=F(1,IXC(5,I))-F14(I)
            F(2,IXC(5,I))=F(2,IXC(5,I))-F24(I)
            F(3,IXC(5,I))=F(3,IXC(5,I))-F34(I)
            M(1,IXC(5,I))=M(1,IXC(5,I))-M14(I)
            M(2,IXC(5,I))=M(2,IXC(5,I))-M24(I)
            M(3,IXC(5,I))=M(3,IXC(5,I))-M34(I)
            STIFN(IXC(5,I))=STIFN(IXC(5,I))+STI(I)*FAC(I,2)
            STIFR(IXC(5,I))=STIFR(IXC(5,I))+STIR(I)*FAC(I,2)        
            FTHE(IXC(5,I))=FTHE(IXC(5,I)) + THEM(I,4)
            CONDN(IXC(5,I))=CONDN(IXC(5,I))+CONDE(I) 
           ENDDO
         ELSE
#include "vectorize.inc"
           DO I=JFT,JLT
            F(1,IXC(5,I))=F(1,IXC(5,I))-F14(I)
            F(2,IXC(5,I))=F(2,IXC(5,I))-F24(I)
            F(3,IXC(5,I))=F(3,IXC(5,I))-F34(I)
            M(1,IXC(5,I))=M(1,IXC(5,I))-M14(I)
            M(2,IXC(5,I))=M(2,IXC(5,I))-M24(I)
            M(3,IXC(5,I))=M(3,IXC(5,I))-M34(I)
            STIFN(IXC(5,I))=STIFN(IXC(5,I))+STI(I)*FAC(I,2)
            STIFR(IXC(5,I))=STIFR(IXC(5,I))+STIR(I)*FAC(I,2)        
            FTHE(IXC(5,I))=FTHE(IXC(5,I)) + THEM(I,4)
           ENDDO
         ENDIF         
       
        ENDIF
C     
      ELSE
       IF(JTHE == 0) THEN
         DO I=JFT,JLT
            F(1,IXC(5,I))=F(1,IXC(5,I))-F14(I)
            F(2,IXC(5,I))=F(2,IXC(5,I))-F24(I)
            F(3,IXC(5,I))=F(3,IXC(5,I))-F34(I)
            M(1,IXC(5,I))=M(1,IXC(5,I))-M14(I)
            M(2,IXC(5,I))=M(2,IXC(5,I))-M24(I)
            M(3,IXC(5,I))=M(3,IXC(5,I))-M34(I)
            STIFN(IXC(5,I))=STIFN(IXC(5,I))+STI(I)*FAC(I,2)
            STIFR(IXC(5,I))=STIFR(IXC(5,I))+STIR(I)*FAC(I,2)
         ENDDO
        ELSE
         IF(NODADT_THERM == 1 ) THEN  
          DO I=JFT,JLT
            F(1,IXC(5,I))=F(1,IXC(5,I))-F14(I)
            F(2,IXC(5,I))=F(2,IXC(5,I))-F24(I)
            F(3,IXC(5,I))=F(3,IXC(5,I))-F34(I)
            M(1,IXC(5,I))=M(1,IXC(5,I))-M14(I)
            M(2,IXC(5,I))=M(2,IXC(5,I))-M24(I)
            M(3,IXC(5,I))=M(3,IXC(5,I))-M34(I)
            STIFN(IXC(5,I))=STIFN(IXC(5,I))+STI(I)*FAC(I,2)
            STIFR(IXC(5,I))=STIFR(IXC(5,I))+STIR(I)*FAC(I,2)        
            FTHE(IXC(5,I))=FTHE(IXC(5,I)) + THEM(I,4)
            CONDN(IXC(5,I))=CONDN(IXC(5,I))+CONDE(I)
          ENDDO
         ELSE
          DO I=JFT,JLT
            F(1,IXC(5,I))=F(1,IXC(5,I))-F14(I)
            F(2,IXC(5,I))=F(2,IXC(5,I))-F24(I)
            F(3,IXC(5,I))=F(3,IXC(5,I))-F34(I)
            M(1,IXC(5,I))=M(1,IXC(5,I))-M14(I)
            M(2,IXC(5,I))=M(2,IXC(5,I))-M24(I)
            M(3,IXC(5,I))=M(3,IXC(5,I))-M34(I)
            STIFN(IXC(5,I))=STIFN(IXC(5,I))+STI(I)*FAC(I,2)
            STIFR(IXC(5,I))=STIFR(IXC(5,I))+STIR(I)*FAC(I,2)        
            FTHE(IXC(5,I))=FTHE(IXC(5,I)) + THEM(I,4)
          ENDDO
         ENDIF         
        ENDIF
C        
      ENDIF
C
      RETURN
      END
