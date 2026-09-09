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
      SUBROUTINE CNCOEF3B(JFT    ,JLT     ,PM     ,MAT    ,GEO    ,
     2                    PID    ,AREA    ,SHF    ,THK0   ,
     3                    THK02  ,NU      ,G      ,YM     ,
     4                    A11    ,A12     ,THK    ,THKE   ,SSP    ,
     5                    RHO    ,VOLG    ,GS     ,MTN    ,ITHK   ,
     6                    NPT    ,DT1C    ,DT1    ,IHBE   ,AMU    ,
     7                    GSR    ,A11SR   ,A12SR  ,NUSR   ,SHFSR  ,
     8                    KRZ    ,IGEO    ,A11R  ,ISUBSTACK, PM_STACK,
     9                    UPARAM ,DIRA    ,DIRB   ,UVAR   ,FAC58  ,
     A                    NEL    ,ZOFFSET )
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
#include      "impl1_c.inc"
#include      "impl2_c.inc"
C-----------------------------------------------
C   D u m m y   A r g u m e n t s
C-----------------------------------------------
      INTEGER JFT, JLT,MTN,ITHK,NPT,IHBE,ISUBSTACK
      INTEGER , INTENT(IN) :: NEL
      INTEGER MAT(*), PID(*), IGEO(NPROPGI,*)
      my_real
     .   GEO(NPROPG,*), PM(NPROPM,*), AREA(*),
     .   SHF(*),THK0(*),THK02(*),THK(*),THKE(*),
     .   NU(*),G(*),YM(*),A11(*),A12(*),AMU(*),
     .   VOLG(*),SSP(*),RHO(*),GS(*),DT1C(*),DT1,
     .   GSR(*), A11SR(*), A12SR(*), NUSR(*), SHFSR(*),KRZ(*),
     .   A11R(*),PM_STACK(20,*),UPARAM(*),
     .   DIRA(JLT,*),DIRB(JLT,*),UVAR(JLT,*),FAC58(MVSIZ,2)
       my_real , INTENT(OUT) , DIMENSION(NEL):: ZOFFSET 
C-----------------------------------------------
C   L o c a l   V a r i a b l e s
C-----------------------------------------------
      INTEGER I,MX,IPID,IGTYP,IPGMAT,IGMAT,IPOS
      my_real FAC1TMP,KFAC,DN,K58(3),
     . RFAC,RFAT,R1,R2,S1,S2,T1,T2,T3,RS1,RS2,
     . R12,S12,R22,S22,E11,E22,K58I, Z0
C-----------------------------------------------
      IF(ITHK>0.AND.ISMDISP==0)THEN
        DO I=JFT,JLT
          THK0(I)=MAX(EM20,THK(I))
        ENDDO
       ELSE
        DO I=JFT,JLT
          THK0(I)=THKE(I)
        ENDDO
      ENDIF
C------explicit KFAC=1.0e-3 for quad 1.0e-2 for T3(could be 1.0e-2 for all)--
C------implicit KFAC=0.1 for all----
      IF(IMPL_S>0)THEN
       KFAC= EM01*MIN(ONE,KZ_TOL*2000)
      ELSE
       KFAC= EM03
      ENDIF
C
       IGTYP = IGEO(11,PID(1))
       IGMAT = IGEO(98,PID(1))
       IPGMAT = 700
      IF(IGTYP == 11 .AND. IGMAT > 0) THEN
           DO I=JFT,JLT
               THK02(I) = THK0(I)*THK0(I)
               VOLG(I) = THK0(I)*AREA(I)
               DT1C(I) = DT1
               IPID=PID(I)
               MX = PID(I)
               RHO(I) = GEO(IPGMAT +1 ,MX) 
               YM(I)  = GEO(IPGMAT +2 ,MX) 
               NU(I)  = GEO(IPGMAT +3 ,MX)
               G(I)   = GEO(IPGMAT +4 ,MX) 
               A11(I) = GEO(IPGMAT +5 ,MX)
               A12(I) = GEO(IPGMAT +6 ,MX) 
               A11R(I)= GEO(IPGMAT +7 ,MX)
               SSP(I) = GEO(IPGMAT +9 ,MX)
               GSR(I)  =GEO(IPGMAT +10 ,MX)
               A11SR(I)=GEO(IPGMAT +11 ,MX)
               A12SR(I)=GEO(IPGMAT +12 ,MX)
               NUSR(I) =GEO(IPGMAT +13 ,MX)
               KRZ(I) =KFAC*G(I) 
!!               IZ(I) = GEO(198,PID(I)) ! ---> sum(ti*(ti/2 + zi**2) 
          ENDDO
      ELSEIF(IGTYP == 52 .OR. 
     .      ((IGTYP == 17 .OR. IGTYP == 51) .AND. IGMAT > 0 )) THEN
           DO I=JFT,JLT
               THK02(I) = THK0(I)*THK0(I)
               VOLG(I) = THK0(I)*AREA(I)
               DT1C(I) = DT1
               IPID=PID(I)
               RHO(I) = PM_STACK(1 ,ISUBSTACK) 
               YM(I)  = PM_STACK(2 ,ISUBSTACK) 
               NU(I)  = PM_STACK(3 ,ISUBSTACK)
               G(I)   = PM_STACK(4 ,ISUBSTACK) 
               A11(I) = PM_STACK(5 ,ISUBSTACK)
               A12(I) = PM_STACK(6 ,ISUBSTACK) 
               A11R(I)= PM_STACK(7 ,ISUBSTACK)
               SSP(I) = PM_STACK(9 ,ISUBSTACK)
               GSR(I)  =PM_STACK(10 ,ISUBSTACK)
               A11SR(I)=PM_STACK(11 ,ISUBSTACK)
               A12SR(I)=PM_STACK(12 ,ISUBSTACK)
               NUSR(I) =PM_STACK(13 ,ISUBSTACK)
               KRZ(I) =KFAC*G(I)
          ENDDO 
      ELSEIF(MTN == 58 .or. MTN == 158) THEN
           MX  =MAT(JFT)
C---- due to too high young update (Starter) w/ input func            
           FAC1TMP = PM(23,MX)/PM(20,MX)
           K58(1) = UPARAM( 9)  ! young dir1
           K58(2) = UPARAM(10)  ! young dir2
           K58(3) = MAX(UPARAM(13),UPARAM(14))
           K58I = EM02
           IF (FAC1TMP <ONE) K58I = HALF*K58I     
           FAC58(JFT:JLT,1:2) = K58I
         IF(NPT==1) THEN
           DO I=JFT,JLT                                             
             R1 = DIRA(I,1)                                         
             S1 = DIRA(I,2)                                         
             R2 = DIRB(I,1)                                         
             S2 = DIRB(I,2)                                         
             RS1= R1*S1                                             
             RS2= R2*S2                                             
             R12= R1*R1                                             
             R22= R2*R2                                             
             S12= S1*S1                                             
             S22= S2*S2                                             
             T1 = K58(1)                                         
             T2 = K58(2)                                         
             T3 = K58(3)                                         
             E11 = R12*T1 + R22*T2
             E22 = S12*T1 + S22*T2
             YM(I)  = MAX(E11,E22)
             G(I)  = HALF*FAC1TMP*YM(I)
             NU(I)  = ZERO
             NUSR(I) =EM01
C---- for dt compute -> will be updated by cndt in case of dtnoda        
             A11(I) = YM(I)
             A12(I) = NU(I)*A11(I)
             RFAC = EXP(UVAR(I,4)) 
             RFAT = EXP(UVAR(I,5))
C----  FAC58(I,1:2) could be different values, but too complicated     
             IF (UVAR(I,11)/=ZERO.AND.UVAR(I,12)/=ZERO) THEN
               FAC58(I,1:2) = EM01*K58I
             ELSEIF (MIN(RFAC,RFAT)>ONE) THEN
               FAC58(I,1:2) = 1.2*K58I
           END IF
           ENDDO                                                    
         ELSE
           DO I=JFT,JLT                                             
             E11 = K58(1)                                         
             E22 = K58(2)                                         
             YM(I)  = MAX(E11,E22)
             G(I)  = HALF*YM(I)
             NU(I)  = ZERO
             NUSR(I) =EM01
C---- for dt compute       
             A11(I) = YM(I)
             A12(I) = NU(I)*A11(I)
           ENDDO                                                    
           END IF
           MX  =MAT(JFT)
           DO I=JFT,JLT
             THK02(I) = THK0(I)*THK0(I)
             VOLG(I) = THK0(I)*AREA(I)
             DT1C(I) = DT1
             RHO(I)=PM(1,MX)
             KRZ(I) =KFAC*G(I)
             GSR(I)  =SQRT(G(I))
C----- used in mem damping
             RFAC = MAX(FAC58(I,1),FAC58(I,2))             
             A11SR(I)=SQRT(RFAC*YM(I))
             A12SR(I)=NUSR(I)*A11SR(I)
           ENDDO 
            
      ELSE
          MX  =MAT(JFT)
          DO I=JFT,JLT
               THK02(I) = THK0(I)*THK0(I)
               VOLG(I) = THK0(I)*AREA(I)
               DT1C(I) = DT1
               RHO(I)=PM(1,MX)
               IPID=PID(I)
               YM(I) =PM(20,MX)
               NU(I)  =PM(21,MX)
               G(I)  =PM(22,MX)
               A11(I) =PM(24,MX)
               A12(I) =PM(25,MX)
               SSP(I) =PM(27,MX)
               GSR(I)  =PM(12,MX)
               A11SR(I)=PM(13,MX)
               A12SR(I)=PM(14,MX)
               NUSR(I) =PM(190,MX)
               KRZ(I) =KFAC*G(I)
          ENDDO
      ENDIF
      IF(NPT==1) THEN
        DO I=JFT,JLT
         SHF(I)=ZERO
         SHFSR(I)=ZERO
        ENDDO
      ELSE
        DO I=JFT,JLT
          SHF(I)=GEO(38,PID(I))
          SHFSR(I)=GEO(100,PID(I))
        ENDDO
      ENDIF
      DO I=JFT,JLT
         GS(I)=G(I)*SHF(I)
      ENDDO
      IF (MTN == 58  .or. MTN == 158) THEN
        CONTINUE
      ELSEIF(MTN>=24)THEN
        DO I=JFT,JLT
          A12(I)  =NU(I)*A11(I)
          A12SR(I)=NUSR(I)*A11SR(I)
        ENDDO
      ENDIF
c
c---  Coefficient Visco
c
      IF (IMPL_S == 1) THEN
        DN = ZERO
      ELSE
        DN = GEO(13,PID(1))
        IF(DN == ZERO) DN = ZEP01 + FIVEEM3 ! 0.015 default value
      ENDIF
      AMU(JFT:JLT) = DN
      Z0 = GEO(199,PID(1))
      ZOFFSET(JFT:JLT)  = ZERO
      SELECT CASE(IGTYP)
        CASE (1,9,10,11,16)
           DO I=JFT,JLT
            ZOFFSET(I) = Z0*THK0(I)
           ENDDO 
        CASE (17,51,52)
            IPOS   = IGEO(99,PID(1))
             IF(IPOS == 2) THEN
               DO I=JFT,JLT
                 ZOFFSET(I) = Z0 - HALF*THK0(I)
               ENDDO 
              ELSEIF (IPOS== 3 .OR. IPOS == 4) THEN
               DO I=JFT,JLT
                 Z0= HALF*THK0(I)
                 ZOFFSET (I) = Z0
               ENDDO  
             ENDIF    
        CASE DEFAULT 
            ZOFFSET(JFT:JLT)  = ZERO
      END SELECT
c-----------
      RETURN
      END
