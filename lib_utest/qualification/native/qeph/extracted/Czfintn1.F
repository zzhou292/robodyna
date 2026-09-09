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
        SUBROUTINE CZFINTN1(JFT   ,JLT   ,THK  ,C1   ,AA   ,VHG  ,
     2                      X13   ,X24   ,Y13  ,Y24  ,Z1   ,MX23 ,
     3                      MX13  ,MX34  ,MY13 ,MY23 ,MY34 ,VGLAS,
     4                      VSTRE ,MSTRE ,VF   ,VM   ,FAC  ,A11  ,
     5                      A12   ,G     ,SHF  ,SIGY ,OFF  ,FAC1 ,
     6                      RHO   ,AREA  ,DT1  ,EINT ,AMU  ,VHI  ,
     7                      NPT   ,IPARTC,EVIS ,KFTS ,GSR  ,NEL  ,
     8                      A11SR ,A12SR ,NUSR ,SHFSR,MTN  ,FAC58) 
C elasto-plastic hourglass stresses+linear damping
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
C        CALCUL VF(3,4),VM(2,4) : FORCES INTERNES GENERALISEES
C        ENTREES : THK,1/AREA,PME,X13,X24,Y13,Y24,MX13,MX34,MY13,MY34
C                  dhg(6): increment of the generalized hourglass deformation
C                  VGLAS(12): DEF HOURGLASS GENERALISEE 
C                  VGLAS(1-6)->ETA; VGLAS(7-12)->KSI;
C                  fac=et/e
C        SORTIES : VF,VM
C                  VF(3,4),VM(2,4) : 
C                  J=1,2 STOCK LA PARTIE ANTISYM (F(1)=-F(3),F(2)=-F(4))                                  
C                  J=3,4 STOCK LA PARTIE SYM (F(1)=F(3),F(2)=F(4))
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
#include "implicit_f.inc"
#include "param_c.inc"
#include "mvsiz_p.inc"
C-----------------------------------------------
C   D U M M Y   A R G U M E N T S
C-----------------------------------------------
        INTEGER JFT,JLT,NPT,IPARTC(*),NEL,MTN
        my_real 
     .       THK(*) ,C1(*)  ,AA(*)  ,VHG(MVSIZ,6),
     .       X13(*) ,X24(*) ,Y13(*) ,Y24(*)  ,Z1(*),
     .       MX13(*),MX23(*),MX34(*),MY13(*),MY23(*),MY34(*),
     .       VGLAS(NEL,12),VSTRE(NEL,5),MSTRE(NEL,3),VF(MVSIZ,3,4),VM(MVSIZ,2,4),
     .       FAC(MVSIZ,2),A11(*) ,A12(*) ,G(*),SHF(*),VHI(MVSIZ,3),SIGY(*),
     .       FAC1(*) ,RHO(*) ,AREA(*), DT1,OFF(*),EINT(JLT,2),AMU(*) ,
     .       EVIS(NPSAV,*),
     .       GSR(*), A11SR(*), A12SR(*), NUSR(*), SHFSR(*),FAC58(MVSIZ,2)
         INTEGER KFTS
C-----------------------------------------------
C   L O C A L   V A R I A B L E S
C-----------------------------------------------
        INTEGER I,MX, IC, II, J, JST(MVSIZ+1)
        my_real 
     .  STIER,C2,C3,C4,C5,C7,C8,HSURA,SVM,
     .  SS1,SS2,SS3,SC6,SC5,CXX_K,CYY_K,HXX,HYY,SF1,SF2,B13,B24,
     .  CXX,CYY,Y13S,X13S,Y34S6,Y23S5,X23S5,X34S6,HXX_K,HYY_K,
     .  BXX,BYY,BXX_K,BYY_K,
     .  C1M,C2M,CMM,CNN,UFAC,TOL,COEF,EH1,EH2,
     .  CNNX,CNNY,CNNX_K,CNNY_K,CMMX,CMMY,CMMX_K,CMMY_K,SXY0,MXY0,
     .  SIGY2,UNDOUZSR,AUX,
     .  SSV0,SSV1,SSV2,SSV3,SC6_V,SC5_V,CXX_V,CYY_V,CXY_V,CXZ_V,CYZ_V,
     .  HXX_V,HYY_V,HXY_V,SS1_V,SS2_V,SS3_V,SF1_V,SF2_V,HVL,
     .  EMY,ESY,COEF1,COEFH,FBEND,FBEND_V,C12,
     .  C6(MVSIZ),DGLAS(MVSIZ,12),
     .  ESX(MVSIZ),TESY(MVSIZ),EMX(MVSIZ),DHG(MVSIZ,6),ETMP(MVSIZ,2)
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
      C7=FOUR_OVER_3
      COEFH= ZEP999
      COEF= ZEP85
      STIER  = FIVEP333  
      TOL= EM18 
      FBEND_V =THREEP464
C
      IF (NPT==0) THEN
       COEF1=SIXTEEN
      ELSE
       COEF1=TWENTY5
      ENDIF
      IF (NPT==1) THEN
       FBEND =ZERO
       FBEND_V =ZERO
      ELSE
       FBEND =ONE_OVER_12
      ENDIF
      UNDOUZSR=SQRT(ONE_OVER_12)

      DO I=JFT,JLT
        DHG(I,1)=VHG(I,1)*DT1
        DHG(I,2)=VHG(I,2)*DT1
        DHG(I,3)=VHG(I,3)*DT1
        DHG(I,4)=VHG(I,4)*DT1
        DHG(I,5)=VHG(I,5)*DT1
        DHG(I,6)=VHG(I,6)*DT1
      END DO
      IF(MTN==58) THEN
        IF (NPT==1) THEN
         DO I=JFT,JLT
          C3=FOUR*AA(I)
          HXX=C3*MY34(I)
          HYY=C3*MX34(I)
          HXX_K=C3*MY23(I)
          HYY_K=C3*MX23(I)
          CXX=HXX*VGLAS(I,11)
          CYY=HYY*VGLAS(I,12)
          CXX_K=HXX_K*VGLAS(I,11)
          CYY_K=HYY_K*VGLAS(I,12)
            C2 = A11(I)*FAC1(I)
          C1M=C2*FAC58(I,1)
          C2M=C2*FAC58(I,2)
          C12 = EM02*MIN(C1M,C2M)
          DGLAS(I,1) =C1M*CXX-C12*CYY
          DGLAS(I,2) =C2M*CYY-C12*CXX
          DGLAS(I,7) =C1M*CXX_K-C12*CYY_K
          DGLAS(I,8) =C2M*CYY_K-C12*CXX_K
         END DO
c       if (mod(ncycle,10)==0) print *,'FAC58(I,1-2)=',FAC58(nel,1),FAC58(nel,2)     
          ELSE
         DO I=JFT,JLT
          C3=FOUR*AA(I)
          HXX=C3*MY34(I)
          HYY=C3*MX34(I)
          HXX_K=C3*MY23(I)
          HYY_K=C3*MX23(I)
          CXX=HXX*DHG(I,1)
          CYY=HYY*DHG(I,2)
          CXX_K=HXX_K*DHG(I,1)
          CYY_K=HYY_K*DHG(I,2)
          BXX=HXX*DHG(I,3)
          BYY=HYY*DHG(I,4)
          BXX_K=HXX_K*DHG(I,3)
          BYY_K=HYY_K*DHG(I,4)
            C2 = A11(I)*FAC1(I)
          C1M=C2*FAC58(I,1)
          C2M=C2*FAC58(I,2)
          C12 = EM02*MIN(C1M,C2M)
C      -----------for energy calculation-Mean_value--------
          C6(I)=C1(I)*FBEND
          SS1= MY34(I)*VGLAS(I,1) +MY23(I)*VGLAS(I,7)
          SS2= MX23(I)*VGLAS(I,8) +MX34(I)*VGLAS(I,2)
          SF1= MY34(I)*VGLAS(I,3) +MY23(I)*VGLAS(I,9)
          SF2= -MX23(I)*VGLAS(I,10)-MX34(I)*VGLAS(I,4)
          SC5= MY34(I)*VGLAS(I,5) +MX34(I)*VGLAS(I,6) 
          SC6= MY23(I)*VGLAS(I,11)+MX23(I)*VGLAS(I,12)
          C5=HALF*OFF(I)*THK(I)*C7
          ESX(I)= SS1*DHG(I,1)+SS2*DHG(I,2)
          ETMP(I,1) = C5*(ESX(I) +
     .                    FOURTH*(SC5*DHG(I,5)+SC6*DHG(I,6)))
          EMX(I)= (SF1*DHG(I,3)-SF2*DHG(I,4))*C6(I)
          ETMP(I,2) = C5*EMX(I)
          DGLAS(I,1) =C1M*CXX-C12*CYY
          DGLAS(I,2) =C2M*CYY-C12*CXX
          DGLAS(I,3) =C1M*BXX-C12*BYY
          DGLAS(I,4) =C2M*BYY-C12*BXX
          DGLAS(I,7) =C1M*CXX_K-C12*CYY_K
          DGLAS(I,8) =C2M*CYY_K-C12*CXX_K
          DGLAS(I,9) =C1M*BXX_K-C12*BYY_K
          DGLAS(I,10)=C2M*BYY_K-C12*BXX_K
          C2=FAC1(I)*G(I)*SHF(I)*ONE_OVER_64
          DGLAS(I,5) =C2*HXX*DHG(I,5)
          DGLAS(I,6) =C2*HYY*DHG(I,5)
          DGLAS(I,11)=C2*HXX_K*DHG(I,6)
          DGLAS(I,12)=C2*HYY_K*DHG(I,6)
         END DO
        END IF !(NPT==1) THEN
      ELSE
      DO I=JFT,JLT
        C3=FOUR*AA(I)
        HXX=C3*MY34(I)
        HYY=C3*MX34(I)
        HXX_K=C3*MY23(I)
        HYY_K=C3*MX23(I)
        CXX=HXX*DHG(I,1)
        CYY=HYY*DHG(I,2)
        CXX_K=HXX_K*DHG(I,1)
        CYY_K=HYY_K*DHG(I,2)
        BXX=HXX*DHG(I,3)
        BYY=HYY*DHG(I,4)
        BXX_K=HXX_K*DHG(I,3)
        BYY_K=HYY_K*DHG(I,4)
        C1M=A11(I)*FAC1(I)
        C2M=A12(I)*FAC1(I)
C      -----------for energy calculation-Mean_value--------
        C6(I)=C1(I)*FBEND
        SS1= MY34(I)*VGLAS(I,1) +MY23(I)*VGLAS(I,7)
        SS2= MX23(I)*VGLAS(I,8) +MX34(I)*VGLAS(I,2)
        SF1= MY34(I)*VGLAS(I,3) +MY23(I)*VGLAS(I,9)
        SF2= -MX23(I)*VGLAS(I,10)-MX34(I)*VGLAS(I,4)
        SC5= MY34(I)*VGLAS(I,5) +MX34(I)*VGLAS(I,6) 
        SC6= MY23(I)*VGLAS(I,11)+MX23(I)*VGLAS(I,12)
        C5=HALF*OFF(I)*THK(I)*C7
        ESX(I)= SS1*DHG(I,1)+SS2*DHG(I,2)
        ETMP(I,1) = C5*(ESX(I) +
     .              FOURTH*(SC5*DHG(I,5)+SC6*DHG(I,6)))
        EMX(I)= (SF1*DHG(I,3)-SF2*DHG(I,4))*C6(I)
        ETMP(I,2) = C5*EMX(I)
        DGLAS(I,1) =C1M*CXX-C2M*CYY
        DGLAS(I,2) =C1M*CYY-C2M*CXX
        DGLAS(I,3) =C1M*BXX-C2M*BYY
        DGLAS(I,4) =C1M*BYY-C2M*BXX
        DGLAS(I,7) =C1M*CXX_K-C2M*CYY_K
        DGLAS(I,8) =C1M*CYY_K-C2M*CXX_K
        DGLAS(I,9) =C1M*BXX_K-C2M*BYY_K
        DGLAS(I,10)=C1M*BYY_K-C2M*BXX_K
        C2=FAC1(I)*G(I)*SHF(I)*ONE_OVER_64
        DGLAS(I,5) =C2*HXX*DHG(I,5)
        DGLAS(I,6) =C2*HYY*DHG(I,5)
        DGLAS(I,11)=C2*HXX_K*DHG(I,6)
        DGLAS(I,12)=C2*HYY_K*DHG(I,6)
      END DO
      END IF !IF(MTN==58 )
      
      IF(MTN==58 .AND.NPT==1) THEN
       DO I=JFT,JLT
C      -----------for energy calculation-Mean_value--------
        C8 = C7*OFF(I)
        SS1= (MY34(I)*DGLAS(I,1) +MY23(I)*DGLAS(I,7))*C8
        SS2= (MX23(I)*DGLAS(I,8) +MX34(I)*DGLAS(I,2))*C8
        HSURA=THK(I)*AA(I)
        HVL = AMU(I)*SQRT(RHO(I)*AREA(I)*FAC1(I))*OFF(I)
        SSV0=MY23(I)*MY23(I)
        SSV1=MY34(I)*MY34(I)
        SSV2=MX23(I)*MX23(I)
        SSV3=MX34(I)*MX34(I)
        HXX_V= STIER*(SSV1+SSV0)
        HXY_V=-STIER*(MY34(I)*MX34(I)+MY23(I)*MX23(I))
        HYY_V= STIER*(SSV2+SSV3)
        AUX=AA(I)*HVL
        C1M = A11SR(I)*AUX
        C2M = A12SR(I)*AUX
        CXX_V=C1M*HXX_V
        CYY_V=C1M*HYY_V
        CXY_V=C2M*HXY_V
        SS1_V=CXX_V*VHG(I,1)+CXY_V*VHG(I,2)
        SS2_V=CYY_V*VHG(I,2)+CXY_V*VHG(I,1)
C--------------------------------------------------
        SS1=SS1+SS1_V
        SS2=SS2+SS2_V
        C2=FOURTH*THK(I)
C--------NOEUDS 1,3-------
        B13=(MY13(I)*X24(I)-MX13(I)*Y24(I))*HSURA
        VF(I,1,1)=VF(I,1,1)+B13*SS1
        VF(I,1,3)=C2*SS1
        VF(I,2,1)=VF(I,2,1)+B13*SS2
        VF(I,2,3)=C2*SS2
C--------NOEUDS 2,4-------
        B24=(MX13(I)*Y13(I)-MY13(I)*X13(I))*HSURA
        VF(I,1,2)= VF(I,1,2)+B24*SS1
        VF(I,1,4)=-VF(I,1,3)
        VF(I,2,2)= VF(I,2,2)+B24*SS2
        VF(I,2,4)=-VF(I,2,3)
C
        C2=Z1(I)*HSURA
        VF(I,3,1)=VF(I,3,1)+C2*(SS1*Y24(I)-SS2*X24(I))
        VF(I,3,2)=VF(I,3,2)+C2*(-SS1*Y13(I)+SS2*X13(I))
C      -----------for energy calculation-Mean_value--------
        ESY= ((SS1-SS1_V)*DHG(I,1)+(SS2-SS2_V)*DHG(I,2))*THK(I)
        ETMP(I,1) = ESY
        ETMP(I,2) = ZERO
C       ----viscous energy is removed to hourglass energy----
        TESY(I)= (SS1_V*DHG(I,1)+SS2_V*DHG(I,2))*THK(I)
       ENDDO
            ELSE
      DO I=JFT,JLT
C      ---------elstic increament----------
        VGLAS(I,1) =VGLAS(I,1) +DGLAS(I,1)
        VGLAS(I,2) =VGLAS(I,2) +DGLAS(I,2)
        VGLAS(I,3) =VGLAS(I,3) +DGLAS(I,3)
        VGLAS(I,4) =VGLAS(I,4) +DGLAS(I,4)
        VGLAS(I,7) =VGLAS(I,7) +DGLAS(I,7)
        VGLAS(I,8) =VGLAS(I,8) +DGLAS(I,8)
        VGLAS(I,9) =VGLAS(I,9) +DGLAS(I,9)
        VGLAS(I,10)=VGLAS(I,10)+DGLAS(I,10)
C  -----------/4 compared to formulation-------
        VGLAS(I,5) =VGLAS(I,5) +DGLAS(I,5)
        VGLAS(I,6) =VGLAS(I,6) +DGLAS(I,6)
        VGLAS(I,11)=VGLAS(I,11)+DGLAS(I,11)
        VGLAS(I,12)=VGLAS(I,12)+DGLAS(I,12)
C-------------enint------------------
        EINT(I,1) = EINT(I,1)+ETMP(I,1)
        EINT(I,2) = EINT(I,2)+ETMP(I,2)
      END DO
C
      DO I=JFT,JLT
        IF (SIGY(I)<ZEP9EP30) THEN
          UFAC=ABS(MIN(FAC(I,1),FAC(I,2))-ONE)
          SIGY2 = SIGY(I)*SIGY(I)
          SVM =ZERO
          SXY0=ZERO   
          IF (UFAC<TOL) THEN
            SXY0= VSTRE(I,1)*VSTRE(I,1)+VSTRE(I,2)*VSTRE(I,2)
     .           -VSTRE(I,1)*VSTRE(I,2)+THREE*VSTRE(I,3)*VSTRE(I,3)
            MXY0= MSTRE(I,1)*MSTRE(I,1)+MSTRE(I,2)*MSTRE(I,2)
     .           -MSTRE(I,1)*MSTRE(I,2)+THREE*MSTRE(I,3)*MSTRE(I,3)
            CNN=COEF
            CMM=COEF*THK(I)*ONE_OVER_16
            CNNX=CNN*VGLAS(I,1)
            CNNY=CNN*VGLAS(I,2)
            CNNX_K=CNN*VGLAS(I,7)
            CNNY_K=CNN*VGLAS(I,8)
            CMMX=CMM*VGLAS(I,3)
            CMMY=CMM*VGLAS(I,4)
            CMMX_K=CMM*VGLAS(I,9)
            CMMY_K=CMM*VGLAS(I,10)
            SXY0= SXY0+CNNX*CNNX+CNNY*CNNY-CNNX*CNNY
            MXY0= MXY0+CMMX*CMMX+CMMY*CMMY-CMMX*CMMY
            SXY0= SXY0+CNNX_K*CNNX_K+CNNY_K*CNNY_K-CNNX_K*CNNY_K
            MXY0= MXY0+CMMX_K*CMMX_K+CMMY_K*CMMY_K-CMMX_K*CMMY_K
            SXY0= SXY0+ABS(CNNX*(TWO*CNNX_K-CNNY_K)
     1                    +CNNY*(TWO*CNNY_K-CNNX_K))
            MXY0= MXY0+ABS(CMMX*(TWO*CMMX_K-CMMY_K)
     1                    +CMMY*(TWO*CMMY_K-CMMX_K))     
            SVM =  SXY0+COEF1*MXY0
          ENDIF 
C-----------ELASTIC-PLASTIC global yield criterion------------
          IF (UFAC>=TOL.OR.SVM>SIGY2) THEN
            EH1=MIN(SXY0/MAX(SIGY2,TOL),ONE)
            EH1=MAX(COEFH*EH1,(ONE-FAC(I,1)))
            EH2=MAX(COEFH,(ONE-FAC(I,2)))
            IF (ESX(I)<ZERO) EH1=ZERO
            IF (EMX(I)<ZERO) EH2=ZERO
C-------------MEMBRANE------------------------
            VGLAS(I,1)=VGLAS(I,1)-EH1*DGLAS(I,1)
            VGLAS(I,2)=VGLAS(I,2)-EH1*DGLAS(I,2)
            VGLAS(I,7)=VGLAS(I,7)-EH1*DGLAS(I,7)
            VGLAS(I,8)=VGLAS(I,8)-EH1*DGLAS(I,8)
C-------------FLEXION------------------------
            VGLAS(I,3) =VGLAS(I,3) -EH2*DGLAS(I,3)
            VGLAS(I,4) =VGLAS(I,4) -EH2*DGLAS(I,4)
            VGLAS(I,9) =VGLAS(I,9) -EH2*DGLAS(I,9)
            VGLAS(I,10)=VGLAS(I,10)-EH2*DGLAS(I,10)
          ENDIF
        ENDIF 
      END DO

      DO I=JFT,JLT
C      -----------for energy calculation-Mean_value--------
        C8 = C7*OFF(I)
        SS1= (MY34(I)*VGLAS(I,1) +MY23(I)*VGLAS(I,7))*C8
        SS2= (MX23(I)*VGLAS(I,8) +MX34(I)*VGLAS(I,2))*C8
        SF1= (MY34(I)*VGLAS(I,3) +MY23(I)*VGLAS(I,9))*C8
        SF2= -(MX23(I)*VGLAS(I,10)+MX34(I)*VGLAS(I,4))*C8
        HSURA=THK(I)*AA(I)
        C2 = C8*THK(I)
        SC5=(MY34(I)*VGLAS(I,5) +MX34(I)*VGLAS(I,6) )*C2
        SC6=(MY23(I)*VGLAS(I,11)+MX23(I)*VGLAS(I,12))*C2
        SS3=SC5+SC6
        HVL = AMU(I)*SQRT(RHO(I)*AREA(I)*FAC1(I))*OFF(I)
        SSV0=MY23(I)*MY23(I)
        SSV1=MY34(I)*MY34(I)
        SSV2=MX23(I)*MX23(I)
        SSV3=MX34(I)*MX34(I)
        HXX_V= STIER*(SSV1+SSV0)
        HXY_V=-STIER*(MY34(I)*MX34(I)+MY23(I)*MX23(I))
        HYY_V= STIER*(SSV2+SSV3)
        C2 =HVL*GSR(I)*SHFSR(I)*UNDOUZSR
        CXZ_V=(SSV1+SSV3)*C2
        CYZ_V=(SSV2+SSV0)*C2
        AUX=AA(I)*HVL
        C1M = A11SR(I)*AUX
        C2M = A12SR(I)*AUX
        CXX_V=C1M*HXX_V
        CYY_V=C1M*HYY_V
        CXY_V=C2M*HXY_V
        SS1_V=CXX_V*VHG(I,1)+CXY_V*VHG(I,2)
        SS2_V=CYY_V*VHG(I,2)+CXY_V*VHG(I,1)
        SF1_V= (CXX_V*VHG(I,3)+CXY_V*VHG(I,4))*FBEND_V
        SF2_V=(-CYY_V*VHG(I,4)-CXY_V*VHG(I,3))*FBEND_V
        SC5_V=CXZ_V*VHG(I,5)*HSURA
        SC6_V=CYZ_V*VHG(I,6)*HSURA
        SS3_V = SC5_V+SC6_V
C--------------------------------------------------
        SS1=SS1+SS1_V
        SS2=SS2+SS2_V
        SS3=SS3+SS3_V
        SC5=SC5+SC5_V
        SC6=SC6+SC6_V
        SF1=SF1+SF1_V
        SF2=SF2+SF2_V
        Y13S= MY13(I)*SS3
        X13S= MX13(I)*SS3
        Y34S6=MY34(I)*SC6
        Y23S5=MY23(I)*SC5
        X23S5=MX23(I)*SC5
        X34S6=MX34(I)*SC6
        C2=FOURTH*THK(I)
C--------NOEUDS 1,3-------
        B13=(MY13(I)*X24(I)-MX13(I)*Y24(I))*HSURA
        VF(I,1,1)=VF(I,1,1)+B13*SS1
        VF(I,1,3)=C2*SS1
        VF(I,2,1)=VF(I,2,1)+B13*SS2
        VF(I,2,3)=C2*SS2
        VF(I,3,3)=SS3
C--------NOEUDS 2,4-------
        B24=(MX13(I)*Y13(I)-MY13(I)*X13(I))*HSURA
        VF(I,1,2)= VF(I,1,2)+B24*SS1
        VF(I,1,4)=-VF(I,1,3)
        VF(I,2,2)= VF(I,2,2)+B24*SS2
        VF(I,2,4)=-VF(I,2,3)
        VF(I,3,4)=-VF(I,3,3)
C--------NOEUDS 1,3-------
        C3=C6(I)*B13
        C4=C6(I)*C2
        VM(I,1,1)=VM(I,1,1)+C3*SF2+Y23S5+Y34S6
        VM(I,1,3)=VM(I,1,3)+C4*SF2-Y13S
        VM(I,2,1)=VM(I,2,1)+C3*SF1-X23S5-X34S6
        VM(I,2,3)=VM(I,2,3)+C4*SF1+X13S
C--------NOEUDS 2,4-------
        C3=C6(I)*B24
        VM(I,1,2)=VM(I,1,2)+C3*SF2+Y23S5-Y34S6
        VM(I,1,4)=VM(I,1,4)-C4*SF2-Y13S
        VM(I,2,2)=VM(I,2,2)+C3*SF1-X23S5+X34S6
        VM(I,2,4)=VM(I,2,4)-C4*SF1+X13S
C
        C2=Z1(I)*HSURA
        VF(I,3,1)=VF(I,3,1)+C2*(SS1*Y24(I)-SS2*X24(I))
        VF(I,3,2)=VF(I,3,2)+C2*(-SS1*Y13(I)+SS2*X13(I))
C
C      -----------for energy calculation-Mean_value--------
        ESY= ((SS1-SS1_V)*DHG(I,1)+(SS2-SS2_V)*DHG(I,2))*THK(I)+
     .         FOURTH*((SC5-SC5_V)*DHG(I,5)+(SC6-SC6_V)*DHG(I,6))
        ETMP(I,1) = HALF*ESY
        EMY= (SF1-SF1_V)*DHG(I,3)-(SF2-SF2_V)*DHG(I,4)
        ETMP(I,2) = HALF*C6(I)*EMY*THK(I)
C       ----viscous energy is removed to hourglass energy----
        TESY(I)= (SS1_V*DHG(I,1)+SS2_V*DHG(I,2))*THK(I)+
     .         (SF1_V*DHG(I,3)-SF2_V*DHG(I,4))*THK(I)*C6(I)+
     .         FOURTH*(SC5_V*DHG(I,5)+SC6_V*DHG(I,6))
      ENDDO 
      END IF!(MTN==58 .AND.NPT==1) THEN
C
      DO I=JFT,JLT
        EINT(I,1) = EINT(I,1)+ETMP(I,1)
        EINT(I,2) = EINT(I,2)+ETMP(I,2)
      END DO

      IC=1
      JST(IC)=JFT
      DO J=JFT+1,JLT
         IF (IPARTC(J)/=IPARTC(J-1)) THEN
            IC=IC+1
            JST(IC)=J
         ENDIF
      ENDDO
      JST(IC+1)=JLT+1
      IF(IC==1) THEN
        MX = IPARTC(JFT)
        DO I=JFT,JLT
          EVIS(8,MX)=EVIS(8,MX) + TESY(I)
        ENDDO
      ELSEIF(IC==2.AND.KFTS>0)THEN
        MX = IPARTC(JFT)
        DO I=JFT, KFTS-1
          EVIS(8,MX)=EVIS(8,MX) + TESY(I)
        ENDDO
        MX = IPARTC(JLT)
        DO I=KFTS,JLT
          EVIS(8,MX)=EVIS(8,MX) + TESY(I)
        ENDDO

      ELSE
        DO II=1,IC
           MX=IPARTC(JST(II))
           IF (JST(II+1)-JST(II)>15) THEN
              DO J=JST(II),JST(II+1)-1
                EVIS(8,MX)=EVIS(8,MX) + TESY(J)
              ENDDO
           ELSE
              DO J=JST(II),JST(II+1)-1
                EVIS(8,MX)=EVIS(8,MX) + TESY(J)
              ENDDO
           ENDIF
        ENDDO
      ENDIF
      RETURN
      END
