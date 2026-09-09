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
      SUBROUTINE CZCORC1(NUMNOD ,NUMELC ,ELBUF_STR,
     1                   JFT    ,JLT    ,X      ,V      ,VR     ,
     2                   IXC    ,PM     ,PLAT   ,AREA   ,
     3                   AREA_I ,V13    ,V24    ,VHI    ,RLXYZ  ,
     4                   VQN    ,VQ     ,LL     ,L13    ,L24    ,
     5                   X13    ,X24    ,Y13    ,Y24    ,MX13   ,
     6                   MX23   ,MX34   ,MY13   ,MY23   ,MY34   ,
     7                   Z1     ,COREL  ,DI     ,DB     ,SMSTR  ,
     9                   IREP   ,NPT    ,NLAY   ,ISMSTR ,
     A                   DIR_A  ,DIR_B  ,OFFG   ,RLXYZV ,CORELV ,
     B                   FACN   ,PY1    ,PX2    ,PY2    ,R11    ,
     C                   R12    ,R13    ,R21    ,R22    ,R23    ,
     D                   R31    ,R32    ,R33    ,RLZ    ,IDRIL  ,
     E                   IXFEM  ,VX1    ,VX2    ,VX3    ,VX4    ,
     F                   VY1    ,VY2    ,VY3    ,VY4    ,VZ1    ,
     G                   VZ2    ,VZ3    ,VZ4    ,VRX1   ,VRX2   ,
     H                   VRX3   ,VRX4   ,VRY1   ,VRY2   ,VRY3   ,
     I                   VRY4   ,VRZ1   ,VRZ2   ,VRZ3   ,VRZ4   ,
     J                   X1G    ,X2G    ,X3G    ,X4G    ,Y1G    ,
     K                   Y2G    ,Y3G    ,Y4G    ,Z1G    ,Z2G    ,
     L                   Z3G    ,Z4G    ,THK    ,DIZ    ,UX1    ,
     M                   UX2    ,UX3    ,UX4    ,UY1    ,UY2    ,
     N                   UY3    ,UY4    ,XL2    ,XL3    ,XL4    ,
     O                   YL2    ,YL3    ,YL4    ,VL1    ,VL2    ,
     P                   VL3    ,VL4    ,NEL    ,Z2     )
C-----------------------------------------------
C   M o d u l e s
C-----------------------------------------------
        USE ELBUFDEF_MOD
        use element_mod , only : nixc
C-----------------------------------------------
#include      "implicit_f.inc"
c-----------------------------------------------
c   g l o b a l   p a r a m e t e r s
c-----------------------------------------------
#include      "mvsiz_p.inc"
#include      "param_c.inc"
#include      "parit_c.inc"
#include      "scr05_c.inc"
#include      "scr17_c.inc"
#include      "com08_c.inc"
#include      "impl1_c.inc"
C-----------------------------------------------
C   D U M M Y   A R G U M E N T S
C-----------------------------------------------
        LOGICAL PLAT(*)
        INTEGER, INTENT(IN) :: NUMNOD,NUMELC
        INTEGER IXC(NIXC,*),JFT,JLT,IREP,NPT,NLAY,ISMSTR,IDRIL,IXFEM,NEL
        my_real
     .     PM(NPROPM,*), X(3,*), V(3,*), VR(3,*),RLXYZ(MVSIZ,2,4),
     .     V13(MVSIZ,3),V24(MVSIZ,3),VHI(MVSIZ,3),MX23(NEL),MY13(NEL),MY23(NEL),MY34(NEL),
     .     LL(NEL),L13(NEL),L24(NEL),X13(NEL),X24(NEL),Y13(NEL),Y24(NEL),MX13(NEL),
     .     VQ(MVSIZ,3,3),AREA(NEL),Z1(NEL),MX34(NEL),VQN(MVSIZ,3,4),AREA_I(NEL),
     .     COREL(MVSIZ,2,4),DI(MVSIZ,6),DB(MVSIZ,3,4),DIR_A(NEL,*),DIR_B(NEL,*),OFFG(NEL),
     .     RLXYZV(MVSIZ,2,4),CORELV(MVSIZ,2,4),FACN(MVSIZ,2),PX2(*),
     .     PY1(NEL), PY2(NEL),R11(MVSIZ),R12(MVSIZ),R13(MVSIZ),
     .     R21(MVSIZ),R22(MVSIZ),R23(MVSIZ),R31(MVSIZ),R32(MVSIZ),
     .     R33(MVSIZ),RLZ(MVSIZ,4),DIZ(MVSIZ,3),THK(*),
     .     VX1(MVSIZ),VX2(MVSIZ),VX3(MVSIZ),VX4(MVSIZ),
     .     VY1(MVSIZ),VY2(MVSIZ),VY3(MVSIZ),VY4(MVSIZ),
     .     VZ1(MVSIZ),VZ2(MVSIZ),VZ3(MVSIZ),VZ4(MVSIZ),
     .     VRX1(MVSIZ),VRX2(MVSIZ),VRX3(MVSIZ),VRX4(MVSIZ),
     .     VRY1(MVSIZ),VRY2(MVSIZ),VRY3(MVSIZ),VRY4(MVSIZ),
     .     VRZ1(MVSIZ),VRZ2(MVSIZ),VRZ3(MVSIZ),VRZ4(MVSIZ),
     .     X1G(MVSIZ),X2G(MVSIZ),X3G(MVSIZ),X4G(MVSIZ),
     .     Y1G(MVSIZ),Y2G(MVSIZ),Y3G(MVSIZ),Y4G(MVSIZ),
     .     Z1G(MVSIZ),Z2G(MVSIZ),Z3G(MVSIZ),Z4G(MVSIZ),
     .     UX1(MVSIZ),UX2(MVSIZ),UX3(MVSIZ),UX4(MVSIZ),
     .     UY1(MVSIZ),UY2(MVSIZ),UY3(MVSIZ),UY4(MVSIZ),
     .     XL2(MVSIZ),XL3(MVSIZ),XL4(MVSIZ),
     .     YL2(MVSIZ),YL3(MVSIZ),YL4(MVSIZ),
     .     Z2(MVSIZ),VL1(MVSIZ,3),VL2(MVSIZ,3),VL3(MVSIZ,3),VL4(MVSIZ,3)
        TYPE (ELBUF_STRUCT_) :: ELBUF_STR
        DOUBLE PRECISION
     .    SMSTR(*)
C-----------------------------------------------
C   L O C A L   V A R I A B L E S
C-----------------------------------------------
        INTEGER IXCTMP2,IXCTMP3,IXCTMP4,IXCTMP5
        INTEGER NNOD,I,J,K,II(6)
        PARAMETER (NNOD = 4)
        my_real
     .     LXYZ0(3),DETA1(MVSIZ),RX(MVSIZ), RY(MVSIZ), RZ(MVSIZ),
     .     SX(MVSIZ),SY(MVSIZ),SSZ(MVSIZ),
     .     VG13(3),VG24(3),VGHI(3),
     .     TOL,X13_2(MVSIZ),Y13_2(MVSIZ),X24_2(MVSIZ),Y24_2(MVSIZ),
     .     C1,C2,S1,
     .     XX,YY,ZZ,
     .     
     .     
     .     HS(MVSIZ),FACI,FAC1,FAC2,LM(MVSIZ),FACDT,
     .     DT05,DT025,EXZ,EYZ,DDRX,DDRY,V13X,V24X,VHIX,DDRZ1,DDRZ2
        my_real :: OFF_LOC
C----------------------------------------------

        IF(IRESP == 1)THEN
          TOL=EM7
        ELSE
          TOL=EM8
        END IF
C
        DO I=1,6
          II(I) = NEL*(I-1)
        ENDDO
C
        IF(IXFEM==0)THEN
          DO I=JFT,JLT
            IXCTMP2=IXC(2,I)
            IXCTMP3=IXC(3,I)
            IXCTMP4=IXC(4,I)
            IXCTMP5=IXC(5,I)

            RX(I)=X(1,IXCTMP3)+X(1,IXCTMP4)-X(1,IXCTMP2)-X(1,IXCTMP5)
            SX(I)=X(1,IXCTMP4)+X(1,IXCTMP5)-X(1,IXCTMP2)-X(1,IXCTMP3)
            RY(I)=X(2,IXCTMP3)+X(2,IXCTMP4)-X(2,IXCTMP2)-X(2,IXCTMP5)
            SY(I)=X(2,IXCTMP4)+X(2,IXCTMP5)-X(2,IXCTMP2)-X(2,IXCTMP3)
            RZ(I)=X(3,IXCTMP3)+X(3,IXCTMP4)-X(3,IXCTMP2)-X(3,IXCTMP5)
            SSZ(I)=X(3,IXCTMP4)+X(3,IXCTMP5)-X(3,IXCTMP2)-X(3,IXCTMP3)
          ENDDO
        ELSE IF(IXFEM==1)THEN
          DO I=JFT,JLT
            RX(I)=X2G(I)+X3G(I)-X1G(I)-X4G(I)
            SX(I)=X3G(I)+X4G(I)-X1G(I)-X2G(I)
            RY(I)=Y2G(I)+Y3G(I)-Y1G(I)-Y4G(I)
            SY(I)=Y3G(I)+Y4G(I)-Y1G(I)-Y2G(I)
            RZ(I)=Z2G(I)+Z3G(I)-Z1G(I)-Z4G(I)
            SSZ(I)=Z3G(I)+Z4G(I)-Z1G(I)-Z2G(I)
          ENDDO
        END IF
C----------------------------
C     LOCAL SYSTEM
C----------------------------
        K = 0
        CALL CLSKEW3(JFT,JLT,K,
     .     RX, RY, RZ,
     .     SX, SY, SSZ,
     .     R11,R12,R13,R21,R22,R23,R31,R32,R33,DETA1,OFFG )
        DO I=JFT,JLT
          AREA(I)=FOURTH*DETA1(I)
          OFF_LOC = ZERO
          IF(ABS(OFFG(I))/=ZERO) OFF_LOC = ONE
          AREA_I(I)=OFF_LOC/AREA(I)
          AREA_I(I) = MAX(AREA_I(I),EM20)
          VQ(I,1,1)=R11(I)
          VQ(I,2,1)=R21(I)
          VQ(I,3,1)=R31(I)
          VQ(I,1,2)=R12(I)
          VQ(I,2,2)=R22(I)
          VQ(I,3,2)=R32(I)
          VQ(I,1,3)=R13(I)
          VQ(I,2,3)=R23(I)
          VQ(I,3,3)=R33(I)
        ENDDO
C--------------------------
C     TRANSFERT GLOBAL-->LOCAL
C--------------------------
        IF(IXFEM==0)THEN
          DO I=JFT,JLT
            IXCTMP2=IXC(2,I)
            IXCTMP3=IXC(3,I)
            IXCTMP4=IXC(4,I)
            IXCTMP5=IXC(5,I)

            LXYZ0(1)=FOURTH*(X(1,IXCTMP4)+X(1,IXCTMP5) + X(1,IXCTMP2)+X(1,IXCTMP3))
            LXYZ0(2)=FOURTH*(X(2,IXCTMP4)+X(2,IXCTMP5) + X(2,IXCTMP2)+X(2,IXCTMP3))
            LXYZ0(3)=FOURTH*(X(3,IXCTMP4)+X(3,IXCTMP5) + X(3,IXCTMP2)+X(3,IXCTMP3))

            J=IXCTMP2
            K=IXCTMP3
            XX=X(1,K)-X(1,J)
            YY=X(2,K)-X(2,J)
            ZZ=X(3,K)-X(3,J)
            XL2(I)=R11(I)*XX+R21(I)*YY+R31(I)*ZZ
            YL2(I)=R12(I)*XX+R22(I)*YY+R32(I)*ZZ
            XX=X(1,J)-LXYZ0(1)
            YY=X(2,J)-LXYZ0(2)
            ZZ=X(3,J)-LXYZ0(3)
            Z1(I)=R13(I)*XX+R23(I)*YY+R33(I)*ZZ
            K=IXCTMP4
            XX=X(1,K)-X(1,J)
            YY=X(2,K)-X(2,J)
            ZZ=X(3,K)-X(3,J)
            XL3(I)=R11(I)*XX+R21(I)*YY+R31(I)*ZZ
            YL3(I)=R12(I)*XX+R22(I)*YY+R32(I)*ZZ
            K=IXCTMP5
            XX=X(1,K)-X(1,J)
            YY=X(2,K)-X(2,J)
            ZZ=X(3,K)-X(3,J)
            XL4(I)=R11(I)*XX+R21(I)*YY+R31(I)*ZZ
            YL4(I)=R12(I)*XX+R22(I)*YY+R32(I)*ZZ
            Z2(I) = Z1(I)*Z1(I)
          ENDDO
        ELSE IF(IXFEM==1)THEN
          DO I=JFT,JLT
            LXYZ0(1)=FOURTH*(X3G(I)+X4G(I)+X1G(I)+X2G(I))
            LXYZ0(2)=FOURTH*(Y3G(I)+Y4G(I)+Y1G(I)+Y2G(I))
            LXYZ0(3)=FOURTH*(Z3G(I)+Z4G(I)+Z1G(I)+Z2G(I))
            XX=X2G(I)-X1G(I)
            YY=Y2G(I)-Y1G(I)
            ZZ=Z2G(I)-Z1G(I)
            XL2(I)=R11(I)*XX+R21(I)*YY+R31(I)*ZZ
            YL2(I)=R12(I)*XX+R22(I)*YY+R32(I)*ZZ
            XX=X1G(I)-LXYZ0(1)
            YY=Y1G(I)-LXYZ0(2)
            ZZ=Z1G(I)-LXYZ0(3)
            Z1(I)=R13(I)*XX+R23(I)*YY+R33(I)*ZZ
            XX=X3G(I)-X1G(I)
            YY=Y3G(I)-Y1G(I)
            ZZ=Z3G(I)-Z1G(I)
            XL3(I)=R11(I)*XX+R21(I)*YY+R31(I)*ZZ
            YL3(I)=R12(I)*XX+R22(I)*YY+R32(I)*ZZ
            XX=X4G(I)-X1G(I)
            YY=Y4G(I)-Y1G(I)
            ZZ=Z4G(I)-Z1G(I)
            XL4(I)=R11(I)*XX+R21(I)*YY+R31(I)*ZZ
            YL4(I)=R12(I)*XX+R22(I)*YY+R32(I)*ZZ
            Z2(I) = Z1(I)*Z1(I)
          ENDDO
        END IF
C----------------------------
C     SMALL STRAIN
C----------------------------
        IF(ISMSTR == 11)THEN
#include "vectorize.inc"
          DO I=JFT,JLT
            IF(ABS(OFFG(I)) == ONE)OFFG(I)=SIGN(TWO,OFFG(I))
            UX1(I) = ZERO
            UY1(I) = ZERO
            UX2(I) = ZERO
            UY2(I) = ZERO
            UX3(I) = ZERO
            UY3(I) = ZERO
            UX4(I) = ZERO
            UY4(I) = ZERO
            IF(ABS(OFFG(I)) == TWO)THEN
              UX2(I) = XL2(I)-SMSTR(II(1)+I)
              UY2(I) = YL2(I)-SMSTR(II(2)+I)
              UX3(I) = XL3(I)-SMSTR(II(3)+I)
              UY3(I) = YL3(I)-SMSTR(II(4)+I)
              UX4(I) = XL4(I)-SMSTR(II(5)+I)
              UY4(I) = YL4(I)-SMSTR(II(6)+I)
              XL2(I) = SMSTR(II(1)+I)
              YL2(I) = SMSTR(II(2)+I)
              XL3(I) = SMSTR(II(3)+I)
              YL3(I) = SMSTR(II(4)+I)
              XL4(I) = SMSTR(II(5)+I)
              YL4(I) = SMSTR(II(6)+I)
              Z1(I) = ZERO
              AREA(I) = HALF*((XL2(I)-XL4(I))*YL3(I)-XL3(I)*(YL2(I)-YL4(I)))
              AREA_I(I) = ONE/MAX(EM20,AREA(I))
            ELSE
              SMSTR(II(1)+I) = XL2(I)
              SMSTR(II(2)+I) = YL2(I)
              SMSTR(II(3)+I) = XL3(I)
              SMSTR(II(4)+I) = YL3(I)
              SMSTR(II(5)+I) = XL4(I)
              SMSTR(II(6)+I) = YL4(I)
            ENDIF
          ENDDO
        ELSEIF(ISMSTR == 1.OR.ISMSTR == 2)THEN
#include "vectorize.inc"
          DO I=JFT,JLT
            IF(ABS(OFFG(I)) == TWO)THEN
              XL2(I) = SMSTR(II(1)+I)
              YL2(I) = SMSTR(II(2)+I)
              XL3(I) = SMSTR(II(3)+I)
              YL3(I) = SMSTR(II(4)+I)
              XL4(I) = SMSTR(II(5)+I)
              YL4(I) = SMSTR(II(6)+I)
              Z1(I) = ZERO
              AREA(I) = HALF*((XL2(I)-XL4(I))*YL3(I)-XL3(I)*(YL2(I)-YL4(I)))
              AREA_I(I) = ONE/MAX(EM20,AREA(I))
            ELSE
              SMSTR(II(1)+I)=XL2(I)
              SMSTR(II(2)+I)=YL2(I)
              SMSTR(II(3)+I)=XL3(I)
              SMSTR(II(4)+I)=YL3(I)
              SMSTR(II(5)+I)=XL4(I)
              SMSTR(II(6)+I)=YL4(I)
            ENDIF
          ENDDO
        ENDIF
        IF(ISMSTR == 1)THEN
          DO I=JFT,JLT
            IF(OFFG(I) == ONE)OFFG(I)=TWO
          ENDDO
        ENDIF
C----------------------------
C     ORTHOTROPY
C----------------------------
        CALL CORTDIR3(ELBUF_STR,DIR_A ,DIR_B  ,JFT    ,JLT    ,
     .                NLAY     ,IREP  ,RX     ,RY     ,RZ     ,
     .                SX       ,SY    ,SSZ    ,R11    ,R21    ,
     .                R31      ,R12   ,R22    ,R32    ,NEL    )
C----------------------------
        IF (IVECTOR == 1)THEN
        ELSE
C
          DO I=JFT,JLT
            LXYZ0(1)=FOURTH*(XL2(I)+XL3(I)+XL4(I))
            LXYZ0(2)=FOURTH*(YL2(I)+YL3(I)+YL4(I))
            COREL(I,1,1)=-LXYZ0(1)
            COREL(I,1,2)=XL2(I)-LXYZ0(1)
            COREL(I,1,3)=XL3(I)-LXYZ0(1)
            COREL(I,1,4)=XL4(I)-LXYZ0(1)
            COREL(I,2,1)=-LXYZ0(2)
            COREL(I,2,2)=YL2(I)-LXYZ0(2)
            COREL(I,2,3)=YL3(I)-LXYZ0(2)
            COREL(I,2,4)=YL4(I)-LXYZ0(2)
            X13(I)=(COREL(I,1,1)-COREL(I,1,3))*HALF
            X24(I)=(COREL(I,1,2)-COREL(I,1,4))*HALF
            Y13(I)=(COREL(I,2,1)-COREL(I,2,3))*HALF
            Y24(I)=(COREL(I,2,2)-COREL(I,2,4))*HALF
C
            MX13(I)=(COREL(I,1,1)+COREL(I,1,3))*HALF
            MX23(I)=(COREL(I,1,2)+COREL(I,1,3))*HALF
            MX34(I)=(COREL(I,1,3)+COREL(I,1,4))*HALF
            MY13(I)=(COREL(I,2,1)+COREL(I,2,3))*HALF
            MY23(I)=(COREL(I,2,2)+COREL(I,2,3))*HALF
            MY34(I)=(COREL(I,2,3)+COREL(I,2,4))*HALF
C
            PY1(I) = -X24(I)
            PX2(I) = -Y13(I)
            PY2(I) =  X13(I)
C-
            X13_2(I) =X13(I)*X13(I)
            Y13_2(I) =Y13(I)*Y13(I)
            X24_2(I) =X24(I)*X24(I)
            Y24_2(I) =Y24(I)*Y24(I)
            L13(I)=X13_2(I)+Y13_2(I)
C-------------------------------
            L24(I)=X24_2(I)+Y24_2(I)
C  ---------Factor for characteristic length---
            C1 =COREL(I,1,2)*COREL(I,2,4)-COREL(I,2,2)*COREL(I,1,4)
            C2 =COREL(I,1,1)*COREL(I,2,3)-COREL(I,2,1)*COREL(I,1,3)
            HS(I) =MAX(ABS(C1),ABS(C2))*AREA_I(I)
C
          ENDDO
        ENDIF
C----------------------------
C---------characteristic length ratio ---
C----------------------------
        FACDT = FIVE_OVER_4
C-------same than QBAT
        IF (IDT1SH>0) FACDT =FOUR_OVER_3
        DO I=JFT,JLT
          RX(I)=XL2(I)+XL3(I)-XL4(I)
          RY(I)=YL2(I)+YL3(I)-YL4(I)
          SX(I)=-XL2(I)+XL3(I)+XL4(I)
          SY(I)=-YL2(I)+YL3(I)+YL4(I)
          C1=SQRT(RX(I)*RX(I)+RY(I)*RY(I))
          C2=SQRT(SX(I)*SX(I)+SY(I)*SY(I))
          S1=FOURTH*(MAX(C1,C2)/MIN(C1,C2)-ONE)
          FAC1=MIN(HALF,S1)+ONE
          FAC2=FOUR*AREA(I)/(C1*C2)
          FAC2=3.413*MAX(ZERO,FAC2-0.7071)
          FAC2=0.78+0.22*FAC2*FAC2*FAC2
          FACI=TWO*FAC1*FAC2
C
          LL(I)=MAX(L13(I),L24(I))
          LM(I)=HALF*(L13(I)+L24(I))
          FACN(I,1)=SQRT(L24(I)/LL(I))
          FACN(I,2)=SQRT(L13(I)/LL(I))
          S1 = SQRT(FACI*(FACDT+HS(I))*LL(I))
          S1 = MAX(S1,EM10)
          LL(I) = AREA(I)/S1
        ENDDO
C-------cas thk >>L----
        IF (IMPL_S>0) THEN
          DO I=JFT,JLT
            S1=EM01*THK(I)*THK(I)
            LM(I)=MAX(LM(I),S1)
          ENDDO
        END IF
        IF (IVECTOR == 1)THEN
        ELSE
          IF(IXFEM==0)THEN
            DO I=JFT,JLT
              K=IXC(2,I)
              RLXYZ(I,1,1) =VQ(I,1,1)*VR(1,K)+VQ(I,2,1)*VR(2,K)+VQ(I,3,1)*VR(3,K)
              RLXYZ(I,2,1) =VQ(I,1,2)*VR(1,K)+VQ(I,2,2)*VR(2,K)+VQ(I,3,2)*VR(3,K)
              K=IXC(3,I)
              RLXYZ(I,1,2) =VQ(I,1,1)*VR(1,K)+VQ(I,2,1)*VR(2,K)+VQ(I,3,1)*VR(3,K)
              RLXYZ(I,2,2) =VQ(I,1,2)*VR(1,K)+VQ(I,2,2)*VR(2,K)+VQ(I,3,2)*VR(3,K)
              K=IXC(4,I)
              RLXYZ(I,1,3) =VQ(I,1,1)*VR(1,K)+VQ(I,2,1)*VR(2,K)+VQ(I,3,1)*VR(3,K)
              RLXYZ(I,2,3) =VQ(I,1,2)*VR(1,K)+VQ(I,2,2)*VR(2,K)+VQ(I,3,2)*VR(3,K)
              K=IXC(5,I)
              RLXYZ(I,1,4) =VQ(I,1,1)*VR(1,K)+VQ(I,2,1)*VR(2,K)+VQ(I,3,1)*VR(3,K)
              RLXYZ(I,2,4) =VQ(I,1,2)*VR(1,K)+VQ(I,2,2)*VR(2,K)+VQ(I,3,2)*VR(3,K)
            ENDDO
          ELSE IF(IXFEM==1)THEN
            DO I=JFT,JLT

              RLXYZ(I,1,1) =VQ(I,1,1)*VRX1(I)+VQ(I,2,1)*VRY1(I)+VQ(I,3,1)*VRZ1(I)
              RLXYZ(I,2,1) =VQ(I,1,2)*VRX1(I)+VQ(I,2,2)*VRY1(I)+VQ(I,3,2)*VRZ1(I)

              RLXYZ(I,1,2) =VQ(I,1,1)*VRX2(I)+VQ(I,2,1)*VRY2(I)+VQ(I,3,1)*VRZ2(I)
              RLXYZ(I,2,2) =VQ(I,1,2)*VRX2(I)+VQ(I,2,2)*VRY2(I)+VQ(I,3,2)*VRZ2(I)

              RLXYZ(I,1,3) =VQ(I,1,1)*VRX3(I)+VQ(I,2,1)*VRY3(I)+VQ(I,3,1)*VRZ3(I)
              RLXYZ(I,2,3) =VQ(I,1,2)*VRX3(I)+VQ(I,2,2)*VRY3(I)+VQ(I,3,2)*VRZ3(I)

              RLXYZ(I,1,4) =VQ(I,1,1)*VRX4(I)+VQ(I,2,1)*VRY4(I)+VQ(I,3,1)*VRZ4(I)
              RLXYZ(I,2,4) =VQ(I,1,2)*VRX4(I)+VQ(I,2,2)*VRY4(I)+VQ(I,3,2)*VRZ4(I)
            ENDDO
          ENDIF
        ENDIF
C
        IF(IXFEM==0)THEN
          DO I=JFT,JLT

            IXCTMP2=IXC(2,I)
            IXCTMP3=IXC(3,I)
            IXCTMP4=IXC(4,I)
            IXCTMP5=IXC(5,I)

            VL1(I,1)=V(1,IXCTMP2)
            VL1(I,2)=V(2,IXCTMP2)
            VL1(I,3)=V(3,IXCTMP2)
            VL2(I,1)=V(1,IXCTMP3)
            VL2(I,2)=V(2,IXCTMP3)
            VL2(I,3)=V(3,IXCTMP3)
            VL3(I,1)=V(1,IXCTMP4)
            VL3(I,2)=V(2,IXCTMP4)
            VL3(I,3)=V(3,IXCTMP4)
            VL4(I,1)=V(1,IXCTMP5)
            VL4(I,2)=V(2,IXCTMP5)
            VL4(I,3)=V(3,IXCTMP5)
          ENDDO
          DO I=JFT,JLT
            VG13(1)=VL1(I,1)-VL3(I,1)
            VG24(1)=VL2(I,1)-VL4(I,1)
            VGHI(1)=VL1(I,1)-VL2(I,1)+VL3(I,1)-VL4(I,1)
C
            VG13(2)=VL1(I,2)-VL3(I,2)
            VG24(2)=VL2(I,2)-VL4(I,2)
            VGHI(2)=VL1(I,2)-VL2(I,2)+VL3(I,2)-VL4(I,2)
C
            VG13(3)=VL1(I,3)-VL3(I,3)
            VG24(3)=VL2(I,3)-VL4(I,3)
            VGHI(3)=VL1(I,3)-VL2(I,3)+VL3(I,3)-VL4(I,3)
C
            V13(I,1)=(VQ(I,1,1)*VG13(1)+VQ(I,2,1)*VG13(2)+VQ(I,3,1)*VG13(3))
            V24(I,1)=(VQ(I,1,1)*VG24(1)+VQ(I,2,1)*VG24(2)+VQ(I,3,1)*VG24(3))
            VHI(I,1)=(VQ(I,1,1)*VGHI(1)+VQ(I,2,1)*VGHI(2)+VQ(I,3,1)*VGHI(3))
            V13(I,2)=(VQ(I,1,2)*VG13(1)+VQ(I,2,2)*VG13(2)+VQ(I,3,2)*VG13(3))
            V24(I,2)=(VQ(I,1,2)*VG24(1)+VQ(I,2,2)*VG24(2)+VQ(I,3,2)*VG24(3))
            VHI(I,2)=(VQ(I,1,2)*VGHI(1)+VQ(I,2,2)*VGHI(2)+VQ(I,3,2)*VGHI(3))
            V13(I,3)=(VQ(I,1,3)*VG13(1)+VQ(I,2,3)*VG13(2)+VQ(I,3,3)*VG13(3))
            V24(I,3)=(VQ(I,1,3)*VG24(1)+VQ(I,2,3)*VG24(2)+VQ(I,3,3)*VG24(3))
            VHI(I,3)=(VQ(I,1,3)*VGHI(1)+VQ(I,2,3)*VGHI(2)+VQ(I,3,3)*VGHI(3))
          ENDDO
        ELSE IF(IXFEM==1)THEN
          DO I=JFT,JLT
            VG13(1)=VX1(I)-VX3(I)
            VG24(1)=VX2(I)-VX4(I)
            VGHI(1)=VX1(I)-VX2(I)+VX3(I)-VX4(I)
C
            VG13(2)=VY1(I)-VY3(I)
            VG24(2)=VY2(I)-VY4(I)
            VGHI(2)=VY1(I)-VY2(I)+VY3(I)-VY4(I)
C
            VG13(3)=VZ1(I)-VZ3(I)
            VG24(3)=VZ2(I)-VZ4(I)
            VGHI(3)=VZ1(I)-VZ2(I)+VZ3(I)-VZ4(I)
C
            V13(I,1)=(VQ(I,1,1)*VG13(1)+VQ(I,2,1)*VG13(2)+VQ(I,3,1)*VG13(3))
            V24(I,1)=(VQ(I,1,1)*VG24(1)+VQ(I,2,1)*VG24(2)+VQ(I,3,1)*VG24(3))
            VHI(I,1)=(VQ(I,1,1)*VGHI(1)+VQ(I,2,1)*VGHI(2)+VQ(I,3,1)*VGHI(3))
            V13(I,2)=(VQ(I,1,2)*VG13(1)+VQ(I,2,2)*VG13(2)+VQ(I,3,2)*VG13(3))
            V24(I,2)=(VQ(I,1,2)*VG24(1)+VQ(I,2,2)*VG24(2)+VQ(I,3,2)*VG24(3))
            VHI(I,2)=(VQ(I,1,2)*VGHI(1)+VQ(I,2,2)*VGHI(2)+VQ(I,3,2)*VGHI(3))
            V13(I,3)=(VQ(I,1,3)*VG13(1)+VQ(I,2,3)*VG13(2)+VQ(I,3,3)*VG13(3))
            V24(I,3)=(VQ(I,1,3)*VG24(1)+VQ(I,2,3)*VG24(2)+VQ(I,3,3)*VG24(3))
            VHI(I,3)=(VQ(I,1,3)*VGHI(1)+VQ(I,2,3)*VGHI(2)+VQ(I,3,3)*VGHI(3))
          ENDDO
        END IF
C--------------------------
        IF (IDRIL>0) THEN
          IF(IXFEM==0)THEN
            DO I=JFT,JLT
              K=IXC(2,I)
              RLZ(I,1) =(VQ(I,1,3)*VR(1,K)+VQ(I,2,3)*VR(2,K)+VQ(I,3,3)*VR(3,K))*AREA_I(I)
              K=IXC(3,I)
              RLZ(I,2) =(VQ(I,1,3)*VR(1,K)+VQ(I,2,3)*VR(2,K)+VQ(I,3,3)*VR(3,K))*AREA_I(I)
              K=IXC(4,I)
              RLZ(I,3) =(VQ(I,1,3)*VR(1,K)+VQ(I,2,3)*VR(2,K)+VQ(I,3,3)*VR(3,K))*AREA_I(I)
              K=IXC(5,I)
              RLZ(I,4) =(VQ(I,1,3)*VR(1,K)+VQ(I,2,3)*VR(2,K)+VQ(I,3,3)*VR(3,K))*AREA_I(I)
            ENDDO
          ELSE IF(IXFEM==1)THEN
            DO I=JFT,JLT
              RLZ(I,1) =(VQ(I,1,3)*VRX1(I)+VQ(I,2,3)*VRY1(I)+VQ(I,3,3)*VRZ1(I))*AREA_I(I)
              RLZ(I,2) =(VQ(I,1,3)*VRX2(I)+VQ(I,2,3)*VRY2(I)+VQ(I,3,3)*VRZ2(I))*AREA_I(I)
              RLZ(I,3) =(VQ(I,1,3)*VRX3(I)+VQ(I,2,3)*VRY3(I)+VQ(I,3,3)*VRZ3(I))*AREA_I(I)
              RLZ(I,4) =(VQ(I,1,3)*VRX4(I)+VQ(I,2,3)*VRY4(I)+VQ(I,3,3)*VRZ4(I))*AREA_I(I)
            ENDDO
          ENDIF
        END IF
C--------------------------
C-------Correction 2nd order rigid rotation due to V(t+dt/2),X(t+dt)----
C--------------------------
        IF (IMPL_S == 0.OR.IMP_LR>0) THEN
          DT05 = HALF*DT1
          DT025 =FOURTH*DT1
          DO I=JFT,JLT
            EXZ =  Y24(I)*V13(I,3)-Y13(I)*V24(I,3)
            EYZ = -X24(I)*V13(I,3)+X13(I)*V24(I,3)
            DDRY=DT05*EXZ*AREA_I(I)
            DDRX=DT05*EYZ*AREA_I(I)
            V13X = V13(I,1)
            V24X = V24(I,1)
            VHIX = VHI(I,1)
            IF (ABS(X13(I)-X24(I)) < EM10) THEN
              DDRZ1 = ZERO
            ELSE
              DDRZ1 = DT025*(V13(I,2)-V24(I,2))/(X13(I)-X24(I))
            ENDIF
            V13(I,1) = V13(I,1)-DDRY*V13(I,3)-DDRZ1*V13(I,2)
            V24(I,1) = V24(I,1)-DDRY*V24(I,3)-DDRZ1*V24(I,2)
            VHI(I,1) = VHI(I,1)-DDRY*VHI(I,3)-DDRZ1*VHI(I,2)
            IF (ABS(Y13(I)+Y24(I)) < EM10) THEN
              DDRZ2 = ZERO
            ELSE
              DDRZ2 = DT025*(V13X+V24X)/(Y13(I)+Y24(I))
            ENDIF
            V13(I,2) = V13(I,2)-DDRX*V13(I,3)-DDRZ2*V13X
            V24(I,2) = V24(I,2)-DDRX*V24(I,3)-DDRZ2*V24X
            VHI(I,2) = VHI(I,2)-DDRX*VHI(I,3)-DDRZ2*VHIX
          ENDDO
        ENDIF
C--------------------------
C------- 3 ROTATIONS to 2 ROTATIONS----
C--------------------------
        IF (IVECTOR == 1)THEN
        ELSE
          IF(IXFEM==0)THEN
            CALL CZCORP5(NUMNOD ,JLT    ,NUMELC ,VR     ,NPT    ,TOL    ,
     2                   IXC    ,PLAT   ,AREA   ,AREA_I ,V13    ,
     3                   V24    ,VHI    ,RLXYZ ,VQN    ,VQ     ,
     4                   X13    ,X24    ,Y13    ,Y24    ,MX13   ,
     6                   MY13   ,
     7                   Z1     ,DI     ,DB     ,COREL  ,RLZ    ,
     8                   LM     ,
     9                   L13    ,L24    ,IDRIL  ,DIZ    )

          ELSE IF(IXFEM==1)THEN
            CALL CZCORP5X(JFT    ,JLT    ,VR     ,NPT    ,TOL    ,
     2                    IXC    ,PLAT   ,AREA   ,AREA_I ,V13    ,
     3                    V24    ,VHI    ,RLXYZ  ,VQN    ,VQ     ,
     4                    X13    ,X24    ,Y13    ,Y24    ,MX13   ,
     6                    MX23   ,MX34   ,MY13   ,MY23   ,MY34   ,
     7                    Z1     ,DI     ,DB     ,COREL  ,RLZ    ,
     8                    LM     ,X13_2  ,Y13_2  ,X24_2  ,Y24_2  ,
     9                    L13    ,L24    ,VRX1   ,VRX2   ,VRX3   ,
     A                    VRX4   ,VRY1   ,VRY2   ,VRY3   ,VRY4   ,
     B                    VRZ1   ,VRZ2   ,VRZ3   ,VRZ4   )
          END IF
        END IF !IF (IVECTOR == 1)THEN
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
        DO I=JFT,JLT
          V13(I,1)=V13(I,1)*AREA_I(I)
          V24(I,1)=V24(I,1)*AREA_I(I)
          VHI(I,1)=VHI(I,1)*FOURTH
C
          V13(I,2)=V13(I,2)*AREA_I(I)
          V24(I,2)=V24(I,2)*AREA_I(I)
          VHI(I,2)=VHI(I,2)*FOURTH
C
          V13(I,3)=V13(I,3)*AREA_I(I)
          V24(I,3)=V24(I,3)*AREA_I(I)
          VHI(I,3)=VHI(I,3)*FOURTH
        ENDDO
C
        RETURN
      END
      SUBROUTINE CZCORP5X(JFT    ,JLT    ,VR     ,NPT    ,TOL    ,
     2                    IXC    ,PLAT   ,AREA   ,AREA_I ,V13    ,
     3                    V24    ,VHI    ,RLXYZ  ,VQN    ,VQ     ,
     4                    X13    ,X24    ,Y13    ,Y24    ,MX13   ,
     6                    MX23   ,MX34   ,MY13   ,MY23   ,MY34   ,
     7                    Z1     ,DI     ,DB     ,COREL  ,RLZ    ,
     8                    LL     ,X13_2  ,Y13_2  ,X24_2  ,Y24_2  ,
     9                    L13    ,L24    ,VRX1   ,VRX2   ,VRX3   ,
     A                    VRX4   ,VRY1   ,VRY2   ,VRY3   ,VRY4   ,
     B                    VRZ1   ,VRZ2   ,VRZ3   ,VRZ4   )
      use element_mod , only : nixc
C-----------------------------------------------
C   I m p l i c i t   T y p e s
C-----------------------------------------------
#include      "implicit_f.inc"
C-----------------------------------------------
C   G l o b a l   P a r a m e t e r s
C-----------------------------------------------
#include      "mvsiz_p.inc"
#include      "impl1_c.inc"
#include      "scr05_c.inc"
C-----------------------------------------------
C   D u m m y   A r g u m e n t s
C-----------------------------------------------
        LOGICAL PLAT(*)
        INTEGER IXC(NIXC,*),JFT,JLT,NPT
        my_real
     .     VR(3,*),V13(MVSIZ,3),V24(MVSIZ,3),VHI(MVSIZ,3),
     .     MX23(*),MY13(*),MY23(*),MY34(*),
     .     X13(*),X24(*),Y13(*),Y24(*),MX13(*),
     .     VQ(MVSIZ,3,3),AREA(*),Z1(*),MX34(*),VQN(MVSIZ,3,4),AREA_I(*),
     .     DI(MVSIZ,6),DB(MVSIZ,3,4),LL(*),L13(*),L24(*),
     .     TOL,X13_2(*),Y13_2(*),X24_2(*),Y24_2(*),
     .     RLXYZ(MVSIZ,2,4),COREL(MVSIZ,2,4),RLZ(MVSIZ,4),
     .     VRX1(*),VRX2(*),VRX3(*),VRX4(*),
     .     VRY1(*),VRY2(*),VRY3(*),VRY4(*),
     .     VRZ1(*),VRZ2(*),VRZ3(*),VRZ4(*)
C-----------------------------------------------
C   L O C A L   V A R I A B L E S
C-----------------------------------------------
        INTEGER NNOD,I,J,K
        PARAMETER (NNOD = 4)
        my_real
     .     DETA,
     .     RRXYZ(3,NNOD),
     .     Z2(MVSIZ),A_4,SZ,SZ1,SZ2,SL,C1,
     .     AR(3),AD(NNOD),BTB(6),XX,YY,ZZ,XY,XZ,YZ,
     .     ABC,XXYZ2,YYXZ2,ZZXY2,D(6),DIZ2(6),
     .     ALR(3),ALD(NNOD),DBAD(3)
C=======================================================================
        DO I=JFT,JLT
          Z2(I)=Z1(I)*Z1(I)
          IF (Z2(I)<LL(I)*TOL.OR.NPT == 1) THEN
            Z1(I)=ZERO
            PLAT(I)=.TRUE.
          ELSE
            PLAT(I)=.FALSE.
C--------------------------------------------------
C WARPING SPECIAL TREATMENT
C full projection eliminer drilling rotations and rigid rotations
C--------------------------------------------------------------------------
            A_4=AREA(I)*FOURTH
C
C----------  node N ----------
            SZ1=MX13(I)*Y24(I)-MY13(I)*X24(I)
            SZ2=A_4+SZ1
            SZ=Z2(I)*L24(I)
            SL=ONE/SQRT(SZ+SZ2*SZ2)
            VQN(I,1,1)=-Z1(I)*Y24(I)
            VQN(I,2,1)= Z1(I)*X24(I)
            VQN(I,3,1)= SZ2*SL
            VQN(I,1,3)=-VQN(I,1,1)
            VQN(I,2,3)=-VQN(I,2,1)
            VQN(I,1,1)= VQN(I,1,1)*SL
            VQN(I,2,1)= VQN(I,2,1)*SL
C
            SZ2=A_4-SZ1
            SL=ONE/SQRT(SZ+SZ2*SZ2)
            VQN(I,1,3)= VQN(I,1,3)*SL
            VQN(I,2,3)= VQN(I,2,3)*SL
            VQN(I,3,3)= SZ2*SL
C
            SZ1=MX13(I)*Y13(I)-MY13(I)*X13(I)
            SZ2=A_4+SZ1
            SZ=Z2(I)*L13(I)
            SL=ONE/SQRT(SZ+SZ2*SZ2)
            VQN(I,1,2)=-Z1(I)*Y13(I)
            VQN(I,2,2)= Z1(I)*X13(I)
            VQN(I,3,2)= SZ2*SL
            VQN(I,1,4)=-VQN(I,1,2)
            VQN(I,2,4)=-VQN(I,2,2)
            VQN(I,1,2)= VQN(I,1,2)*SL
            VQN(I,2,2)= VQN(I,2,2)*SL
C
            SZ2=A_4-SZ1
            SL=ONE/SQRT(SZ+SZ2*SZ2)
            VQN(I,1,4)= VQN(I,1,4)*SL
            VQN(I,2,4)= VQN(I,2,4)*SL
            VQN(I,3,4)= SZ2*SL
C
            K=IXC(2,I)
            RRXYZ(1,1) =RLXYZ(I,1,1)
            RRXYZ(2,1) =RLXYZ(I,2,1)
            RRXYZ(3,1) =VQ(I,1,3)*VRX1(I)+VQ(I,2,3)*VRY1(I)
     1                 +VQ(I,3,3)*VRZ1(I)
            K=IXC(3,I)
            RRXYZ(1,2) =RLXYZ(I,1,2)
            RRXYZ(2,2) =RLXYZ(I,2,2)
            RRXYZ(3,2) =VQ(I,1,3)*VRX2(I)+VQ(I,2,3)*VRY2(I)
     1                 +VQ(I,3,3)*VRZ2(I)
            K=IXC(4,I)
            RRXYZ(1,3) =RLXYZ(I,1,3)
            RRXYZ(2,3) =RLXYZ(I,2,3)
            RRXYZ(3,3) =VQ(I,1,3)*VRX3(I)+VQ(I,2,3)*VRY3(I)
     1                 +VQ(I,3,3)*VRZ3(I)
            K=IXC(5,I)
            RRXYZ(1,4) =RLXYZ(I,1,4)
            RRXYZ(2,4) =RLXYZ(I,2,4)
            RRXYZ(3,4) =VQ(I,1,3)*VRX4(I)+VQ(I,2,3)*VRY4(I)
     1                 +VQ(I,3,3)*VRZ4(I)
            IF (IMPL_S>0.AND.IKPROJ<0) THEN
C-------------------------------------
C     DRILLING PROJECTION ONLY
C-------------------------------------
              RLXYZ(I,1,1)=(1.-VQN(I,1,1)*VQN(I,1,1))*RRXYZ(1,1)
     1                        -VQN(I,1,1)*VQN(I,2,1) *RRXYZ(2,1)
     2                        -VQN(I,1,1)*VQN(I,3,1) *RRXYZ(3,1)
              RLXYZ(I,2,1)=(1.-VQN(I,2,1)*VQN(I,2,1))*RRXYZ(2,1)
     1                        -VQN(I,1,1)*VQN(I,2,1) *RRXYZ(1,1)
     2                        -VQN(I,2,1)*VQN(I,3,1) *RRXYZ(3,1)
C
              RLXYZ(I,1,2)=(1.-VQN(I,1,2)*VQN(I,1,2))*RRXYZ(1,2)
     1                        -VQN(I,1,2)*VQN(I,2,2) *RRXYZ(2,2)
     2                        -VQN(I,1,2)*VQN(I,3,2) *RRXYZ(3,2)
              RLXYZ(I,2,2)=(1.-VQN(I,2,2)*VQN(I,2,2))*RRXYZ(2,2)
     1                        -VQN(I,1,2)*VQN(I,2,2) *RRXYZ(1,2)
     2                        -VQN(I,2,2)*VQN(I,3,2) *RRXYZ(3,2)
C
              RLXYZ(I,1,3)=(1.-VQN(I,1,3)*VQN(I,1,3))*RRXYZ(1,3)
     1                        -VQN(I,1,3)*VQN(I,2,3) *RRXYZ(2,3)
     2                        -VQN(I,1,3)*VQN(I,3,3) *RRXYZ(3,3)
              RLXYZ(I,2,3)=(1.-VQN(I,2,3)*VQN(I,2,3))*RRXYZ(2,3)
     1                        -VQN(I,1,3)*VQN(I,2,3) *RRXYZ(1,3)
     2                        -VQN(I,2,3)*VQN(I,3,3) *RRXYZ(3,3)
C
              RLXYZ(I,1,4)=(1.-VQN(I,1,4)*VQN(I,1,4))*RRXYZ(1,4)
     1                        -VQN(I,1,4)*VQN(I,2,4) *RRXYZ(2,4)
     2                        -VQN(I,1,4)*VQN(I,3,4) *RRXYZ(3,4)
              RLXYZ(I,2,4)=(1.-VQN(I,2,4)*VQN(I,2,4))*RRXYZ(2,4)
     1                        -VQN(I,1,4)*VQN(I,2,4) *RRXYZ(1,4)
     2                        -VQN(I,2,4)*VQN(I,3,4) *RRXYZ(3,4)
            ELSE
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
C-----------------------full projection------------------
              AR(1)=-Z1(I)*VHI(I,2)+Y13(I)*V13(I,3)+Y24(I)*V24(I,3)
     1               +MY13(I)*VHI(I,3)
     2               +RRXYZ(1,1)+RRXYZ(1,2)+RRXYZ(1,3)+RRXYZ(1,4)
              AR(2)= Z1(I)*VHI(I,1)-X13(I)*V13(I,3)-X24(I)*V24(I,3)
     1               -MX13(I)*VHI(I,3)
     2               +RRXYZ(2,1)+RRXYZ(2,2)+RRXYZ(2,3)+RRXYZ(2,4)
              AR(3)= X13(I)*V13(I,2)+X24(I)*V24(I,2)+MX13(I)*VHI(I,2)
     1              -Y13(I)*V13(I,1)-Y24(I)*V24(I,1)-MY13(I)*VHI(I,1)
     2               +RRXYZ(3,1)+RRXYZ(3,2)+RRXYZ(3,3)+RRXYZ(3,4)
              AD(1)= VQN(I,1,1)*RRXYZ(1,1)+VQN(I,2,1)*RRXYZ(2,1)+
     1               VQN(I,3,1)*RRXYZ(3,1)
              AD(2)= VQN(I,1,2)*RRXYZ(1,2)+VQN(I,2,2)*RRXYZ(2,2)+
     1               VQN(I,3,2)*RRXYZ(3,2)
              AD(3)= VQN(I,1,3)*RRXYZ(1,3)+VQN(I,2,3)*RRXYZ(2,3)+
     1               VQN(I,3,3)*RRXYZ(3,3)
              AD(4)= VQN(I,1,4)*RRXYZ(1,4)+VQN(I,2,4)*RRXYZ(2,4)+
     1               VQN(I,3,4)*RRXYZ(3,4)
C
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
              XX = COREL(I,1,1)*COREL(I,1,1)+COREL(I,1,2)*COREL(I,1,2)
     1            +COREL(I,1,3)*COREL(I,1,3)+COREL(I,1,4)*COREL(I,1,4)
              YY = COREL(I,2,1)*COREL(I,2,1)+COREL(I,2,2)*COREL(I,2,2)
     1            +COREL(I,2,3)*COREL(I,2,3)+COREL(I,2,4)*COREL(I,2,4)
              XY = COREL(I,1,1)*COREL(I,2,1)+COREL(I,1,2)*COREL(I,2,2)
     1            +COREL(I,1,3)*COREL(I,2,3)+COREL(I,1,4)*COREL(I,2,4)
              XZ =(COREL(I,1,1)-COREL(I,1,2)+COREL(I,1,3)-COREL(I,1,4))
     .                *Z1(I)
              YZ =(COREL(I,2,1)-COREL(I,2,2)+COREL(I,2,3)-COREL(I,2,4))
     .                *Z1(I)
              ZZ = FOUR*Z2(I)
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
              BTB(1)= VQN(I,1,1)*VQN(I,1,1)+VQN(I,1,2)*VQN(I,1,2)
     1               +VQN(I,1,3)*VQN(I,1,3)+VQN(I,1,4)*VQN(I,1,4)
              BTB(2)= VQN(I,2,1)*VQN(I,2,1)+VQN(I,2,2)*VQN(I,2,2)
     1               +VQN(I,2,3)*VQN(I,2,3)+VQN(I,2,4)*VQN(I,2,4)
              BTB(3)= VQN(I,3,1)*VQN(I,3,1)+VQN(I,3,2)*VQN(I,3,2)
     1               +VQN(I,3,3)*VQN(I,3,3)+VQN(I,3,4)*VQN(I,3,4)
              BTB(4)= VQN(I,1,1)*VQN(I,2,1)+VQN(I,1,2)*VQN(I,2,2)
     1               +VQN(I,1,3)*VQN(I,2,3)+VQN(I,1,4)*VQN(I,2,4)
              BTB(5)= VQN(I,1,1)*VQN(I,3,1)+VQN(I,1,2)*VQN(I,3,2)
     1               +VQN(I,1,3)*VQN(I,3,3)+VQN(I,1,4)*VQN(I,3,4)
              BTB(6)= VQN(I,2,1)*VQN(I,3,1)+VQN(I,2,2)*VQN(I,3,2)
     1               +VQN(I,2,3)*VQN(I,3,3)+VQN(I,2,4)*VQN(I,3,4)
              D(1)= YY+ZZ+FOUR-BTB(1)
              D(2)= XX+ZZ+FOUR-BTB(2)
              D(3)= XX+YY+FOUR-BTB(3)
              D(4)= -XY-BTB(4)
              D(5)= -XZ-BTB(5)
              D(6)= -YZ-BTB(6)
              IF(IRESP == 1)THEN
                CALL A3INVDP(D,DIZ2)
                DI(I,1:6) = DIZ2(1:6)
              ELSE
                ABC = D(1)*D(2)*D(3)
                XXYZ2 = D(1)*D(6)*D(6)
                YYXZ2 = D(2)*D(5)*D(5)
                ZZXY2 = D(3)*D(4)*D(4)
                DETA = ABS(ABC+TWO*D(4)*D(5)*D(6)-XXYZ2-YYXZ2-ZZXY2)
                DETA = ONE/MAX(DETA,EM20)
                DI(I,1) = (ABC-XXYZ2)*DETA/MAX(D(1),EM20)
                DI(I,2) = (ABC-YYXZ2)*DETA/MAX(D(2),EM20)
                DI(I,3) = (ABC-ZZXY2)*DETA/MAX(D(3),EM20)
                DI(I,4) = (D(5)*D(6)-D(4)*D(3))*DETA
                DI(I,5) = (D(6)*D(4)-D(5)*D(2))*DETA
                DI(I,6) = (D(4)*D(5)-D(6)*D(1))*DETA
              END IF !(IRESP == 1)THEN
              DO J=1,NNOD
                DB(I,1,J)= DI(I,1)*VQN(I,1,J)+DI(I,4)*VQN(I,2,J)
     1                    +DI(I,5)*VQN(I,3,J)
                DB(I,2,J)= DI(I,4)*VQN(I,1,J)+DI(I,2)*VQN(I,2,J)
     1                    +DI(I,6)*VQN(I,3,J)
                DB(I,3,J)= DI(I,5)*VQN(I,1,J)+DI(I,6)*VQN(I,2,J)
     1                    +DI(I,3)*VQN(I,3,J)
              ENDDO
C
              DBAD(1)= DB(I,1,1)*AD(1)+DB(I,1,2)*AD(2)
     1                +DB(I,1,3)*AD(3)+DB(I,1,4)*AD(4)
              DBAD(2)= DB(I,2,1)*AD(1)+DB(I,2,2)*AD(2)
     1                +DB(I,2,3)*AD(3)+DB(I,2,4)*AD(4)
              DBAD(3)= DB(I,3,1)*AD(1)+DB(I,3,2)*AD(2)
     1                +DB(I,3,3)*AD(3)+DB(I,3,4)*AD(4)
C
              ALR(1) =DI(I,1)*AR(1)+DI(I,4)*AR(2)+DI(I,5)*AR(3)-DBAD(1)
              ALR(2) =DI(I,4)*AR(1)+DI(I,2)*AR(2)+DI(I,6)*AR(3)-DBAD(2)
              ALR(3) =DI(I,5)*AR(1)+DI(I,6)*AR(2)+DI(I,3)*AR(3)-DBAD(3)
C
              ALD(1) = AD(1)+VQN(I,1,1)*DBAD(1)+VQN(I,2,1)*DBAD(2)
     1                +VQN(I,3,1)*DBAD(3)
     2                -DB(I,1,1)*AR(1)-DB(I,2,1)*AR(2)-DB(I,3,1)*AR(3)
              ALD(2) = AD(2)+VQN(I,1,2)*DBAD(1)+VQN(I,2,2)*DBAD(2)
     1                +VQN(I,3,2)*DBAD(3)
     2                -DB(I,1,2)*AR(1)-DB(I,2,2)*AR(2)-DB(I,3,2)*AR(3)
              ALD(3) = AD(3)+VQN(I,1,3)*DBAD(1)+VQN(I,2,3)*DBAD(2)
     1                +VQN(I,3,3)*DBAD(3)
     2                -DB(I,1,3)*AR(1)-DB(I,2,3)*AR(2)-DB(I,3,3)*AR(3)
              ALD(4) = AD(4)+VQN(I,1,4)*DBAD(1)+VQN(I,2,4)*DBAD(2)
     1                +VQN(I,3,4)*DBAD(3)
     2                -DB(I,1,4)*AR(1)-DB(I,2,4)*AR(2)-DB(I,3,4)*AR(3)
C
              C1 = TWO*ALR(3)
              V13(I,1)= V13(I,1)+C1*Y13(I)
              V24(I,1)= V24(I,1)+C1*Y24(I)
              VHI(I,1)= VHI(I,1)+FOUR*(ALR(3)*MY13(I)-Z1(I)*ALR(2))
              V13(I,2)= V13(I,2)-C1*X13(I)
              V24(I,2)= V24(I,2)-C1*X24(I)
              VHI(I,2)= VHI(I,2)-FOUR*(ALR(3)*MX13(I)-Z1(I)*ALR(1))
              V13(I,3)= V13(I,3)-TWO*(Y13(I)*ALR(1)-X13(I)*ALR(2))
              V24(I,3)= V24(I,3)-TWO*(Y24(I)*ALR(1)-X24(I)*ALR(2))
              VHI(I,3)= VHI(I,3)+FOUR*(MX13(I)*ALR(2)-MY13(I)*ALR(1))
              RLXYZ(I,1,1)= RRXYZ(1,1)-ALR(1)-VQN(I,1,1)*ALD(1)
              RLXYZ(I,1,2)= RRXYZ(1,2)-ALR(1)-VQN(I,1,2)*ALD(2)
              RLXYZ(I,1,3)= RRXYZ(1,3)-ALR(1)-VQN(I,1,3)*ALD(3)
              RLXYZ(I,1,4)= RRXYZ(1,4)-ALR(1)-VQN(I,1,4)*ALD(4)
C
              RLXYZ(I,2,1)= RRXYZ(2,1)-ALR(2)-VQN(I,2,1)*ALD(1)
              RLXYZ(I,2,2)= RRXYZ(2,2)-ALR(2)-VQN(I,2,2)*ALD(2)
              RLXYZ(I,2,3)= RRXYZ(2,3)-ALR(2)-VQN(I,2,3)*ALD(3)
              RLXYZ(I,2,4)= RRXYZ(2,4)-ALR(2)-VQN(I,2,4)*ALD(4)
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
            END IF !(IMPL_S>0.AND.IKPROJ<0) THEN
C
          ENDIF
        ENDDO
C
        RETURN
      END
      SUBROUTINE A3INVDP(D,DI)
C-----------------------------------------------
C   I m p l i c i t   T y p e s
C-----------------------------------------------
#include      "implicit_f.inc"
C-----------------------------------------------
C   D u m m y   A r g u m e n t s
C-----------------------------------------------
        my_real D(6), DI(6)
C-----------------------------------------------
C   L o c a l   V a r i a b l e s
C-----------------------------------------------
        DOUBLE PRECISION
     .     ABC,XXYZ2,YYXZ2,ZZXY2,DETA
C-----------------------------------------------
        ABC = D(1)*D(2)*D(3)
        XXYZ2 = D(1)*D(6)*D(6)
        YYXZ2 = D(2)*D(5)*D(5)
        ZZXY2 = D(3)*D(4)*D(4)
        DETA = ABS(ABC+TWO*D(4)*D(5)*D(6)-XXYZ2-YYXZ2-ZZXY2)
        DETA = ONE/MAX(DETA,EM20)
        DI(1) = (ABC-XXYZ2)*DETA/MAX(D(1),EM20)
        DI(2) = (ABC-YYXZ2)*DETA/MAX(D(2),EM20)
        DI(3) = (ABC-ZZXY2)*DETA/MAX(D(3),EM20)
        DI(4) = (D(5)*D(6)-D(4)*D(3))*DETA
        DI(5) = (D(6)*D(4)-D(5)*D(2))*DETA
        DI(6) = (D(4)*D(5)-D(6)*D(1))*DETA
C
        RETURN
      END
