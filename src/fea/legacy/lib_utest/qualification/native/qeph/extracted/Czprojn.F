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
      SUBROUTINE CZPROJN(
     1            JFT    ,JLT    ,VQN    ,VQ     ,VF    ,
     2            VM     ,PLAT   ,
     3            F11    ,F12    ,F13    ,F14    ,F21   ,
     4            F22    ,F23    ,F24    ,F31    ,F32   ,
     5            F33    ,F34    ,M11    ,M12    ,M13   ,
     6            M14    ,M21    ,M22    ,M23    ,M24   ,
     7            M31    ,M32    ,M33    ,M34    ,FZERO ,
     8            Z1     ,COREL  ,DI     ,DB     ,IFINI ,
     9            IDRIL  ,DIZ    ,VMZ    )
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
C        TRANSMET LES FORCES INTERNES LOCALES VF,VM ---> GLOBALES FIJ ,MIJ
C        ENTREES : 
C        SORTIES : FIJ,MIJ
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
#include      "implicit_f.inc"
#include      "mvsiz_p.inc"
#include      "impl1_c.inc"
C-----------------------------------------------
C   D U M M Y   A R G U M E N T S
C-----------------------------------------------
      LOGICAL PLAT(*)
      INTEGER JFT,JLT,IDRIL,IFINI
      my_real 
     .   VQN(MVSIZ,3,4),VF(MVSIZ,3,4),VM(MVSIZ,2,4),VQ(MVSIZ,3,3),
     .   COREL(MVSIZ,2,4),DI(MVSIZ,6),DB(MVSIZ,3,4),Z1(*)
      my_real
     .   F11(MVSIZ), F12(MVSIZ), F13(MVSIZ), F14(MVSIZ),
     .   F21(MVSIZ), F22(MVSIZ), F23(MVSIZ), F24(MVSIZ),
     .   F31(MVSIZ), F32(MVSIZ), F33(MVSIZ), F34(MVSIZ),
     .   M11(MVSIZ), M12(MVSIZ), M13(MVSIZ), M14(MVSIZ),
     .   M21(MVSIZ), M22(MVSIZ), M23(MVSIZ), M24(MVSIZ),
     .   M31(MVSIZ), M32(MVSIZ), M33(MVSIZ), M34(MVSIZ),
C
     .   FZERO(3,4,*),DIZ(MVSIZ,3),VMZ(MVSIZ,4)
C-----------------------------------------------
C   L O C A L   V A R I A B L E S
C-----------------------------------------------
      INTEGER  J, K
      my_real 
     .     MM(3,4),FL(3,4),ML(2,4),C1,
     .     AR(3),AD(4),ALR(3),ALD(4),DBAD(3),
     .       ARZ
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
      IF(IFINI > 0 )THEN
      DO K=JFT,JLT
C
C      I=1
       FL(1,1)= VF(K,1,1)+VF(K,1,3)
       FL(1,2)= VF(K,1,2)+VF(K,1,4)
       FL(1,3)=-VF(K,1,1)+VF(K,1,3)
       FL(1,4)=-VF(K,1,2)+VF(K,1,4)
C      I=2
       FL(2,1)= VF(K,2,1)+VF(K,2,3)
       FL(2,2)= VF(K,2,2)+VF(K,2,4)
       FL(2,3)=-VF(K,2,1)+VF(K,2,3)
       FL(2,4)=-VF(K,2,2)+VF(K,2,4)
C      I=3
       FL(3,1)= VF(K,3,1)+VF(K,3,3)
       FL(3,2)= VF(K,3,2)+VF(K,3,4)
       FL(3,3)=-VF(K,3,1)+VF(K,3,3)
       FL(3,4)=-VF(K,3,2)+VF(K,3,4)
C
C      I=1
       FL(1,1)=FL(1,1)+FZERO(1,1,K)
       FL(1,2)=FL(1,2)+FZERO(1,2,K)
       FL(1,3)=FL(1,3)+FZERO(1,3,K)
       FL(1,4)=FL(1,4)+FZERO(1,4,K)
C      I=2
       FL(2,1)=FL(2,1)+FZERO(2,1,K)
       FL(2,2)=FL(2,2)+FZERO(2,2,K)
       FL(2,3)=FL(2,3)+FZERO(2,3,K)
       FL(2,4)=FL(2,4)+FZERO(2,4,K)
C      I=3
       FL(3,1)= FL(3,1)+FZERO(3,1,K)
       FL(3,2)= FL(3,2)+FZERO(3,2,K)
       FL(3,3)= FL(3,3)+FZERO(3,3,K)
       FL(3,4)= FL(3,4)+FZERO(3,4,K)
C
C      I=1
       ML(1,1)= VM(K,1,1)+VM(K,1,3)
       ML(1,2)= VM(K,1,2)+VM(K,1,4)
       ML(1,3)=-VM(K,1,1)+VM(K,1,3)
       ML(1,4)=-VM(K,1,2)+VM(K,1,4)
C      I=2
       ML(2,1)= VM(K,2,1)+VM(K,2,3)
       ML(2,2)= VM(K,2,2)+VM(K,2,4)
       ML(2,3)=-VM(K,2,1)+VM(K,2,3)
       ML(2,4)=-VM(K,2,2)+VM(K,2,4)
C---------------------------------------
C   TRANS LOCAL-->GLOBAL ET 5DDL-->6DDL
C---------------------------------------
       IF (PLAT(K)) THEN
C
C        J=1
C        I=1
         F11(K)= VQ(K,1,1)*FL(1,1)+VQ(K,1,2)*FL(2,1)+VQ(K,1,3)*FL(3,1)
         M11(K)= VQ(K,1,1)*ML(1,1)+VQ(K,1,2)*ML(2,1)
C        I=2
         F21(K)= VQ(K,2,1)*FL(1,1)+VQ(K,2,2)*FL(2,1)+VQ(K,2,3)*FL(3,1)
         M21(K)= VQ(K,2,1)*ML(1,1)+VQ(K,2,2)*ML(2,1)
C        I=3
         F31(K)= VQ(K,3,1)*FL(1,1)+VQ(K,3,2)*FL(2,1)+VQ(K,3,3)*FL(3,1)
         M31(K)= VQ(K,3,1)*ML(1,1)+VQ(K,3,2)*ML(2,1)
C
C        J=2
C        I=1
         F12(K)= VQ(K,1,1)*FL(1,2)+VQ(K,1,2)*FL(2,2)+VQ(K,1,3)*FL(3,2)
         M12(K)= VQ(K,1,1)*ML(1,2)+VQ(K,1,2)*ML(2,2)
C        I=2
         F22(K)= VQ(K,2,1)*FL(1,2)+VQ(K,2,2)*FL(2,2)+VQ(K,2,3)*FL(3,2)
         M22(K)= VQ(K,2,1)*ML(1,2)+VQ(K,2,2)*ML(2,2)
C        I=3
         F32(K)= VQ(K,3,1)*FL(1,2)+VQ(K,3,2)*FL(2,2)+VQ(K,3,3)*FL(3,2)
         M32(K)= VQ(K,3,1)*ML(1,2)+VQ(K,3,2)*ML(2,2)
C
C        J=3
C        I=1
         F13(K)= VQ(K,1,1)*FL(1,3)+VQ(K,1,2)*FL(2,3)+VQ(K,1,3)*FL(3,3)
         M13(K)= VQ(K,1,1)*ML(1,3)+VQ(K,1,2)*ML(2,3)
C        I=2
         F23(K)= VQ(K,2,1)*FL(1,3)+VQ(K,2,2)*FL(2,3)+VQ(K,2,3)*FL(3,3)
         M23(K)= VQ(K,2,1)*ML(1,3)+VQ(K,2,2)*ML(2,3)
C        I=3
         F33(K)= VQ(K,3,1)*FL(1,3)+VQ(K,3,2)*FL(2,3)+VQ(K,3,3)*FL(3,3)
         M33(K)= VQ(K,3,1)*ML(1,3)+VQ(K,3,2)*ML(2,3)
C
C        J=4
C        I=1
         F14(K)= VQ(K,1,1)*FL(1,4)+VQ(K,1,2)*FL(2,4)+VQ(K,1,3)*FL(3,4)
         M14(K)= VQ(K,1,1)*ML(1,4)+VQ(K,1,2)*ML(2,4)
C        I=2
         F24(K)= VQ(K,2,1)*FL(1,4)+VQ(K,2,2)*FL(2,4)+VQ(K,2,3)*FL(3,4)
         M24(K)= VQ(K,2,1)*ML(1,4)+VQ(K,2,2)*ML(2,4)
C        I=3
         F34(K)= VQ(K,3,1)*FL(1,4)+VQ(K,3,2)*FL(2,4)+VQ(K,3,3)*FL(3,4)
         M34(K)= VQ(K,3,1)*ML(1,4)+VQ(K,3,2)*ML(2,4)
C         
         IF (IDRIL>0) THEN
          M11(K)= M11(K)+ VQ(K,1,3)*VMZ(K,1)
          M21(K)= M21(K)+ VQ(K,2,3)*VMZ(K,1)
          M31(K)= M31(K)+ VQ(K,3,3)*VMZ(K,1)
C
          M12(K)= M12(K)+ VQ(K,1,3)*VMZ(K,2)
          M22(K)= M22(K)+ VQ(K,2,3)*VMZ(K,2)
          M32(K)= M32(K)+ VQ(K,3,3)*VMZ(K,2)
C
          M13(K)= M13(K)+ VQ(K,1,3)*VMZ(K,3)
          M23(K)= M23(K)+ VQ(K,2,3)*VMZ(K,3)
          M33(K)= M33(K)+ VQ(K,3,3)*VMZ(K,3)
C
          M14(K)= M14(K)+ VQ(K,1,3)*VMZ(K,4)
          M24(K)= M24(K)+ VQ(K,2,3)*VMZ(K,4)
          M34(K)= M34(K)+ VQ(K,3,3)*VMZ(K,4)
         END IF
C---------warped element------------
      ELSE
       IF (IMPL_S>0.AND.IKPROJ<=0) THEN
C-------------------------------------
C     DRILLING RE-PROJECTION ONLY
C-------------------------------------
         MM(1,1)=(ONE-VQN(K,1,1)*VQN(K,1,1))*ML(1,1)-
     1                VQN(K,1,1)*VQN(K,2,1) *ML(2,1)
         MM(2,1)=(ONE-VQN(K,2,1)*VQN(K,2,1))*ML(2,1)-
     1                VQN(K,1,1)*VQN(K,2,1) *ML(1,1)
         MM(3,1)=    -VQN(K,1,1)*VQN(K,3,1) *ML(1,1)-
     1                VQN(K,2,1)*VQN(K,3,1) *ML(2,1)
C
C        J=2
         MM(1,2)=(ONE - VQN(K,1,2)*VQN(K,1,2))*ML(1,2)-
     1                VQN(K,1,2)*VQN(K,2,2) *ML(2,2)
         MM(2,2)=(ONE - VQN(K,2,2)*VQN(K,2,2))*ML(2,2)-
     1                VQN(K,1,2)*VQN(K,2,2) *ML(1,2)
         MM(3,2)=    -VQN(K,1,2)*VQN(K,3,2) *ML(1,2)-
     1                VQN(K,2,2)*VQN(K,3,2) *ML(2,2)
C
C        J=3
         MM(1,3)=(ONE-VQN(K,1,3)*VQN(K,1,3))*ML(1,3)-
     1                VQN(K,1,3)*VQN(K,2,3) *ML(2,3)
         MM(2,3)=(ONE-VQN(K,2,3)*VQN(K,2,3))*ML(2,3)-
     1                VQN(K,1,3)*VQN(K,2,3) *ML(1,3)
         MM(3,3)=    -VQN(K,1,3)*VQN(K,3,3) *ML(1,3)-
     1                VQN(K,2,3)*VQN(K,3,3) *ML(2,3)
C
C        J=4
         MM(1,4)=(ONE-VQN(K,1,4)*VQN(K,1,4))*ML(1,4)-
     1                VQN(K,1,4)*VQN(K,2,4) *ML(2,4)
         MM(2,4)=(ONE-VQN(K,2,4)*VQN(K,2,4))*ML(2,4)-
     1                VQN(K,1,4)*VQN(K,2,4) *ML(1,4)
         MM(3,4)=    -VQN(K,1,4)*VQN(K,3,4) *ML(1,4)-
     1                VQN(K,2,4)*VQN(K,3,4) *ML(2,4)
         IF (IDRIL>0) THEN
          DO J=1,4
           MM(1,J)=MM(1,J)+ VQN(K,1,J)*VMZ(K,J)
           MM(2,J)=MM(2,J)+ VQN(K,2,J)*VMZ(K,J)
           MM(3,J)=MM(3,J)+ VQN(K,3,J)*VMZ(K,J)
          END DO !J=1,4
         END IF
       ELSE
C----REPROJECTION(full)------
        AR(1)= -Z1(K)*(FL(2,1)-FL(2,2)+FL(2,3)-FL(2,4))
     1         +COREL(K,2,1)*FL(3,1)+ML(1,1)
     2         +COREL(K,2,2)*FL(3,2)+ML(1,2)
     3         +COREL(K,2,3)*FL(3,3)+ML(1,3)
     4         +COREL(K,2,4)*FL(3,4)+ML(1,4)
        AR(2)=  Z1(K)*(FL(1,1)-FL(1,2)+FL(1,3)-FL(1,4))
     1          -COREL(K,1,1)*FL(3,1)+ML(2,1)
     2          -COREL(K,1,2)*FL(3,2)+ML(2,2)
     3          -COREL(K,1,3)*FL(3,3)+ML(2,3)
     4          -COREL(K,1,4)*FL(3,4)+ML(2,4)
        AR(3)=-COREL(K,2,1)*FL(1,1)+COREL(K,1,1)*FL(2,1)
     1        -COREL(K,2,2)*FL(1,2)+COREL(K,1,2)*FL(2,2)
     2        -COREL(K,2,3)*FL(1,3)+COREL(K,1,3)*FL(2,3)
     3        -COREL(K,2,4)*FL(1,4)+COREL(K,1,4)*FL(2,4)
        AD(1)= VQN(K,1,1)*ML(1,1)+VQN(K,2,1)*ML(2,1)
        AD(2)= VQN(K,1,2)*ML(1,2)+VQN(K,2,2)*ML(2,2)
        AD(3)= VQN(K,1,3)*ML(1,3)+VQN(K,2,3)*ML(2,3)
        AD(4)= VQN(K,1,4)*ML(1,4)+VQN(K,2,4)*ML(2,4)
C     
         DBAD(1)= DB(K,1,1)*AD(1)+DB(K,1,2)*AD(2)
     1           +DB(K,1,3)*AD(3)+DB(K,1,4)*AD(4)
         DBAD(2)= DB(K,2,1)*AD(1)+DB(K,2,2)*AD(2)
     1           +DB(K,2,3)*AD(3)+DB(K,2,4)*AD(4)
         DBAD(3)= DB(K,3,1)*AD(1)+DB(K,3,2)*AD(2)
     1           +DB(K,3,3)*AD(3)+DB(K,3,4)*AD(4)
C
          ALR(1) =DI(K,1)*AR(1)+DI(K,4)*AR(2)+DI(K,5)*AR(3)-DBAD(1)
          ALR(2) =DI(K,4)*AR(1)+DI(K,2)*AR(2)+DI(K,6)*AR(3)-DBAD(2)
          ALR(3) =DI(K,5)*AR(1)+DI(K,6)*AR(2)+DI(K,3)*AR(3)-DBAD(3)
CC
          ALD(1) = AD(1)+VQN(K,1,1)*DBAD(1)+VQN(K,2,1)*DBAD(2)
     1            +VQN(K,3,1)*DBAD(3)
     2            -DB(K,1,1)*AR(1)-DB(K,2,1)*AR(2)-DB(K,3,1)*AR(3)
          ALD(2) = AD(2)+VQN(K,1,2)*DBAD(1)+VQN(K,2,2)*DBAD(2)
     1            +VQN(K,3,2)*DBAD(3)
     2            -DB(K,1,2)*AR(1)-DB(K,2,2)*AR(2)-DB(K,3,2)*AR(3)
          ALD(3) = AD(3)+VQN(K,1,3)*DBAD(1)+VQN(K,2,3)*DBAD(2)
     1            +VQN(K,3,3)*DBAD(3)
     2            -DB(K,1,3)*AR(1)-DB(K,2,3)*AR(2)-DB(K,3,3)*AR(3)
          ALD(4) = AD(4)+VQN(K,1,4)*DBAD(1)+VQN(K,2,4)*DBAD(2)
     1            +VQN(K,3,4)*DBAD(3)
     2            -DB(K,1,4)*AR(1)-DB(K,2,4)*AR(2)-DB(K,3,4)*AR(3)
         IF (IDRIL>0) THEN
          ARZ = VMZ(K,1)+VMZ(K,2)+VMZ(K,3)+VMZ(K,4)
          ALR(1) =ALR(1)+DIZ(K,1)*ARZ
          ALR(2) =ALR(2)+DIZ(K,2)*ARZ
          ALR(3) =ALR(3)+DIZ(K,3)*ARZ
         END IF !(IDRIL>0) THEN
C
          C1 =Z1(K)*ALR(2)
          FL(1,1)= FL(1,1)-C1+COREL(K,2,1)*ALR(3)
          FL(1,2)= FL(1,2)+C1+COREL(K,2,2)*ALR(3)
          FL(1,3)= FL(1,3)-C1+COREL(K,2,3)*ALR(3)
          FL(1,4)= FL(1,4)+C1+COREL(K,2,4)*ALR(3)
C
          C1 =Z1(K)*ALR(1)
          FL(2,1)= FL(2,1)+C1-COREL(K,1,1)*ALR(3)
          FL(2,2)= FL(2,2)-C1-COREL(K,1,2)*ALR(3)
          FL(2,3)= FL(2,3)+C1-COREL(K,1,3)*ALR(3)
          FL(2,4)= FL(2,4)-C1-COREL(K,1,4)*ALR(3)
C
         DO J=1,4
          FL(3,J)= FL(3,J)-COREL(K,2,J)*ALR(1)+COREL(K,1,J)*ALR(2)
          MM(1,J)= ML(1,J)-ALR(1)-VQN(K,1,J)*ALD(J)
          MM(2,J)= ML(2,J)-ALR(2)-VQN(K,2,J)*ALD(J)
          MM(3,J)=        -ALR(3)-VQN(K,3,J)*ALD(J)
         ENDDO
C         
         IF (IDRIL>0) THEN
          DO J=1,4
           MM(3,J)= MM(3,J)+VMZ(K,J)
          ENDDO
         END IF !(IDRIL>0) THEN
       END IF !((IMPL_S>0.AND.IKPROJ<0).OR.IDRIL>0) THEN
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
C        J=1
C        I=1
         F11(K)= VQ(K,1,1)*FL(1,1)+VQ(K,1,2)*FL(2,1)+VQ(K,1,3)*FL(3,1)
         M11(K)= VQ(K,1,1)*MM(1,1)+VQ(K,1,2)*MM(2,1)+VQ(K,1,3)*MM(3,1)
C        I=2
         F21(K)= VQ(K,2,1)*FL(1,1)+VQ(K,2,2)*FL(2,1)+VQ(K,2,3)*FL(3,1)
         M21(K)= VQ(K,2,1)*MM(1,1)+VQ(K,2,2)*MM(2,1)+VQ(K,2,3)*MM(3,1)
C        I=3
         F31(K)= VQ(K,3,1)*FL(1,1)+VQ(K,3,2)*FL(2,1)+VQ(K,3,3)*FL(3,1)
         M31(K)= VQ(K,3,1)*MM(1,1)+VQ(K,3,2)*MM(2,1)+VQ(K,3,3)*MM(3,1)
C
C        J=2
C        I=1
         F12(K)= VQ(K,1,1)*FL(1,2)+VQ(K,1,2)*FL(2,2)+VQ(K,1,3)*FL(3,2)
         M12(K)= VQ(K,1,1)*MM(1,2)+VQ(K,1,2)*MM(2,2)+VQ(K,1,3)*MM(3,2)
C        I=2
         F22(K)= VQ(K,2,1)*FL(1,2)+VQ(K,2,2)*FL(2,2)+VQ(K,2,3)*FL(3,2)
         M22(K)= VQ(K,2,1)*MM(1,2)+VQ(K,2,2)*MM(2,2)+VQ(K,2,3)*MM(3,2)
C        I=3
         F32(K)= VQ(K,3,1)*FL(1,2)+VQ(K,3,2)*FL(2,2)+VQ(K,3,3)*FL(3,2)
         M32(K)= VQ(K,3,1)*MM(1,2)+VQ(K,3,2)*MM(2,2)+VQ(K,3,3)*MM(3,2)
C
C        J=3
C        I=1
         F13(K)= VQ(K,1,1)*FL(1,3)+VQ(K,1,2)*FL(2,3)+VQ(K,1,3)*FL(3,3)
         M13(K)= VQ(K,1,1)*MM(1,3)+VQ(K,1,2)*MM(2,3)+VQ(K,1,3)*MM(3,3)
C        I=2
         F23(K)= VQ(K,2,1)*FL(1,3)+VQ(K,2,2)*FL(2,3)+VQ(K,2,3)*FL(3,3)
         M23(K)= VQ(K,2,1)*MM(1,3)+VQ(K,2,2)*MM(2,3)+VQ(K,2,3)*MM(3,3)
C        I=3
         F33(K)= VQ(K,3,1)*FL(1,3)+VQ(K,3,2)*FL(2,3)+VQ(K,3,3)*FL(3,3)
         M33(K)= VQ(K,3,1)*MM(1,3)+VQ(K,3,2)*MM(2,3)+VQ(K,3,3)*MM(3,3)
C
C        J=4
C        I=1
         F14(K)= VQ(K,1,1)*FL(1,4)+VQ(K,1,2)*FL(2,4)+VQ(K,1,3)*FL(3,4)
         M14(K)= VQ(K,1,1)*MM(1,4)+VQ(K,1,2)*MM(2,4)+VQ(K,1,3)*MM(3,4)
C        I=2
         F24(K)= VQ(K,2,1)*FL(1,4)+VQ(K,2,2)*FL(2,4)+VQ(K,2,3)*FL(3,4)
         M24(K)= VQ(K,2,1)*MM(1,4)+VQ(K,2,2)*MM(2,4)+VQ(K,2,3)*MM(3,4)
C        I=3
         F34(K)= VQ(K,3,1)*FL(1,4)+VQ(K,3,2)*FL(2,4)+VQ(K,3,3)*FL(3,4)
         M34(K)= VQ(K,3,1)*MM(1,4)+VQ(K,3,2)*MM(2,4)+VQ(K,3,3)*MM(3,4)
C
       ENDIF
C
      ENDDO
C-----case w/o initial F --------------
C
      ELSE
      DO K=JFT,JLT
C
C      I=1
       FL(1,1)= VF(K,1,1)+VF(K,1,3)
       FL(1,2)= VF(K,1,2)+VF(K,1,4)
       FL(1,3)=-VF(K,1,1)+VF(K,1,3)
       FL(1,4)=-VF(K,1,2)+VF(K,1,4)
C      I=2
       FL(2,1)= VF(K,2,1)+VF(K,2,3)
       FL(2,2)= VF(K,2,2)+VF(K,2,4)
       FL(2,3)=-VF(K,2,1)+VF(K,2,3)
       FL(2,4)=-VF(K,2,2)+VF(K,2,4)
C      I=3
       FL(3,1)= VF(K,3,1)+VF(K,3,3)
       FL(3,2)= VF(K,3,2)+VF(K,3,4)
       FL(3,3)=-VF(K,3,1)+VF(K,3,3)
       FL(3,4)=-VF(K,3,2)+VF(K,3,4)
C
C      I=1
       ML(1,1)= VM(K,1,1)+VM(K,1,3)
       ML(1,2)= VM(K,1,2)+VM(K,1,4)
       ML(1,3)=-VM(K,1,1)+VM(K,1,3)
       ML(1,4)=-VM(K,1,2)+VM(K,1,4)
C      I=2
       ML(2,1)= VM(K,2,1)+VM(K,2,3)
       ML(2,2)= VM(K,2,2)+VM(K,2,4)
       ML(2,3)=-VM(K,2,1)+VM(K,2,3)
       ML(2,4)=-VM(K,2,2)+VM(K,2,4)
C---------------------------------------
C   TRANS LOCAL-->GLOBAL ET 5DDL-->6DDL
C---------------------------------------
       IF (PLAT(K)) THEN
C
C        J=1
C        I=1
         F11(K)= VQ(K,1,1)*FL(1,1)+VQ(K,1,2)*FL(2,1)+VQ(K,1,3)*FL(3,1)
         M11(K)= VQ(K,1,1)*ML(1,1)+VQ(K,1,2)*ML(2,1)
C        I=2
         F21(K)= VQ(K,2,1)*FL(1,1)+VQ(K,2,2)*FL(2,1)+VQ(K,2,3)*FL(3,1)
         M21(K)= VQ(K,2,1)*ML(1,1)+VQ(K,2,2)*ML(2,1)
C        I=3
         F31(K)= VQ(K,3,1)*FL(1,1)+VQ(K,3,2)*FL(2,1)+VQ(K,3,3)*FL(3,1)
         M31(K)= VQ(K,3,1)*ML(1,1)+VQ(K,3,2)*ML(2,1)
C
C        J=2
C        I=1
         F12(K)= VQ(K,1,1)*FL(1,2)+VQ(K,1,2)*FL(2,2)+VQ(K,1,3)*FL(3,2)
         M12(K)= VQ(K,1,1)*ML(1,2)+VQ(K,1,2)*ML(2,2)
C        I=2
         F22(K)= VQ(K,2,1)*FL(1,2)+VQ(K,2,2)*FL(2,2)+VQ(K,2,3)*FL(3,2)
         M22(K)= VQ(K,2,1)*ML(1,2)+VQ(K,2,2)*ML(2,2)
C        I=3
         F32(K)= VQ(K,3,1)*FL(1,2)+VQ(K,3,2)*FL(2,2)+VQ(K,3,3)*FL(3,2)
         M32(K)= VQ(K,3,1)*ML(1,2)+VQ(K,3,2)*ML(2,2)
C
C        J=3
C        I=1
         F13(K)= VQ(K,1,1)*FL(1,3)+VQ(K,1,2)*FL(2,3)+VQ(K,1,3)*FL(3,3)
         M13(K)= VQ(K,1,1)*ML(1,3)+VQ(K,1,2)*ML(2,3)
C        I=2
         F23(K)= VQ(K,2,1)*FL(1,3)+VQ(K,2,2)*FL(2,3)+VQ(K,2,3)*FL(3,3)
         M23(K)= VQ(K,2,1)*ML(1,3)+VQ(K,2,2)*ML(2,3)
C        I=3
         F33(K)= VQ(K,3,1)*FL(1,3)+VQ(K,3,2)*FL(2,3)+VQ(K,3,3)*FL(3,3)
         M33(K)= VQ(K,3,1)*ML(1,3)+VQ(K,3,2)*ML(2,3)
C
C        J=4
C        I=1
         F14(K)= VQ(K,1,1)*FL(1,4)+VQ(K,1,2)*FL(2,4)+VQ(K,1,3)*FL(3,4)
         M14(K)= VQ(K,1,1)*ML(1,4)+VQ(K,1,2)*ML(2,4)
C        I=2
         F24(K)= VQ(K,2,1)*FL(1,4)+VQ(K,2,2)*FL(2,4)+VQ(K,2,3)*FL(3,4)
         M24(K)= VQ(K,2,1)*ML(1,4)+VQ(K,2,2)*ML(2,4)
C        I=3
         F34(K)= VQ(K,3,1)*FL(1,4)+VQ(K,3,2)*FL(2,4)+VQ(K,3,3)*FL(3,4)
         M34(K)= VQ(K,3,1)*ML(1,4)+VQ(K,3,2)*ML(2,4)
C         
         IF (IDRIL>0) THEN
          M11(K)= M11(K)+ VQ(K,1,3)*VMZ(K,1)
          M21(K)= M21(K)+ VQ(K,2,3)*VMZ(K,1)
          M31(K)= M31(K)+ VQ(K,3,3)*VMZ(K,1)
C
          M12(K)= M12(K)+ VQ(K,1,3)*VMZ(K,2)
          M22(K)= M22(K)+ VQ(K,2,3)*VMZ(K,2)
          M32(K)= M32(K)+ VQ(K,3,3)*VMZ(K,2)
C
          M13(K)= M13(K)+ VQ(K,1,3)*VMZ(K,3)
          M23(K)= M23(K)+ VQ(K,2,3)*VMZ(K,3)
          M33(K)= M33(K)+ VQ(K,3,3)*VMZ(K,3)
C
          M14(K)= M14(K)+ VQ(K,1,3)*VMZ(K,4)
          M24(K)= M24(K)+ VQ(K,2,3)*VMZ(K,4)
          M34(K)= M34(K)+ VQ(K,3,3)*VMZ(K,4)
         END IF
C
      ELSE
       IF (IMPL_S>0.AND.IKPROJ<=0) THEN
C-------------------------------------
C     DRILLING RE-PROJECTION ONLY
C-------------------------------------
         MM(1,1)=(ONE-VQN(K,1,1)*VQN(K,1,1))*ML(1,1)-
     1                VQN(K,1,1)*VQN(K,2,1) *ML(2,1)
         MM(2,1)=(ONE-VQN(K,2,1)*VQN(K,2,1))*ML(2,1)-
     1                VQN(K,1,1)*VQN(K,2,1) *ML(1,1)
         MM(3,1)=    -VQN(K,1,1)*VQN(K,3,1) *ML(1,1)-
     1                VQN(K,2,1)*VQN(K,3,1) *ML(2,1)
C
C        J=2
         MM(1,2)=(ONE - VQN(K,1,2)*VQN(K,1,2))*ML(1,2)-
     1                VQN(K,1,2)*VQN(K,2,2) *ML(2,2)
         MM(2,2)=(ONE - VQN(K,2,2)*VQN(K,2,2))*ML(2,2)-
     1                VQN(K,1,2)*VQN(K,2,2) *ML(1,2)
         MM(3,2)=    -VQN(K,1,2)*VQN(K,3,2) *ML(1,2)-
     1                VQN(K,2,2)*VQN(K,3,2) *ML(2,2)
C
C        J=3
         MM(1,3)=(ONE-VQN(K,1,3)*VQN(K,1,3))*ML(1,3)-
     1                VQN(K,1,3)*VQN(K,2,3) *ML(2,3)
         MM(2,3)=(ONE-VQN(K,2,3)*VQN(K,2,3))*ML(2,3)-
     1                VQN(K,1,3)*VQN(K,2,3) *ML(1,3)
         MM(3,3)=    -VQN(K,1,3)*VQN(K,3,3) *ML(1,3)-
     1                VQN(K,2,3)*VQN(K,3,3) *ML(2,3)
C
C        J=4
         MM(1,4)=(ONE-VQN(K,1,4)*VQN(K,1,4))*ML(1,4)-
     1                VQN(K,1,4)*VQN(K,2,4) *ML(2,4)
         MM(2,4)=(ONE-VQN(K,2,4)*VQN(K,2,4))*ML(2,4)-
     1                VQN(K,1,4)*VQN(K,2,4) *ML(1,4)
         MM(3,4)=    -VQN(K,1,4)*VQN(K,3,4) *ML(1,4)-
     1                VQN(K,2,4)*VQN(K,3,4) *ML(2,4)
         IF (IDRIL>0) THEN
          DO J=1,4
           MM(1,J)=MM(1,J)+ VQN(K,1,J)*VMZ(K,J)
           MM(2,J)=MM(2,J)+ VQN(K,2,J)*VMZ(K,J)
           MM(3,J)=MM(3,J)+ VQN(K,3,J)*VMZ(K,J)
          END DO !J=1,4
         END IF
       ELSE
C----REPROJECTION(full)------
        AR(1)= -Z1(K)*(FL(2,1)-FL(2,2)+FL(2,3)-FL(2,4))
     1         +COREL(K,2,1)*FL(3,1)+ML(1,1)
     2         +COREL(K,2,2)*FL(3,2)+ML(1,2)
     3         +COREL(K,2,3)*FL(3,3)+ML(1,3)
     4         +COREL(K,2,4)*FL(3,4)+ML(1,4)
        AR(2)=  Z1(K)*(FL(1,1)-FL(1,2)+FL(1,3)-FL(1,4))
     1          -COREL(K,1,1)*FL(3,1)+ML(2,1)
     2          -COREL(K,1,2)*FL(3,2)+ML(2,2)
     3          -COREL(K,1,3)*FL(3,3)+ML(2,3)
     4          -COREL(K,1,4)*FL(3,4)+ML(2,4)
        AR(3)=-COREL(K,2,1)*FL(1,1)+COREL(K,1,1)*FL(2,1)
     1        -COREL(K,2,2)*FL(1,2)+COREL(K,1,2)*FL(2,2)
     2        -COREL(K,2,3)*FL(1,3)+COREL(K,1,3)*FL(2,3)
     3        -COREL(K,2,4)*FL(1,4)+COREL(K,1,4)*FL(2,4)
        AD(1)= VQN(K,1,1)*ML(1,1)+VQN(K,2,1)*ML(2,1)
        AD(2)= VQN(K,1,2)*ML(1,2)+VQN(K,2,2)*ML(2,2)
        AD(3)= VQN(K,1,3)*ML(1,3)+VQN(K,2,3)*ML(2,3)
        AD(4)= VQN(K,1,4)*ML(1,4)+VQN(K,2,4)*ML(2,4)
C     
         DBAD(1)= DB(K,1,1)*AD(1)+DB(K,1,2)*AD(2)
     1           +DB(K,1,3)*AD(3)+DB(K,1,4)*AD(4)
         DBAD(2)= DB(K,2,1)*AD(1)+DB(K,2,2)*AD(2)
     1           +DB(K,2,3)*AD(3)+DB(K,2,4)*AD(4)
         DBAD(3)= DB(K,3,1)*AD(1)+DB(K,3,2)*AD(2)
     1           +DB(K,3,3)*AD(3)+DB(K,3,4)*AD(4)
C
          ALR(1) =DI(K,1)*AR(1)+DI(K,4)*AR(2)+DI(K,5)*AR(3)-DBAD(1)
          ALR(2) =DI(K,4)*AR(1)+DI(K,2)*AR(2)+DI(K,6)*AR(3)-DBAD(2)
          ALR(3) =DI(K,5)*AR(1)+DI(K,6)*AR(2)+DI(K,3)*AR(3)-DBAD(3)
CC
          ALD(1) = AD(1)+VQN(K,1,1)*DBAD(1)+VQN(K,2,1)*DBAD(2)
     1            +VQN(K,3,1)*DBAD(3)
     2            -DB(K,1,1)*AR(1)-DB(K,2,1)*AR(2)-DB(K,3,1)*AR(3)
          ALD(2) = AD(2)+VQN(K,1,2)*DBAD(1)+VQN(K,2,2)*DBAD(2)
     1            +VQN(K,3,2)*DBAD(3)
     2            -DB(K,1,2)*AR(1)-DB(K,2,2)*AR(2)-DB(K,3,2)*AR(3)
          ALD(3) = AD(3)+VQN(K,1,3)*DBAD(1)+VQN(K,2,3)*DBAD(2)
     1            +VQN(K,3,3)*DBAD(3)
     2            -DB(K,1,3)*AR(1)-DB(K,2,3)*AR(2)-DB(K,3,3)*AR(3)
          ALD(4) = AD(4)+VQN(K,1,4)*DBAD(1)+VQN(K,2,4)*DBAD(2)
     1            +VQN(K,3,4)*DBAD(3)
     2            -DB(K,1,4)*AR(1)-DB(K,2,4)*AR(2)-DB(K,3,4)*AR(3)
         IF (IDRIL>0) THEN
          ARZ = VMZ(K,1)+VMZ(K,2)+VMZ(K,3)+VMZ(K,4)
          ALR(1) =ALR(1)+DIZ(K,1)*ARZ
          ALR(2) =ALR(2)+DIZ(K,2)*ARZ
          ALR(3) =ALR(3)+DIZ(K,3)*ARZ
         END IF !(IDRIL>0) THEN
C
          C1 =Z1(K)*ALR(2)
          FL(1,1)= FL(1,1)-C1+COREL(K,2,1)*ALR(3)
          FL(1,2)= FL(1,2)+C1+COREL(K,2,2)*ALR(3)
          FL(1,3)= FL(1,3)-C1+COREL(K,2,3)*ALR(3)
          FL(1,4)= FL(1,4)+C1+COREL(K,2,4)*ALR(3)
C
          C1 =Z1(K)*ALR(1)
          FL(2,1)= FL(2,1)+C1-COREL(K,1,1)*ALR(3)
          FL(2,2)= FL(2,2)-C1-COREL(K,1,2)*ALR(3)
          FL(2,3)= FL(2,3)+C1-COREL(K,1,3)*ALR(3)
          FL(2,4)= FL(2,4)-C1-COREL(K,1,4)*ALR(3)
C
         DO J=1,4
          FL(3,J)= FL(3,J)-COREL(K,2,J)*ALR(1)+COREL(K,1,J)*ALR(2)
          MM(1,J)= ML(1,J)-ALR(1)-VQN(K,1,J)*ALD(J)
          MM(2,J)= ML(2,J)-ALR(2)-VQN(K,2,J)*ALD(J)
          MM(3,J)=        -ALR(3)-VQN(K,3,J)*ALD(J)
         ENDDO
C         
         IF (IDRIL>0) THEN
          DO J=1,4
           MM(3,J)= MM(3,J)+VMZ(K,J)
          ENDDO
         END IF !(IDRIL>0) THEN
C         
       END IF !((IMPL_S>0.AND.IKPROJ<0).OR.IDRIL>0) THEN
C---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8
C        J=1
C        I=1
         F11(K)= VQ(K,1,1)*FL(1,1)+VQ(K,1,2)*FL(2,1)+VQ(K,1,3)*FL(3,1)
         M11(K)= VQ(K,1,1)*MM(1,1)+VQ(K,1,2)*MM(2,1)+VQ(K,1,3)*MM(3,1)
C        I=2
         F21(K)= VQ(K,2,1)*FL(1,1)+VQ(K,2,2)*FL(2,1)+VQ(K,2,3)*FL(3,1)
         M21(K)= VQ(K,2,1)*MM(1,1)+VQ(K,2,2)*MM(2,1)+VQ(K,2,3)*MM(3,1)
C        I=3
         F31(K)= VQ(K,3,1)*FL(1,1)+VQ(K,3,2)*FL(2,1)+VQ(K,3,3)*FL(3,1)
         M31(K)= VQ(K,3,1)*MM(1,1)+VQ(K,3,2)*MM(2,1)+VQ(K,3,3)*MM(3,1)
C
C        J=2
C        I=1
         F12(K)= VQ(K,1,1)*FL(1,2)+VQ(K,1,2)*FL(2,2)+VQ(K,1,3)*FL(3,2)
         M12(K)= VQ(K,1,1)*MM(1,2)+VQ(K,1,2)*MM(2,2)+VQ(K,1,3)*MM(3,2)
C        I=2
         F22(K)= VQ(K,2,1)*FL(1,2)+VQ(K,2,2)*FL(2,2)+VQ(K,2,3)*FL(3,2)
         M22(K)= VQ(K,2,1)*MM(1,2)+VQ(K,2,2)*MM(2,2)+VQ(K,2,3)*MM(3,2)
C        I=3
         F32(K)= VQ(K,3,1)*FL(1,2)+VQ(K,3,2)*FL(2,2)+VQ(K,3,3)*FL(3,2)
         M32(K)= VQ(K,3,1)*MM(1,2)+VQ(K,3,2)*MM(2,2)+VQ(K,3,3)*MM(3,2)
C
C        J=3
C        I=1
         F13(K)= VQ(K,1,1)*FL(1,3)+VQ(K,1,2)*FL(2,3)+VQ(K,1,3)*FL(3,3)
         M13(K)= VQ(K,1,1)*MM(1,3)+VQ(K,1,2)*MM(2,3)+VQ(K,1,3)*MM(3,3)
C        I=2
         F23(K)= VQ(K,2,1)*FL(1,3)+VQ(K,2,2)*FL(2,3)+VQ(K,2,3)*FL(3,3)
         M23(K)= VQ(K,2,1)*MM(1,3)+VQ(K,2,2)*MM(2,3)+VQ(K,2,3)*MM(3,3)
C        I=3
         F33(K)= VQ(K,3,1)*FL(1,3)+VQ(K,3,2)*FL(2,3)+VQ(K,3,3)*FL(3,3)
         M33(K)= VQ(K,3,1)*MM(1,3)+VQ(K,3,2)*MM(2,3)+VQ(K,3,3)*MM(3,3)
C
C        J=4
C        I=1
         F14(K)= VQ(K,1,1)*FL(1,4)+VQ(K,1,2)*FL(2,4)+VQ(K,1,3)*FL(3,4)
         M14(K)= VQ(K,1,1)*MM(1,4)+VQ(K,1,2)*MM(2,4)+VQ(K,1,3)*MM(3,4)
C        I=2
         F24(K)= VQ(K,2,1)*FL(1,4)+VQ(K,2,2)*FL(2,4)+VQ(K,2,3)*FL(3,4)
         M24(K)= VQ(K,2,1)*MM(1,4)+VQ(K,2,2)*MM(2,4)+VQ(K,2,3)*MM(3,4)
C        I=3
         F34(K)= VQ(K,3,1)*FL(1,4)+VQ(K,3,2)*FL(2,4)+VQ(K,3,3)*FL(3,4)
         M34(K)= VQ(K,3,1)*MM(1,4)+VQ(K,3,2)*MM(2,4)+VQ(K,3,3)*MM(3,4)
C
       ENDIF
C
      ENDDO
C
      END IF !IF(IFINI > 0 )THEN
C
      RETURN
      END
