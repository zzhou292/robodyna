C SPDX-License-Identifier: AGPL-3.0-or-later
C Exact OpenRadioss arithmetic includes; see the source manifest and README.
      SUBROUTINE MAPPED_TYPE25_STIFFNESS(P,LENGTH,ACTIVE,MS,IN,ROOT,
     . RESULT) BIND(C,NAME='mapped_type25_native_stiffness')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: P(10),LENGTH,MS(3),IN(3)
      INTEGER(C_INT),INTENT(IN) :: ACTIVE,ROOT(2)
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT(4)
      INTEGER I,NEL,JFT,JLT,IRB1,IRB2,MGN(1),NC1(1),NC2(1)
      DOUBLE PRECISION,PARAMETER :: ZERO=0D0,TWO=2D0,FOUR=4D0
      DOUBLE PRECISION GEO(144,1),EPLA(1),XM(1),XK(1),XC(1),YK(1),YC(1)
      DOUBLE PRECISION XKA,YKA,XKM(1),HX(1),HY(1),XH(1),XHR(1)
      DOUBLE PRECISION XCM(1),XKR(1),XCR(1),XIN(1),ALDP(1),OFF(1)
      DOUBLE PRECISION RBY_NODE(2),STI(3,1),STIR(3,1),MASS2,IN2,A
      NEL=1
      JFT=1
      JLT=1
      MGN=1
      NC1=1
      NC2=2
      GEO=0D0
      GEO(1,1)=P(1)
      GEO(9,1)=P(2)
      GEO(3,1)=P(3)
      GEO(10,1)=P(4)
      GEO(19,1)=P(5)
      GEO(23,1)=P(6)
      GEO(4,1)=P(7)
      GEO(11,1)=P(8)
      GEO(20,1)=P(9)
      GEO(24,1)=P(10)
      GEO(41,1)=1D0
      GEO(45,1)=1D0
      GEO(53,1)=1D0
      GEO(57,1)=1D0
      ALDP=LENGTH
      OFF=REAL(ACTIVE,C_DOUBLE)
      RBY_NODE=REAL(ROOT,C_DOUBLE)
#include "Translation.inc"
#include "Rotation.inc"
#include "Nodal.inc"
      RESULT=(/STI(1,1),STI(2,1),STIR(1,1),STIR(2,1)/)
      END

      SUBROUTINE MAPPED_TYPE25_SCATTER(NN,NEL,NODES,VALUES,STIFN,STIFR)
     . BIND(C,NAME='mapped_type25_native_scatter')
      USE ISO_C_BINDING
      IMPLICIT NONE
      INTEGER(C_INT),INTENT(IN) :: NN,NEL,NODES(2,NEL)
      REAL(C_DOUBLE),INTENT(IN) :: VALUES(4,NEL)
      REAL(C_DOUBLE),INTENT(INOUT) :: STIFN(NN),STIFR(NN)
      INTEGER I,NC1(NEL),NC2(NEL)
      DOUBLE PRECISION STI(3,NEL),STIR(3,NEL)
      NC1=NODES(1,:)
      NC2=NODES(2,:)
      STI(1:2,:)=VALUES(1:2,:)
      STIR(1:2,:)=VALUES(3:4,:)
C Original translation and rotation assembly are separate source-order loops.
      DO I=1,NEL
#include "Scatter0.inc"
#include "Scatter1.inc"
      ENDDO
      DO I=1,NEL
#include "Scatter2.inc"
#include "Scatter3.inc"
      ENDDO
      END
