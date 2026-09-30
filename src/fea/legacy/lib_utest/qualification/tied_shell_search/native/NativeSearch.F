! SPDX-License-Identifier: AGPL-3.0-or-later
! Independent one-candidate wrapper. Coordinates are native working units;
! complete I2DST3/I2BAR3/I7LIN3 own all projection and ordered selection.
      SUBROUTINE TIED_NATIVE_SEARCH(TRI,X,THKMAIN,THKSECND,
     . CANDIDATE,SELECTED,SAVED_ST,SAVED_DISTANCE,RESULT)
     . BIND(C,NAME="tied_native_search")
      USE ISO_C_BINDING, ONLY:C_DOUBLE,C_INT
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      INTEGER(C_INT),VALUE :: TRI,CANDIDATE
      REAL(C_DOUBLE),INTENT(IN) :: X(3,5),THKMAIN,THKSECND
      INTEGER(C_INT),INTENT(INOUT) :: SELECTED
      REAL(C_DOUBLE),INTENT(INOUT) :: SAVED_ST(2),SAVED_DISTANCE
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT(9)
      INTEGER I,IRECT(4,1),CAND_E(1),CAND_N(1),IRTL(1),
     . IX3(MVSIZ),IX4(MVSIZ),NODE1,NODE2,NODE3,NODE4,J,FAN
      MY_REAL GAPV(MVSIZ),DD,DX1,DY1,DZ1,DX3,DY3,DZ3,DMIN(1),ST(2,1),
     . XX(MVSIZ,4),YY(MVSIZ,4),ZZ(MVSIZ,4),XI(MVSIZ),YI(MVSIZ),ZI(MVSIZ),
     . X0(MVSIZ),Y0(MVSIZ),Z0(MVSIZ),NX(MVSIZ,4),NY(MVSIZ,4),NZ(MVSIZ,4),
     . P(MVSIZ,4),LB(MVSIZ,4),LC(MVSIZ,4),S(MVSIZ),T(MVSIZ),PENE
      I=1
      IRECT(:,1)=[1,2,3,4]
      IF(TRI==1) IRECT(4,1)=3
      CAND_E=1
#include "Gap.inc"
      DO J=1,4
        XX(1,J)=X(1,J);YY(1,J)=X(2,J);ZZ(1,J)=X(3,J)
      ENDDO
      XI(1)=X(1,5);YI(1)=X(2,5);ZI(1)=X(3,5)
      IX3=3;IX4=4
      IF(TRI==1) IX4=3
      CAND_E=CANDIDATE;CAND_N=1
      IRTL=SELECTED;ST(:,1)=SAVED_ST;DMIN=SAVED_DISTANCE
      CALL I2DST3(1,1,GAPV,CAND_E,CAND_N,ZERO,IRTL,ST,DMIN,2,
     . IX3,IX4,XX(:,1),XX(:,2),XX(:,3),XX(:,4),
     . YY(:,1),YY(:,2),YY(:,3),YY(:,4),ZZ(:,1),ZZ(:,2),ZZ(:,3),ZZ(:,4),
     . XI,YI,ZI,X0,Y0,Z0,NX(:,1),NY(:,1),NZ(:,1),NX(:,2),NY(:,2),NZ(:,2),
     . NX(:,3),NY(:,3),NZ(:,3),NX(:,4),NY(:,4),NZ(:,4),
     . P(:,1),P(:,2),P(:,3),P(:,4),LB(:,1),LB(:,2),LB(:,3),LB(:,4),
     . LC(:,1),LC(:,2),LC(:,3),LC(:,4),S,T)
      FAN=1
      DO J=2,4
        IF(P(1,J)>P(1,FAN)) FAN=J
      ENDDO
      IF(TRI==1) FAN=1
      PENE=P(1,FAN)
      RESULT(1:6)=[GAPV(1),PENE,GAPV(1)-PENE,S(1),T(1),DBLE(FAN-1)]
      RESULT(7)=ZERO
      IF(PENE>ZERO.AND.S(1)<ONEP5.AND.S(1)>-ONEP5.AND.
     . T(1)<ONEP5.AND.T(1)>-ONEP5) RESULT(7)=ONE
      RESULT(8:9)=ST(:,1)
      SELECTED=IRTL(1);SAVED_ST=ST(:,1);SAVED_DISTANCE=DMIN(1)
      END
