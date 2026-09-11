! SPDX-License-Identifier: AGPL-3.0-or-later
! Prepared fields come from the independent LAW90 preparation ABI. These are
! direct native parameter-slot assignments, not a second preparation formula.
      SUBROUTINE LAW90_NATIVE_POINT(P,X,Y,N,STRAIN,RATE,TIME,
     . HISTORY,CURSORS,VALUES) BIND(C,NAME='law90_native_point')
      USE ISO_C_BINDING, ONLY:C_DOUBLE,C_INT
#include "implicit_f.inc"
#include "scr05_c.inc"
#include "impl1_c.inc"
      INTEGER(C_INT),INTENT(IN) :: N
      INTEGER(C_INT),INTENT(INOUT) :: CURSORS(3)
      REAL(C_DOUBLE),INTENT(IN) :: P(33),X(N),Y(N),STRAIN(6),RATE(6),TIME
      REAL(C_DOUBLE),INTENT(INOUT) :: HISTORY(10)
      REAL(C_DOUBLE),INTENT(OUT) :: VALUES(11)
      MY_REAL UPARAM(32),TF(2,N),RHO0(1),D(1,6),EPS(1,6),OLD(1,6),
     . STRESS(1,6),SSP(1),VISCMAX(1),ET(1),EPSD(1),OFF(1),OFFG(1),
     . UVAR(1,10),ASRATE
      INTEGER NPF(2),IFUNC(1),VARTMP(1,3),NGL(1),I
      IRESP=0
      IMPL_S=0
      UPARAM=ZERO
      UPARAM(1)=P(28)
      UPARAM(4)=P(4)
      UPARAM(5)=P(5)
      UPARAM(6)=P(10)
      UPARAM(7)=P(9)
      UPARAM(9)=P(17)
      UPARAM(10)=P(18)
      UPARAM(12)=P(12)
      UPARAM(13)=P(31)
      UPARAM(15)=P(11)
      UPARAM(16)=P(19)
      UPARAM(17)=P(20)
      UPARAM(18)=P(8)
      UPARAM(19)=P(6)
      RHO0(1)=P(2)
      NPF=[0,2*N]
      IFUNC(1)=1
! Synthetic oracle point label, not an original source EID; FAIL0 never prints.
      NGL(1)=1
      ASRATE=ZERO
      OLD=ZERO
      OFF=ONE
      OFFG=ONE
      ET=ONE
      SSP=ZERO
      VISCMAX=ZERO
      EPSD=ZERO
      DO I=1,N
        TF(1,I)=X(I)
        TF(2,I)=Y(I)
      ENDDO
      D(1,:)=RATE
      EPS(1,:)=STRAIN
      UVAR(1,:)=HISTORY
      VARTMP(1,:)=CURSORS
      CALL LAW90_POINT_REF_SIGEPS90(1,10,1,IFUNC,NPF,TF,TIME,
     . UPARAM,RHO0,D(1,1),D(1,2),D(1,3),D(1,4),D(1,5),D(1,6),
     . EPS(1,1),EPS(1,2),EPS(1,3),EPS(1,4),EPS(1,5),EPS(1,6),
     . OLD(1,1),OLD(1,2),OLD(1,3),OLD(1,4),OLD(1,5),OLD(1,6),
     . STRESS(1,1),STRESS(1,2),STRESS(1,3),STRESS(1,4),STRESS(1,5),STRESS(1,6),
     . SSP,VISCMAX,UVAR,3,VARTMP,10,0,ASRATE,OFFG,0,ET,EPSD,OFF,NGL)
      HISTORY=UVAR(1,:)
      CURSORS=VARTMP(1,:)
      VALUES(1:6)=STRESS(1,:)
      VALUES(7:11)=[SSP(1),EPSD(1),ET(1),VISCMAX(1),OFF(1)]
      END
