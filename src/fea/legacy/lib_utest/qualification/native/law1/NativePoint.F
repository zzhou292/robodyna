! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact SIGEPS01C leaf and authenticated HM_READ_MAT01 coefficient statements.
      SUBROUTINE LAW1_POINT_PHYSICAL(BASIC,GSIN,BASE,INC,LAYER,THICK,VALUES)
     . BIND(C,NAME='law1_point_physical')
      USE ISO_C_BINDING,ONLY:C_DOUBLE
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "law1_param_c.inc"
      REAL(C_DOUBLE),INTENT(IN)::BASIC(3),BASE(5),INC(5)
      REAL(C_DOUBLE),VALUE::GSIN,LAYER,THICK
      REAL(C_DOUBLE),INTENT(OUT)::VALUES(10)
      MY_REAL PM(40),YOUNG,ANU,RHOR,G,C0,C1,E0,E1MN2,EN1N2,SDSP,
     . GS(1),THK(1),THKLYL(1),OFF(1),SOLD(1,5),SNEW(1,5),DEPS(MVSIZ,5),EPS(MVSIZ,3)
      INTEGER ISRATE
      NPROPM=40
      PM=ZERO
      YOUNG=BASIC(1)
      ANU=BASIC(2)
      RHOR=BASIC(3)
#include "Law1Coefficients.inc"
      GS=GSIN
      THK=THICK
      THKLYL=LAYER
      OFF=ONE
      SOLD(1,:)=BASE
      SNEW=ZERO
      DEPS=ZERO
      DEPS(1,:)=INC
      EPS=ZERO
      CALL LAW1_POINT_REF_SIGEPS01C(1,1,1,1,GS,
     . SOLD(:,1),SOLD(:,2),SOLD(:,3),SOLD(:,4),SOLD(:,5),
     . SNEW(:,1),SNEW(:,2),SNEW(:,3),SNEW(:,4),SNEW(:,5),
     . DEPS(:,1),DEPS(:,2),DEPS(:,3),DEPS(:,4),DEPS(:,5),
     . THK,THKLYL,OFF,PM,2,EPS(:,1),EPS(:,2),EPS(:,3))
      VALUES(1:5)=SNEW(1,:)
      VALUES(6)=THK(1)
      VALUES(7:10)=[G,E1MN2,EN1N2,SDSP]
      END SUBROUTINE
