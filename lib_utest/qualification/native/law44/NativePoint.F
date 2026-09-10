! SPDX-License-Identifier: AGPL-3.0-or-later
! Parameter layout adapted from OpenRadioss, Copyright (C) 2026 Siemens.
! C binding around the unchanged SIGEPS44C point update. Explicit experimental
! curve branch with optional filtered total-rate VP2; parameter layout follows
! HM_READ_MAT44. RATE carries explicit C/P, EPSD_PG, ASRATE and accepted UVAR1.
      SUBROUTINE LAW44_POINT_EVALUATE(NPTS,CURVE,E,NU,RHO,GS,
     . BASE,DEPS,RATE,VALUES) BIND(C,NAME='law44_point_evaluate')
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
#include "implicit_f.inc"
      INTEGER(C_INT),VALUE :: NPTS
      REAL(C_DOUBLE),VALUE :: E,NU,RHO,GS
      REAL(C_DOUBLE),INTENT(IN) :: CURVE(2,NPTS+1),BASE(6),DEPS(5),RATE(5)
      REAL(C_DOUBLE),INTENT(OUT) :: VALUES(13)
      INTEGER KFUNC(1),NPF(2),VARTMP(1,1)
      MY_REAL UPARAM(24),ZERO_V(1),RHO_V(1),GS_V(1),THKLY(1),RATE_V(1),
     . OFF(1),LOFF(1),ETSE(1),THK(1),PLA(1),UVAR(1,1),SIGP(1,3),
     . EPSP(1),YLD(1),DPLA(1),SSP(1),VISCMAX(1),SNEW(1,5),
     . SOLD(1,5),INC(1,5),G,A11
      G=E/TWO/(ONE+NU)
      A11=E/(ONE-NU*NU)
      UPARAM=ZERO
      UPARAM(1)=E
      UPARAM(2)=NU
      UPARAM(4)=1.0D30
      UPARAM(5)=1.0D30
      UPARAM(6)=1.0D30
      UPARAM(7)=2.0D30
      UPARAM(9)=ONE
      UPARAM(10)=ONE
      UPARAM(12)=ONE
      UPARAM(16)=G
      UPARAM(17)=TWO*G
      UPARAM(18)=THREE*G
      UPARAM(19)=E/(THREE*(ONE-TWO*NU))
      UPARAM(20)=A11
      UPARAM(21)=A11*NU
      UPARAM(22)=1.0D30
      UPARAM(23)=TWO
      UPARAM(24)=ONE
      IF (RATE(1)>ZERO) THEN
        UPARAM(11)=ONE/RATE(1)
        UPARAM(12)=ONE/RATE(2)
        UPARAM(13)=ONE
      ENDIF
! FISOKIN=0. The default RATE(1)=0 preserves the original rate-off reference.
! NPF reserves a leading curve pair, matching native VINTER pointer arithmetic.
      KFUNC=1
      NPF(1)=0
      NPF(2)=2*(NPTS+1)
      VARTMP=1
      ZERO_V=ZERO
      RATE_V=RATE(3)
      RHO_V=RHO
      GS_V=GS
      THKLY=ONE
      OFF=ONE
      LOFF=ONE
      ETSE=ZERO
      THK=ZERO
      PLA=BASE(6)
      UVAR=RATE(5)
      SIGP=ZERO
      EPSP=ZERO
      YLD=ZERO
      DPLA=ZERO
      SSP=ZERO
      VISCMAX=ZERO
      SNEW=ZERO
      SOLD(1,:)=BASE(1:5)
      INC(1,:)=DEPS
      CALL LAW44_POINT_REF_SIGEPS44C(
     . 1,24,1,1,KFUNC,NPF,CURVE,ZERO,ONE,UPARAM,
     . RHO_V,THKLY,OFF,ETSE,RATE_V,
     . INC(:,1),INC(:,2),INC(:,3),INC(:,4),INC(:,5),
     . INC(:,1),INC(:,2),INC(:,3),INC(:,4),INC(:,5),
     . ZERO_V,ZERO_V,ZERO_V,ZERO_V,ZERO_V,
     . SOLD(:,1),SOLD(:,2),SOLD(:,3),SOLD(:,4),SOLD(:,5),
     . SNEW(:,1),SNEW(:,2),SNEW(:,3),SNEW(:,4),SNEW(:,5),
     . SSP,VISCMAX,THK,PLA,UVAR,GS_V,YLD,EPSP,DPLA,RATE(4),
     . 1,VARTMP,SIGP,0,ZERO_V,LOFF)
      VALUES(1:5)=SNEW(1,:)
      VALUES(6)=PLA(1)
      VALUES(7)=DPLA(1)
      VALUES(8)=ETSE(1)
      VALUES(9)=THK(1)
      VALUES(10)=YLD(1)
      VALUES(11)=SSP(1)
      VALUES(12)=OFF(1)
      VALUES(13)=UVAR(1,1)
      END SUBROUTINE
