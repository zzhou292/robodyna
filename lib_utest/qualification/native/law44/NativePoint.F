! SPDX-License-Identifier: AGPL-3.0-or-later
! Parameter layout adapted from OpenRadioss, Copyright (C) 2026 Siemens.
! C binding around the unchanged SIGEPS44C point update. Explicit experimental
! curve branch with optional filtered total-rate VP2; parameter layout follows
! HM_READ_MAT44. RATE carries explicit C/P, EPSD_PG, ASRATE and accepted UVAR1.
      MODULE LAW44_POINT_BRIDGE_MOD
      CONTAINS
      SUBROUTINE LAW44_POINT_PACKET(MFUNC,A,B,NPTS,CURVE,E,NU,RHO,GS,
     . BASE,DEPS,RATE,LAYER_THICKNESS,INITIAL_THICKNESS,VALUES,PARENT_OFF,NATIVE_TIME,ISRATE_OVERRIDE)
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
#include "implicit_f.inc"
      INTEGER(C_INT),VALUE :: MFUNC,NPTS
      REAL(C_DOUBLE),VALUE :: A,B,E,NU,RHO,GS,LAYER_THICKNESS,INITIAL_THICKNESS
      REAL(C_DOUBLE),INTENT(IN) :: CURVE(2,NPTS+1),BASE(6),DEPS(5),RATE(5)
      REAL(C_DOUBLE),INTENT(OUT) :: VALUES(13)
      REAL(C_DOUBLE),OPTIONAL,INTENT(IN) :: PARENT_OFF,NATIVE_TIME
      INTEGER(C_INT),OPTIONAL,INTENT(IN) :: ISRATE_OVERRIDE
      INTEGER KFUNC(1),NPF(2),VARTMP(1,1)
      MY_REAL UPARAM(24),ZERO_V(1),RHO_V(1),GS_V(1),THKLY(1),RATE_V(1),
     . OFF(1),LOFF(1),ETSE(1),THK(1),PLA(1),UVAR(1,1),SIGP(1,3),
     . EPSP(1),YLD(1),DPLA(1),SSP(1),VISCMAX(1),SNEW(1,5),
     . SOLD(1,5),INC(1,5),G,A11,EVALUATION_TIME
      G=E/TWO/(ONE+NU)
      A11=E/(ONE-NU*NU)
      UPARAM=ZERO
      UPARAM(1)=E
      UPARAM(2)=NU
      UPARAM(3)=A
      UPARAM(4)=1.0D30
      UPARAM(5)=1.0D30
      UPARAM(6)=1.0D30
      UPARAM(7)=2.0D30
      UPARAM(8)=B
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
! Analytic starter defaults, HM_READ_MAT44:191-194,228-232. Keep the existing
! table wrapper's sentinel unchanged; neither bridge admits failure/cap behavior.
      IF (MFUNC==0) THEN
        UPARAM(4)=INFINITY
        UPARAM(5)=INFINITY
        UPARAM(6)=INFINITY
        UPARAM(7)=TWO*INFINITY
        UPARAM(22)=INFINITY
        IF (B/=ZERO) UPARAM(22)=(INFINITY-A)/B
      ENDIF
      UPARAM(23)=TWO
      IF (MFUNC>0) UPARAM(24)=ONE
      IF (RATE(1)>ZERO) THEN
        UPARAM(11)=ONE/RATE(1)
        UPARAM(12)=ONE/RATE(2)
        UPARAM(13)=ONE
      ENDIF
! A separate explicit starter ISRATE permits zero C with filtering. Existing
! callers omit this argument and keep their original packet/default behavior.
      IF(PRESENT(ISRATE_OVERRIDE)) UPARAM(13)=REAL(ISRATE_OVERRIDE,C_DOUBLE)
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
      THKLY=LAYER_THICKNESS
      OFF=ONE
      IF(PRESENT(PARENT_OFF)) OFF=PARENT_OFF
      EVALUATION_TIME=ZERO
      IF(PRESENT(NATIVE_TIME)) EVALUATION_TIME=NATIVE_TIME
      LOFF=ONE
      ETSE=ZERO
      THK=INITIAL_THICKNESS
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
     . 1,24,1,MFUNC,KFUNC,NPF,CURVE,EVALUATION_TIME,ONE,UPARAM,
     . RHO_V,THKLY,OFF,ETSE,RATE_V,
     . INC(:,1),INC(:,2),INC(:,3),INC(:,4),INC(:,5),
     . INC(:,1),INC(:,2),INC(:,3),INC(:,4),INC(:,5),
     . ZERO_V,ZERO_V,ZERO_V,ZERO_V,ZERO_V,
     . SOLD(:,1),SOLD(:,2),SOLD(:,3),SOLD(:,4),SOLD(:,5),
     . SNEW(:,1),SNEW(:,2),SNEW(:,3),SNEW(:,4),SNEW(:,5),
     . SSP,VISCMAX,THK,PLA,UVAR,GS_V,YLD,EPSP,DPLA,RATE(4),
     . MFUNC,VARTMP,SIGP,0,ZERO_V,LOFF)
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

! Preserve the original table ABI and coefficient/operation order. The common
! packet owns initialization only; the unchanged native routine owns mechanics.
      SUBROUTINE LAW44_POINT_PHYSICAL(NPTS,CURVE,E,NU,RHO,GS,
     . BASE,DEPS,RATE,LAYER_THICKNESS,INITIAL_THICKNESS,VALUES)
     . BIND(C,NAME='law44_point_physical')
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
      IMPLICIT NONE
      INTEGER(C_INT),VALUE :: NPTS
      REAL(C_DOUBLE),VALUE :: E,NU,RHO,GS,LAYER_THICKNESS,INITIAL_THICKNESS
      REAL(C_DOUBLE),INTENT(IN) :: CURVE(2,NPTS+1),BASE(6),DEPS(5),RATE(5)
      REAL(C_DOUBLE),INTENT(OUT) :: VALUES(13)
      CALL LAW44_POINT_PACKET(1_C_INT,0.0_C_DOUBLE,0.0_C_DOUBLE,NPTS,CURVE,E,NU,RHO,GS,
     . BASE,DEPS,RATE,LAYER_THICKNESS,INITIAL_THICKNESS,VALUES)
      END SUBROUTINE
      END MODULE

! Preserve the original C ABI and the exact unit-thickness/zero-base operation.
      SUBROUTINE LAW44_POINT_EVALUATE(NPTS,CURVE,E,NU,RHO,GS,
     . BASE,DEPS,RATE,VALUES) BIND(C,NAME='law44_point_evaluate')
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
      USE LAW44_POINT_BRIDGE_MOD, ONLY: LAW44_POINT_PHYSICAL
      IMPLICIT NONE
      INTEGER(C_INT),VALUE :: NPTS
      REAL(C_DOUBLE),VALUE :: E,NU,RHO,GS
      REAL(C_DOUBLE),INTENT(IN) :: CURVE(2,NPTS+1),BASE(6),DEPS(5),RATE(5)
      REAL(C_DOUBLE),INTENT(OUT) :: VALUES(13)
      CALL LAW44_POINT_PHYSICAL(NPTS,CURVE,E,NU,RHO,GS,BASE,DEPS,RATE,
     . 1.0_C_DOUBLE,0.0_C_DOUBLE,VALUES)
      END SUBROUTINE
