! SPDX-License-Identifier: AGPL-3.0-or-later
! Adapter to exact CNDLENI extracts, OpenRadioss (C) 2026 Siemens.
      SUBROUTINE QBAT_COEFFICIENTS(LOCAL,INPUT_AREA,MATERIAL,
     . VISCOSITY,OUTPUT) BIND(C,NAME="qbat_native_coefficients")
      USE ISO_C_BINDING, ONLY: C_DOUBLE
#include "implicit_f.inc"
      REAL(C_DOUBLE), INTENT(IN) :: LOCAL(3,4),INPUT_AREA,
     . MATERIAL(5),VISCOSITY(2)
      REAL(C_DOUBLE), INTENT(OUT) :: OUTPUT(6)
      TYPE VISCOSITY_CONTEXT
        MY_REAL VISC_DM
      END TYPE
      TYPE(VISCOSITY_CONTEXT) GROUP_PARAM
      MY_REAL PM(24,1),GEO(13,1),SSP(1),X2L(1),X3L(1),X4L(1),
     . Y2L(1),Y3L(1),Y4L(1),AREA(1),AL1(1),AL2(1),ALMIN(1),
     . DTEL(1),ALDT(1),THK(1),STIFN(4),STIFR(4),STRC(1),
     . RHO,YOUNG,NU,FAC11,LXYZ0(3),COREL(3,4),
     . X13,X24,Y13,Y24,L13,L24,C1,C2,RX,RY,SX,SY,
     . S1,FAC1,FAC2,FACI,VISCE,VISCDEF,VISCMX,A11,VV,STI,STIR
      INTEGER LFT,LLT,I,IMAT,IPROP,IHBE,MTN,IXC(5,1)
      LFT=1
      LLT=1
      IMAT=1
      IPROP=1
      IHBE=11
      MTN=44
      IXC(:,1)=[1,1,2,3,4]
      PM=ZERO
      GEO=ZERO
      COREL=ZERO
      STIFN=ZERO
      STIFR=ZERO
      STRC=ZERO
      PM(1,1)=MATERIAL(1)
      PM(20,1)=MATERIAL(2)
      PM(21,1)=MATERIAL(3)
      PM(24,1)=MATERIAL(5)
      THK(1)=MATERIAL(4)
      GROUP_PARAM%VISC_DM=VISCOSITY(1)
      GEO(13,1)=VISCOSITY(2)
      AREA(1)=INPUT_AREA
      X2L(1)=LOCAL(1,2)
      X3L(1)=LOCAL(1,3)
      X4L(1)=LOCAL(1,4)
      Y2L(1)=LOCAL(2,2)
      Y3L(1)=LOCAL(2,3)
      Y4L(1)=LOCAL(2,4)
#include "extracted/SoundSpeed.inc"
#include "extracted/CharacteristicLength.inc"
#include "extracted/SelectedLength.inc"
#include "extracted/ViscosityDefaults.inc"
#include "extracted/NodalStiffness.inc"
      OUTPUT=[ALDT(1),SSP(1),VISCMX,DTEL(1),STIFN(1),STIFR(1)]
      END
