! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact selected MQVISCB statements, with caller-owned resolved profile.
      SUBROUTINE LAW90_CALLER_VISCOSITY(RATES,RHO_VALUE,RHO_INITIAL,
     . VOLUME,LENGTH,SOUND,MAT_VIS,QOUT,DTOUT,STIOUT)
#include "implicit_f.inc"
      REAL(KIND=8),INTENT(IN) :: RATES(6),RHO_VALUE,RHO_INITIAL
      REAL(KIND=8),INTENT(IN) :: VOLUME,LENGTH,SOUND,MAT_VIS
      REAL(KIND=8),INTENT(OUT) :: QOUT,DTOUT,STIOUT
      INTEGER,PARAMETER :: NEL=1,NPG=8
      INTEGER I,MT,MAT(1),PID(1),IPM(251,1),N2D,IMPL_S,IDYNA,ISMDISP
      REAL(KIND=8) :: D1(1),D2(1),D3(1),DD(1),AD(1),AL(1),CX(1)
      REAL(KIND=8) :: SSP(1),VD2(1),VISI,FACQ,FACQ0,OFF(1),OFFG(1)
      REAL(KIND=8) :: FACPG,VOL(1),PM(88,1),GEO(172,1),RHO0(1),NRHO(1)
      REAL(KIND=8) :: RHOREF(1),RHO(1),VIS(1),DELTAX(1),PSH(1),DTMIN(1)
      REAL(KIND=8) :: QA,QB,CNS1_0,CNS2_0,CNS1,CNS2,QAA_0,QAA
      REAL(KIND=8) :: QX(1),QVIS(1),SSP_EQ(1),DTX(1),STI(1),AIRE(1)
      REAL(KIND=8) :: TIDT,TRHO,TVOL,TAIRE
#include "viscosity_precision.inc"
      D1=RATES(1)
      D2=RATES(2)
      D3=RATES(3)
      VOL=VOLUME
      DELTAX=LENGTH
      SSP=SOUND
      VIS=MAT_VIS
      VD2=ZERO
      RHO=RHO_VALUE
      RHOREF=RHO_VALUE
      PM=ZERO
      PM(1,1)=RHO_INITIAL
      GEO=ZERO
      GEO(14,1)=ONEP1
      GEO(15,1)=FIVEEM2
      FACQ0=ONE
      OFF=ONE
      OFFG=ONE
      MAT=1
      PID=1
      IPM=0
      N2D=0
      IMPL_S=0
      IDYNA=0
      ISMDISP=0
      AIRE=ZERO
#include "viscosity_rates.inc"
#include "viscosity_length.inc"
#include "viscosity_pressure.inc"
#include "viscosity_dt.inc"
#include "viscosity_stiffness.inc"
      QOUT=QVIS(1)
      DTOUT=DTX(1)
      STIOUT=STI(1)
      END
