! Explicit one-element CNDT3 context; no timestep admission or mass scaling.
! OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later; complete native leaves retained.
      MODULE QE_Q2_STIFFNESS_MOD
      USE ISO_C_BINDING, ONLY: C_DOUBLE
      USE QE_Q1_GEOMETRY_MOD, ONLY: QE_GEOMETRY_
      USE QE_Q2_MATERIAL_MOD, ONLY: QE_MATERIAL_
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      PRIVATE
      PUBLIC :: QE_NATIVE_STIFFNESS
      CONTAINS
      SUBROUTINE QE_NATIVE_STIFFNESS(G,M,NODAL_MASS,VISCMX,STI,STIR,DTEL)
#include "com04_c.inc"
#include "scr02_c.inc"
#include "scr18_c.inc"
#include "sms_c.inc"
      TYPE(QE_GEOMETRY_),INTENT(INOUT) :: G
      TYPE(QE_MATERIAL_),INTENT(IN) :: M
      REAL(C_DOUBLE),INTENT(IN) :: NODAL_MASS
      MY_REAL,INTENT(INOUT) :: VISCMX(MVSIZ)
      MY_REAL,INTENT(OUT) :: STI(MVSIZ),STIR(MVSIZ),DTEL(MVSIZ)
      MY_REAL ALPE(MVSIZ),MSC(MVSIZ),DMELC(MVSIZ),PTG(3,MVSIZ),
     . SSP_EQ(MVSIZ),DT2T
      INTEGER NGL(MVSIZ),NELTST,ITYPTST
      NUMMAT=1
      NODADT=1
      IDTMINS=0
      IDTMIN=0
      DTFAC1=ONE
      ALPE=ONE
      MSC=NODAL_MASS
      DMELC=ZERO
      PTG=ONE
      SSP_EQ=ZERO
      DT2T=HUGE(ONE)
      NGL=1
      NELTST=0
      ITYPTST=0
      STI=ZERO
      STIR=ZERO
      DTEL=ZERO
! CNDT3 deliberately mutates LL with the native damping correction. Q1's
! pre-material geometry packet is captured before this call by the bridge.
      CALL CNDT3(1,1,G%OFF,DT2T,M%AMU,NELTST,ITYPTST,STI,STIR,G%OFFG,
     . M%SSP,VISCMX,M%RHO,M%VOL0,M%THK0,M%THK02,M%A11,G%LL,ALPE,NGL,
     . -1,0,4,G%AREA,M%G,M%SHF,MSC,DMELC,0,PTG,1,0,M%A11R,
     . 1,DTEL,1,M%PM,1,1,M%ZOFFSET,SSP_EQ)
      END SUBROUTINE
      END MODULE
