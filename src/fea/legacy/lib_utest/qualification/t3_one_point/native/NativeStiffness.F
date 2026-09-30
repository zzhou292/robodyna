! SPDX-License-Identifier: AGPL-3.0-or-later
! OpenRadioss (C) 2026 Siemens; complete C3DT3 under fixed native context.
! NODADT1/DTFAC1(7)=1: diagnostics only, no timestep or mass admission.
      MODULE ONE_POINT_STIFFNESS_MOD
      USE T3_ENGINE_GEOMETRY_MOD, ONLY: T3_GEOMETRY_
      USE T3_ENGINE_MATERIAL_MOD, ONLY: T3_MATERIAL_
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      PRIVATE
      PUBLIC :: ONE_POINT_STIFFNESS
      CONTAINS
      SUBROUTINE ONE_POINT_STIFFNESS(G,M,VISCMX,DTEL)
#include "scr02_c.inc"
#include "scr18_c.inc"
#include "sms_c.inc"
      TYPE(T3_GEOMETRY_),INTENT(INOUT) :: G
      TYPE(T3_MATERIAL_),INTENT(INOUT) :: M
      MY_REAL,INTENT(INOUT) :: VISCMX(MVSIZ)
      MY_REAL,INTENT(OUT) :: DTEL(MVSIZ)
      MY_REAL MSTG(MVSIZ),DMELTG(MVSIZ),PTG(3,MVSIZ),SSP_EQ(MVSIZ),DT2T
      INTEGER NGL(MVSIZ),NELTST,ITYPTST
      NODADT=1
      IDTMINS=0
      IDTMIN=0
      DTFAC1=ONE
      MSTG=ZERO
      DMELTG=ZERO
      PTG=ONE
      SSP_EQ=ZERO
      DT2T=HUGE(ONE)
      NGL=1
      NELTST=0
      ITYPTST=0
      DTEL=ZERO
! The actual native call mutates LENGTH with the damping/ALPE correction.
! Public R2 observables must already have been packed by the caller.
      CALL C3DT3(1,1,M%PM,G%OFF,DT2T,NELTST,ITYPTST,M%STI,M%STIR,
     . G%OFFG,M%SSP,VISCMX,2,0,0,G%ALPE,MSTG,DMELTG,0,PTG,
     . M%SHF,44,0,M%G,M%A11,M%A11R,1,DTEL,G%LENGTH,M%THK0,
     . G%AREA,NGL,1,1,1,M%ZOFFSET,SSP_EQ)
      END SUBROUTINE
      END MODULE
