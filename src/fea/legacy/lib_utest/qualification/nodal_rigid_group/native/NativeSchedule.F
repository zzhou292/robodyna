! Exact startup/loop duration fragments; prescribed fixed-step selector is
! authored wrapper context. This does not execute the complete RESOL engine.
! OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later.
      SUBROUTINE RBG_START(DT1,DT2,TT)
     . BIND(C,NAME="nodal_rigid_native_start")
      USE ISO_C_BINDING, ONLY:C_DOUBLE
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(OUT)::DT1,DT2,TT
      REAL(C_DOUBLE),PARAMETER::ZERO=0.D0
#include "extracted/FreshDurations.inc"
      END

      SUBROUTINE RBG_SCHEDULE(DT1,DT12,DT2,SELECTED)
     . BIND(C,NAME="nodal_rigid_native_schedule")
      USE ISO_C_BINDING, ONLY:C_DOUBLE
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(INOUT)::DT1,DT12,DT2
      REAL(C_DOUBLE),INTENT(IN)::SELECTED
      REAL(C_DOUBLE),PARAMETER::EP06=1.D6,HALF=.5D0
      REAL(C_DOUBLE) DT2OLD,DT3
      INTEGER,PARAMETER::INCONV=1
#include "extracted/PreviousDrift.inc"
! Authored prescribed fixed-step selector, in place of native stability scan.
      DT2=SELECTED
#include "extracted/KickDuration.inc"
      END
