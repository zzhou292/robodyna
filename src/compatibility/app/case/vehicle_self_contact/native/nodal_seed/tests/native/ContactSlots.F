! SPDX-License-Identifier: AGPL-3.0-or-later
      MODULE LAW42_CONTACT_SLOT_REFERENCE
      USE ISO_C_BINDING, ONLY:C_DOUBLE
      USE LAW42_CONTACT_SLOT_CONSTANTS
      IMPLICIT NONE
      TYPE LOCAL_MAT_PARAM
        REAL(C_DOUBLE) UPARAM(24)
      END TYPE
      CONTAINS
      SUBROUTINE READER(PARAMETERS,PARMAT,PM)
      REAL(C_DOUBLE),INTENT(IN)::PARAMETERS(2)
      REAL(C_DOUBLE),INTENT(OUT)::PARMAT(128)
      REAL(C_DOUBLE),INTENT(INOUT)::PM(200)
      INTEGER I,NORDER
      REAL(C_DOUBLE) MU(10),AL(10),GS,NU,MU0,BULK
      MU=ZERO
      AL=ZERO
      MU(1)=PARAMETERS(1)
      AL(1)=TWO
      NORDER=1
      NU=PARAMETERS(2)
      PARMAT=ZERO
#include "reader_gs.inc"
#include "reader_bulk.inc"
#include "reader_slots.inc"
#include "reader_pm100.inc"
      END SUBROUTINE

      SUBROUTINE SELECTED_UPDATE(PARAMETERS,PM)
      REAL(C_DOUBLE),INTENT(IN)::PARAMETERS(2)
      REAL(C_DOUBLE),INTENT(INOUT)::PM(200)
      TYPE(LOCAL_MAT_PARAM) MAT_PARAM
      INTEGER I,NORDER,MAT_ID,IOUT
      REAL(C_DOUBLE) MU(10),AL(10),GS,NU,BULK,YOUNG,GAMA_INF
      MAT_PARAM%UPARAM=ZERO
      MAT_PARAM%UPARAM(1)=PARAMETERS(1)
      MAT_PARAM%UPARAM(11)=TWO
      MAT_PARAM%UPARAM(21)=PM(100)
      MAT_PARAM%UPARAM(22)=PARAMETERS(2)
      NORDER=1
      MAT_ID=1
      IOUT=6
! Explicit selected caller context: no VISC/PRONY type2/form1 modification.
#include "no_prony_gamma.inc"
#include "selected_update.inc"
#include "selected_update_formats.inc"
      END SUBROUTINE

      SUBROUTINE LAW42_CONTACT_SLOTS_NATIVE(PARAMETERS,SLOTS)
     . BIND(C,NAME='law42_contact_slots_native')
      USE,INTRINSIC::IEEE_ARITHMETIC,ONLY:IEEE_IS_FINITE
      REAL(C_DOUBLE),INTENT(IN)::PARAMETERS(2)
      REAL(C_DOUBLE),INTENT(OUT)::SLOTS(4)
      REAL(C_DOUBLE) PM(200,1),PARMAT(128),BULK,YOUNG,NU,G
      INTEGER I,ILAW,IMAT,IPM(2,1)
      IF(.NOT.ALL(IEEE_IS_FINITE(PARAMETERS)))
     . ERROR STOP 'Nonfinite contact-slot reference input'
      IF(PARAMETERS(1)<=ZERO.OR.PARAMETERS(2)/=0.463_C_DOUBLE)
     . ERROR STOP 'Unselected MAT007 scalar input'
      PM=ZERO
      CALL READER(PARAMETERS,PARMAT,PM(:,1))
      I=1
      ILAW=42
      IPM=0
      IPM(2,1)=ILAW
#include "generic_pm.inc"
#include "reader_high.inc"
#include "reader_keep_bulk.inc"
      CALL SELECTED_UPDATE(PARAMETERS,PM(:,1))
      IMAT=1
#include "updated_high.inc"
      SLOTS=[PM(20,1),PM(32,1),PM(100,1),PM(107,1)]
      END SUBROUTINE
! Independent post-UPDMAT observation for the admitted LAW90 branch. Its
! supplied PM32/PM100 are obtained from the existing full LAW90 preparation
! oracle in the owning test; this routine does not recompute those slots.
      SUBROUTINE LAW90_CONTACT_HIGH_NATIVE(SLOTS,HIGH)
     . BIND(C,NAME='law90_contact_high_native')
      REAL(C_DOUBLE),INTENT(IN)::SLOTS(2)
      REAL(C_DOUBLE),INTENT(OUT)::HIGH
      REAL(C_DOUBLE) PM(200,1)
      INTEGER IMAT,ILAW
      PM=ZERO
      PM(32,1)=SLOTS(1)
      PM(100,1)=SLOTS(2)
      IMAT=1
      ILAW=90
#include "updated_high.inc"
      HIGH=PM(107,1)
      END SUBROUTINE
      END MODULE
