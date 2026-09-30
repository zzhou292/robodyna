! SPDX-License-Identifier: AGPL-3.0-or-later
! SRHO3 has scalar RHO0; MMAIN receives an NEL array. Keep both actual ABIs.
      SUBROUTINE LAW90_FORCE_DENSITY(RHO0,RHON,VOLN,VOLO,OFFG,DVOL,EINT)
#include "implicit_f.inc"
      INTEGER,PARAMETER :: NEL=1
      INTEGER I
      REAL(KIND=8) RHO0,RHON(1),VOLN(1),VOLO(1),OFFG(1),DVOL(1),EINT(1)
#include "density_update.inc"
      END
