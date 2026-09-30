! SPDX-License-Identifier: AGPL-3.0-or-later
! Actual descriptor exponents and explicit canonical mm/s/Mg-to-SI factors
! enter the exact HM_GET_FLOATV scalar conversion region (full source pinned).
      SUBROUTINE LAW90_SDI_UNITS(DVAL,DIMS,RVAL)
     . BIND(C,NAME='law90_sdi_units')
      USE ISO_C_BINDING, ONLY: C_DOUBLE
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: DVAL,DIMS(3)
      REAL(C_DOUBLE),INTENT(OUT) :: RVAL
      REAL(C_DOUBLE) FAC,FAC_M,FAC_L,FAC_T,MASS_DIM,
     . LENGTH_DIM,TIME_DIM,ZERO
      ZERO=0D0
      FAC=1D0
      FAC_M=1000D0
      FAC_L=.001D0
      FAC_T=1D0
      LENGTH_DIM=DIMS(1)
      MASS_DIM=DIMS(2)
      TIME_DIM=DIMS(3)
#include "units_values.inc"
      END
