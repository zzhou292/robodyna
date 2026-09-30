! SPDX-License-Identifier: AGPL-3.0-or-later
      SUBROUTINE LAW42_NATIVE_GLOBALS()
#include "implicit_f.inc"
#include "scr05_c.inc"
#include "impl1_c.inc"
#include "tabsiz_c.inc"
      IRESP=0
      IMPL_S=0
      SNPC=2
      STF=1
      END

! A bulk curve is outside the explicitly selected material profile.
      DOUBLE PRECISION FUNCTION LAW42_REF_FINTER(K,X,NPF,TF,D)
      INTEGER K,NPF(*)
      DOUBLE PRECISION X,TF(*),D
      ERROR STOP 'Unselected LAW42 bulk curve reached'
      END
