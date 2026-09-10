! SPDX-License-Identifier: AGPL-3.0-or-later
! Report the exact NIP3 zero-offset native table, including default-real
! literal rounding. No substitute quadrature or arithmetic is supplied here.
      SUBROUTINE LAW44_POINT_SECTION(POS,FORCE,MOMENT)
     . BIND(C,NAME='law44_point_section')
      USE ISO_C_BINDING, ONLY: C_DOUBLE
#include "implicit_f.inc"
#include "com20_c.inc"
      REAL(C_DOUBLE),INTENT(OUT) :: POS(3),FORCE(3),MOMENT(3)
      MY_REAL LOCAL_WM(11,11)
      CALL LAW44_POINT_REF_COQINI
      CALL LAW44_POINT_REF_COQINI_WM(LOCAL_WM)
      POS=Z0(1:3,3)
      FORCE=WF(1:3,3)
      MOMENT=LOCAL_WM(1:3,3)
      END SUBROUTINE
