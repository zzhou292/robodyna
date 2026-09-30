! SPDX-License-Identifier: AGPL-3.0-or-later
      SUBROUTINE SOLID6Z_FORCE_GLOBALS()
      USE SOLID6Z_FORCE_MESSAGE_MOD, ONLY: GEOMETRY_ERRORS
#include "implicit_f.inc"
#include "scr05_c.inc"
#include "scr07_c.inc"
#include "scr17_c.inc"
#include "scr18_c.inc"
#include "impl1_c.inc"
      IRESP=0
      IDTS6=0
      IDTMIN=0
      IMPL_S=0
      INCONV=0
      IMCONV=0
      INEG_V=0
      MSTOP=0
      GEOMETRY_ERRORS=0
      END
