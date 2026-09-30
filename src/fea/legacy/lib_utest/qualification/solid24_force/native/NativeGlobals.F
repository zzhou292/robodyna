! SPDX-License-Identifier: AGPL-3.0-or-later
      SUBROUTINE HEPH_NATIVE_GLOBALS(STEP)
#include "implicit_f.inc"
#include "param_c.inc"
#include "com01_c.inc"
#include "com04_c.inc"
#include "com08_c.inc"
#include "impl1_c.inc"
#include "scr05_c.inc"
#include "scr07_c.inc"
#include "scr14_c.inc"
#include "scr17_c.inc"
#include "scr18_c.inc"
      REAL(KIND=8),INTENT(IN) :: STEP
      DT1=STEP
      NPROPM=110
      NPROPG=172
      NPSAV=20
      NUMNOD=8
! Selected modern property layout: SZHOUR3 reads CVIS from GEO(13),
! beginning at native INVSTR35, rather than the legacy material PM(4).
      INVSTR=35
      INTEG8=0
      IMPL_S=0
      IDYNA=0
      ISMDISP=0
      ISCAU=0
      IMP_LR=0
      IDTS6=0
      IRESP=0
      MSTOP=0
      INEG_V=0
      INCONV=1
      IMCONV=1
! Family leaves receive explicit 1..NEL bounds from NativeGeometry/Reduce.
! LFT/LLT belong to the retained full driver context, not these common blocks.
      ANIM_N=0
      IAD_GPS=0
      END
