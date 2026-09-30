! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact selected default and parameter packing statements from HM_READ_FAIL_TAB1.
      SUBROUTINE TAB1_NATIVE_DEFAULTS(PARAMETERS,THRESHOLD)
     . BIND(C,NAME='tab1_native_defaults')
      USE ISO_C_BINDING,ONLY:C_DOUBLE
      USE GLASS_TAB1_MESSAGE_MOD
#include "implicit_f.inc"
      REAL(C_DOUBLE),INTENT(OUT)::PARAMETERS(22),THRESHOLD
      TYPE PARAMETER_RECORD
        MY_REAL UPARAM(22)
      END TYPE
      TYPE(PARAMETER_RECORD) FAIL
      MY_REAL DCRIT,DADV,DD,ECRIT,DN,Y1SCALE,Y2SCALE,SCALE_EL,
     . SCALE_TEMP,SHRF,BIAXF,FADE_EXPO,P_THICK,PTHKF,
     . X1SCALE,X2SCALE,EL_REF,FSCAL_UNIT,P_THINNFAIL
      INTEGER MAT_ID,ITAB_INST,IFAIL_SH,ISOLID,IXFEM,DMG_FLAG,INST_FLAG
      DCRIT=ZERO
      DADV=ZERO
      DD=ZERO
      ECRIT=ZERO
      DN=ZERO
      Y1SCALE=ZERO
      Y2SCALE=ZERO
      SCALE_EL=ZERO
      SCALE_TEMP=ZERO
      SHRF=ZERO
      BIAXF=ZERO
      FADE_EXPO=ZERO
      P_THICK=ZERO
      P_THINNFAIL=ZERO
      ITAB_INST=0
      ISOLID=0
      IXFEM=0
      INST_FLAG=0
      MAT_ID=17
! Explicit native any-point policy; app source NUMINT1 override is separate.
      IFAIL_SH=1
#include "Defaults.inc"
! Resolved working-SI unit dimensions; these channels are unused by the
! selected NDIM1/no-size/no-rate-table branch and are never production inputs.
      FSCAL_UNIT=ONE
      X1SCALE=ZERO
      X2SCALE=ZERO
      EL_REF=ZERO
#include "DefaultUnits.inc"
#include "Packing.inc"
      PARAMETERS=FAIL%UPARAM
      THRESHOLD=PTHKF
      END SUBROUTINE
