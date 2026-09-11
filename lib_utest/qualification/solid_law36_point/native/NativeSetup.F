! SPDX-License-Identifier: AGPL-3.0-or-later
! Selected scalar card setup surrounding exact HM_READ_MAT36 expressions.
      SUBROUTINE LAW36_NATIVE_SETUP(E,NU,RHO0,DEFAULTS,UPARAM)
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
#include "implicit_f.inc"
      REAL(C_DOUBLE),INTENT(IN) :: E,NU,RHO0
      INTEGER(C_INT),INTENT(IN) :: DEFAULTS
      REAL(C_DOUBLE),INTENT(OUT) :: UPARAM(31)
      INTEGER J,NFUNC,MFUNC,IFUNC(3),IPFUN,IFUNCE,IFAIL,VP,
     . YLDCHECK,ISMOOTH,OPTE
      MY_REAL G,C1,SOUNDSP,EPSMAX,EPSR1,EPSR2,EPSF,
     . FISOKIN,PSCALE,EINF,CE,RATE(1),YFAC(1)
      TYPE MATERIAL_TAG
        INTEGER G_DMG,L_DMG
      END TYPE
      TYPE(MATERIAL_TAG) MTAG
      MTAG%G_DMG=0
      MTAG%L_DMG=0
      UPARAM=ZERO
      RATE=ZERO
      YFAC=ONE
      IFUNC=0
      IFUNC(1)=1
      NFUNC=1
      MFUNC=2
      IPFUN=0
      IFUNCE=0
      VP=0
      ISMOOTH=0
      FISOKIN=ZERO
      PSCALE=ZERO
      EINF=ZERO
      CE=ZERO
      EPSMAX=ZERO
      EPSR1=ZERO
      EPSR2=ZERO
      EPSF=ZERO
      IF(DEFAULTS/=0) THEN
        EPSMAX=1D30
        EPSR1=1D30
        EPSR2=2D30
        EPSF=3D30
      ENDIF
#include "sentinels.inc"
#include "setup.inc"
      END

      SUBROUTINE LAW36_NATIVE_GLOBALS()
#include "implicit_f.inc"
#include "param_c.inc"
#include "com01_c.inc"
#include "impl1_c.inc"
      NPROPMI=13
      NUMMAT=1
      IMPL_S=0
      END

! The complete implicit donor links, but that branch is outside the public API.
! A native implicit convergence stop must never be mistaken for a valid value.
      SUBROUTINE LAW36_REF_IMP_STOP(CODE)
      INTEGER CODE
      ERROR STOP 'LAW36 implicit branch reached outside qualification'
      END
