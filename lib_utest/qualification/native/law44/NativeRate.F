! SPDX-License-Identifier: AGPL-3.0-or-later
! Selected HM_READ_MAT/MULAWC filter coefficient, OpenRadioss (C) 2026 Siemens.
! No default is supplied here: FCUT has already been resolved by the caller.
      FUNCTION LAW44_POINT_FILTER(FCUT,DT) RESULT(ALPHA)
     . BIND(C,NAME='law44_point_filter')
      USE ISO_C_BINDING, ONLY: C_DOUBLE
#include "implicit_f.inc"
      REAL(C_DOUBLE),VALUE :: FCUT,DT
      REAL(C_DOUBLE) :: ALPHA
      MY_REAL ANGULAR_CUTOFF
      ANGULAR_CUTOFF=TWO*PI*FCUT
      ALPHA=MIN(ONE,ANGULAR_CUTOFF*DT)
      END FUNCTION
! Selected CZFORC3 / C3FORC3 EPSD_PG, before material thickness is advanced.
      FUNCTION LAW44_POINT_SHELL_RATE(DEPS,THK,DT) RESULT(RATE)
     . BIND(C,NAME='law44_point_shell_rate')
      USE ISO_C_BINDING, ONLY: C_DOUBLE
#include "implicit_f.inc"
      REAL(C_DOUBLE),INTENT(IN) :: DEPS(8)
      REAL(C_DOUBLE),VALUE :: THK,DT
      REAL(C_DOUBLE) :: RATE
      MY_REAL DTINV,EPS_K2,EPS_M2
      DTINV=DT/MAX(DT**2,EM20)
      EPS_K2=(DEPS(6)**2+DEPS(7)**2+DEPS(6)*DEPS(7)
     .       +FOURTH*DEPS(8)**2)*ONE_OVER_9*THK**2
      EPS_M2=FOUR_OVER_3*(DEPS(1)**2+DEPS(2)**2
     .       +DEPS(1)*DEPS(2)+FOURTH*DEPS(3)**2)
      RATE=SQRT(EPS_K2+EPS_M2)*DTINV
      END FUNCTION
