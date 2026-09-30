C SPDX-License-Identifier: AGPL-3.0-or-later
C Exact RKINI3 slope loop; caller initializes its resolved curve and stiffness.
      SUBROUTINE TYPE13_SLOPE(POINTS,INITIAL,SCALE,RESULT)
     . BIND(C,NAME='type13_native_slope')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: POINTS(2,5),INITIAL,SCALE
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT
      INTEGER IFUNCT,NPC(2),NPOINT,J,J1
      DOUBLE PRECISION PLD(10),XK,LSCALE,A1,B1,A2,B2
      IFUNCT=1
      NPC=(/1,11/)
      PLD=RESHAPE(POINTS,(/10/))
      XK=INITIAL
      LSCALE=SCALE
#include "extracted/MaximumSlope.inc"
      RESULT=XK
      END

C Starter floor then RMASS Ileng1; no mass/inertia defaults synthesized here.
      SUBROUTINE TYPE13_MASS(MASS,INERTIA,LENGTH,RESULT)
     . BIND(C,NAME='type13_native_mass')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: MASS,INERTIA,LENGTH
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT(3)
      DOUBLE PRECISION,PARAMETER :: HALF=.5D0,EM20=1D-20
      INTEGER I,LFT,LLT,NFT,IXR(3,1)
      DOUBLE PRECISION GEO(9,1),XL(1),EMS(1),XI,XIN,MSR(3,1),INR(3,1)
      LFT=1
      LLT=1
      NFT=0
      IXR=1
      XL=LENGTH
      GEO=0D0
      GEO(1,1)=MASS
      XIN=INERTIA
#include "extracted/InertiaFloor.inc"
      ENDIF
      GEO(9,1)=XIN
#include "extracted/HalfMass.inc"
      I=1
#include "extracted/HalfInertia.inc"
      RESULT=(/MSR(1,1),INR(1,1),XIN/)
      END
