! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact S8E_SIGP caller body makes its native EPSIP observable to the test.
      SUBROUTINE SOLID18_FORCE_SELECT(BASE,GLOBAL,UPARAM,
     . FACTOR,POISSON,SELECTED)
      USE SOLID18_FORCE_ELBUFDEF_MOD
#include "implicit_f.inc"
      REAL(KIND=8),INTENT(IN) :: BASE(20,8),GLOBAL(11),UPARAM(31)
      REAL(KIND=8),INTENT(OUT) :: FACTOR,POISSON
      INTEGER,INTENT(OUT) :: SELECTED
      INTEGER,PARAMETER :: NEL=1,LFT=1,LLT=1
      TYPE(ELBUF_STRUCT_),TARGET :: ELBUF_TAB
      TYPE(G_BUFEL_),POINTER :: GBUF
      TYPE(L_BUFEL_),POINTER :: LBUF
      INTEGER I,IR,IS,IT,IP,ILAY,NPTR,NPTS,NPTT,EPSIP(1)
      REAL(KIND=8) :: EPMIN(1),SIGP(1,6),E0(1),FAC(1),NU(1),C1,NU0
      ELBUF_TAB%GBUF%PLA=GLOBAL(7)
      DO IR=1,2
       DO IS=1,2
        DO IT=1,2
         IP=IR+((IS-1)+(IT-1)*2)*2
         ELBUF_TAB%BUFLY(1)%LBUF(IR,IS,IT)%PLA=BASE(13,IP)
         ELBUF_TAB%BUFLY(1)%LBUF(IR,IS,IT)%SIG(1,:)=BASE(1:6,IP)
        END DO
       END DO
      END DO
      C1=UPARAM(14)
      NU0=UPARAM(6)
#include "selection_modulus.inc"
      SIGP=ZERO
#include "point_selection.inc"
      CALL SOLID18_FORCE_S8ZSIGP3(1,1,SIGP,E0,GBUF%PLA,FAC,1,1)
#include "selection_poisson.inc"
      FACTOR=FAC(1)
      POISSON=NU(1)
      SELECTED=EPSIP(1)-1
      END
