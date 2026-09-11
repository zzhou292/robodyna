! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete native stress rotation/local force/world projection calls.
! IDRIL0 excludes the separate C3FINTRZ call; no drilling is fabricated.
      SUBROUTINE ONE_POINT_PROJECT(G,M,H,FORCE,COUPLE)
      USE T3_ENGINE_GEOMETRY_MOD,ONLY:T3_GEOMETRY_
      USE T3_ENGINE_HISTORY_MOD,ONLY:T3_HISTORY_
      USE T3_ENGINE_MATERIAL_MOD,ONLY:T3_MATERIAL_
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "com01_c.inc"
#include "com08_c.inc"
      TYPE(T3_GEOMETRY_),INTENT(IN)::G
      TYPE(T3_MATERIAL_),INTENT(IN)::M
      TYPE(T3_HISTORY_),INTENT(IN)::H
      MY_REAL FORCE(MVSIZ,3,3),COUPLE(MVSIZ,3,3),NFOR(1,5),NMOM(1,3),FZERO(3,4,1),UNUSED(3,3)
      NFOR=ZERO
      NMOM=ZERO
      CALL C3SROTO3(1,1,G%ECOS,G%ESIN,H%FOR,H%MOM,NFOR,NMOM,1,1)
      FORCE=ZERO
      COUPLE=ZERO
      CALL C3FINT3(1,1,NFOR,NMOM,M%THK0,G%D(1,1),G%D(1,2),G%D(1,3),
     . FORCE(1,1,1),FORCE(1,1,2),FORCE(1,1,3),
     . FORCE(1,2,1),FORCE(1,2,2),FORCE(1,2,3),
     . FORCE(1,3,1),FORCE(1,3,2),FORCE(1,3,3),
     . COUPLE(1,1,1),COUPLE(1,1,2),COUPLE(1,1,3),
     . COUPLE(1,2,1),COUPLE(1,2,2),COUPLE(1,2,3),1)
      ISIGI=0
      NCYCLE=1
      IRUN=1
      TT=ZERO
      FZERO=ZERO
      UNUSED=ZERO
      CALL C3FCUM3(1,1,UNUSED,
     . FORCE(1,1,1),FORCE(1,1,2),FORCE(1,1,3),
     . FORCE(1,2,1),FORCE(1,2,2),FORCE(1,2,3),
     . FORCE(1,3,1),FORCE(1,3,2),FORCE(1,3,3),FZERO,
     . G%F(1,1,1),G%F(1,2,1),G%F(1,3,1),
     . G%F(1,1,2),G%F(1,2,2),G%F(1,3,2),
     . G%F(1,1,3),G%F(1,2,3),G%F(1,3,3))
      CALL C3MCUM3(1,1,UNUSED,
     . COUPLE(1,1,1),COUPLE(1,1,2),COUPLE(1,1,3),
     . COUPLE(1,2,1),COUPLE(1,2,2),COUPLE(1,2,3),
     . COUPLE(1,3,1),COUPLE(1,3,2),COUPLE(1,3,3),
     . G%F(1,1,1),G%F(1,2,1),G%F(1,3,1),
     . G%F(1,1,2),G%F(1,2,2),G%F(1,3,2),
     . G%F(1,1,3),G%F(1,2,3),G%F(1,3,3))
      END
