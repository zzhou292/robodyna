! Fixed one-cell history representation and packet mapping only.
! Native field conventions: OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later.
! See original/LICENSE.md and pinned MULAWGLC/CZFINTN1 provenance.
      MODULE QE_Q2_HISTORY_MOD
      USE ISO_C_BINDING, ONLY: C_DOUBLE
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      PRIVATE
      PUBLIC :: QE_HISTORY_,QE_UNPACK_HISTORY,QE_PACK_HISTORY
      TYPE QE_HISTORY_
        MY_REAL FOR(1,5),FOR_G(1,5),MOM(1,3),HOURG(1,12),STRA(1,8),
     .          THK(MVSIZ),EINT(1,2),EVIS(8,1),ACTIVE
      END TYPE
      CONTAINS
      SUBROUTINE QE_UNPACK_HISTORY(INPUT,H)
      REAL(C_DOUBLE),INTENT(IN) :: INPUT(38)
      TYPE(QE_HISTORY_),INTENT(OUT) :: H
      H%FOR(1,:)=INPUT(1:5)
      H%FOR_G(1,:)=INPUT(6:10)
      H%MOM(1,:)=INPUT(11:13)
      H%HOURG(1,:)=INPUT(14:25)
      H%STRA(1,:)=INPUT(26:33)
      H%THK=ZERO
      H%THK(1)=INPUT(34)
      H%EINT(1,:)=INPUT(35:36)
      H%EVIS=ZERO
      H%EVIS(8,1)=INPUT(37)
      H%ACTIVE=INPUT(38)
      END SUBROUTINE

      SUBROUTINE QE_PACK_HISTORY(H,OUTPUT)
      TYPE(QE_HISTORY_),INTENT(IN) :: H
      REAL(C_DOUBLE),INTENT(OUT) :: OUTPUT(38)
      OUTPUT(1:5)=H%FOR(1,:)
      OUTPUT(6:10)=H%FOR_G(1,:)
      OUTPUT(11:13)=H%MOM(1,:)
      OUTPUT(14:25)=H%HOURG(1,:)
      OUTPUT(26:33)=H%STRA(1,:)
      OUTPUT(34)=H%THK(1)
      OUTPUT(35:36)=H%EINT(1,:)
      OUTPUT(37)=H%EVIS(8,1)
      OUTPUT(38)=H%ACTIVE
      END SUBROUTINE
      END MODULE
