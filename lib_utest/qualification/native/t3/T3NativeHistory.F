! SPDX-License-Identifier: AGPL-3.0-or-later
! OpenRadioss (C) 2026 Siemens, selected C3FORC3 history field packet.
! Three-node centered LAW1: no QEPH HOURG or EVIS storage is fabricated.
      MODULE T3_ENGINE_HISTORY_MOD
      USE ISO_C_BINDING, ONLY: C_DOUBLE
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      PRIVATE
      PUBLIC :: T3_HISTORY_,T3_UNPACK_HISTORY,T3_PACK_HISTORY
      TYPE T3_HISTORY_
        MY_REAL FOR(1,5),FOR_G(1,5),MOM(1,3),STRA(1,8),
     . THK(MVSIZ),EINT(1,2),EPSD,ACTIVE
      END TYPE
      CONTAINS
      SUBROUTINE T3_UNPACK_HISTORY(INPUT,H)
      REAL(C_DOUBLE),INTENT(IN) :: INPUT(26)
      TYPE(T3_HISTORY_),INTENT(OUT) :: H
      H%FOR(1,:)=INPUT(1:5)
      H%FOR_G(1,:)=INPUT(6:10)
      H%MOM(1,:)=INPUT(11:13)
      H%STRA(1,:)=INPUT(14:21)
      H%THK=ZERO
      H%THK(1)=INPUT(22)
      H%EINT(1,:)=INPUT(23:24)
      H%EPSD=INPUT(25)
      H%ACTIVE=INPUT(26)
      END SUBROUTINE
      SUBROUTINE T3_PACK_HISTORY(H,OUTPUT)
      TYPE(T3_HISTORY_),INTENT(IN) :: H
      REAL(C_DOUBLE),INTENT(OUT) :: OUTPUT(26)
      OUTPUT(1:5)=H%FOR(1,:)
      OUTPUT(6:10)=H%FOR_G(1,:)
      OUTPUT(11:13)=H%MOM(1,:)
      OUTPUT(14:21)=H%STRA(1,:)
      OUTPUT(22)=H%THK(1)
      OUTPUT(23:24)=H%EINT(1,:)
      OUTPUT(25)=H%EPSD
      OUTPUT(26)=H%ACTIVE
      END SUBROUTINE
      END MODULE
