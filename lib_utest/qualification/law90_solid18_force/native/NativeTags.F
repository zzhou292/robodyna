! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact selected zero/default/LAW90/forwarding statements from complete sources.
      SUBROUTINE LAW90_FORCE_TAGS(TAGS) BIND(C,NAME="law90_force_tags")
      USE ISO_C_BINDING
      USE LAW90_FORCE_ELBUFTAG_MOD
      IMPLICIT NONE
      INTEGER(C_INT),INTENT(OUT) :: TAGS(6)
      TYPE(MLAW_TAG_) MTAG
      TYPE GLOBAL_FIELDS
        INTEGER G_EPSD,G_EPSQ,G_PLA,G_WPLA
      END TYPE
      TYPE LOCAL_FIELDS
        INTEGER L_EPSD,L_PLA,L_WPLA
      END TYPE
      TYPE(GLOBAL_FIELDS) GBUF
      TYPE(LOCAL_FIELDS) BUFLY
      INTEGER IAD
      IAD=0
#include "tag_global_default.inc"
#include "tag_local_pla_default.inc"
#include "tag_local_rate_default.inc"
#include "tag_global_zero.inc"
#include "tag_local_rate_zero.inc"
#include "tag_local_zero.inc"
#include "tag_law90.inc"
#include "tag_global_apply.inc"
#include "tag_local_apply.inc"
#include "tag_local_rate_apply.inc"
      TAGS=[GBUF%G_PLA,GBUF%G_WPLA,GBUF%G_EPSD,
     .      BUFLY%L_PLA,BUFLY%L_WPLA,BUFLY%L_EPSD]
      END
