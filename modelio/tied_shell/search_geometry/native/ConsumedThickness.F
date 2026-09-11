C SPDX-License-Identifier: AGPL-3.0-or-later
C Exact shell-only branches. The wrapper closes each IF immediately before
C the excluded solid-volume ELSEIF. No solid candidate is supplied here.
      SUBROUTINE NATIVE_CONSUMED_THICKNESS(IXC,IXTG,IPARTC,IPARTTG,
     .    GEO,THK,THK_PART,NELC,NELTG,CONSUMED)
      USE ELEMENT_MOD, ONLY: NIXC,NIXTG
      USE SEARCH_GEOMETRY_CONTEXT, ONLY: NPROPG,NUMELC
      USE CONSTANT_MOD, ONLY: ZERO
      IMPLICIT NONE
      INTEGER IXC(NIXC,*),IXTG(NIXTG,*),IPARTC(*),IPARTTG(*)
      INTEGER NELC,NELTG,MG,IP
      DOUBLE PRECISION GEO(NPROPG,*),THK(*),THK_PART(*),CONSUMED(2)
      DOUBLE PRECISION LOCAL_THKMAIN,THKMAIN
#include "BoundMasterThickness.inc"
          ENDIF
      CONSUMED(1)=LOCAL_THKMAIN
      THKMAIN=ZERO
#include "ProjectionMasterThickness.inc"
          ENDIF
      CONSUMED(2)=THKMAIN
      END
