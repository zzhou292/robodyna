! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact FAIL_SETOFF_NPG_C selected scalar storage view. Logging and failure
! wave output are excluded; all OFFPG/parent arithmetic is retained verbatim.
      SUBROUTINE QBF_PARENT(IR,IS,FLAGS,SURFACES,OFF)
#include "implicit_f.inc"
      INTEGER IR,IS,FLAGS(4),IPG,NPG,NEL,NPTT,NFAIL,IL,JPG,
     . IEL,IFL,IPT,IPOS,IG,COUNTPG,FAIL_NUM(1)
      MY_REAL SURFACES(4),OFF(1),THFACT,NPFAIL,THKLY(1),PTHKF(1,1)
      LOGICAL PRINT_FAIL(1)
      TYPE QBF_FLOC
        INTEGER,POINTER :: OFF(:)
      END TYPE
      TYPE QBF_FAIL
        TYPE(QBF_FLOC) :: FLOC(1)
      END TYPE
      TYPE QBF_LAYER
        INTEGER :: NPTT
        MY_REAL,POINTER :: OFFPG(:)
        TYPE(QBF_FAIL) :: FAIL(2,2,1)
      END TYPE
      TYPE QBF_BUFFER
        TYPE(QBF_LAYER) :: BUFLY(1)
      END TYPE
      TYPE(QBF_BUFFER) ELBUF_STR
      MY_REAL,TARGET :: POINTS(4)
      INTEGER,TARGET :: STATUS(1,4)
      MY_REAL,POINTER :: OFFPG(:)
      INTEGER,POINTER :: FOFF(:)
      NEL=1
      NPG=4
      NFAIL=1
      IL=1
      IPG=(IS-1)*2+IR
      JPG=IPG-1
      THKLY=ONE
      PTHKF=ONE
      POINTS=SURFACES
      STATUS(1,:)=FLAGS
      ELBUF_STR%BUFLY(1)%NPTT=1
      ELBUF_STR%BUFLY(1)%OFFPG=>POINTS
      DO IG=1,4
        ELBUF_STR%BUFLY(1)%FAIL(MOD(IG-1,2)+1,(IG-1)/2+1,1)
     .    %FLOC(1)%OFF=>STATUS(:,IG)
      ENDDO
#include "extracted/PointRemoval.inc"
#include "extracted/ParentRemoval.inc"
! Native logging / failure-wave statements 197:202 have no value outputs.
#include "extracted/ParentRemovalEnd.inc"
      SURFACES=POINTS
      END
