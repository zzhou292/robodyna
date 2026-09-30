! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact selected starter default and positive-threshold FAIL_SETOFF_C blocks.
      MODULE ONE_POINT_PARENT_MOD
      CONTAINS
      SUBROUTINE ONE_POINT_PARENT(FLAGS,WEIGHTS,PARENT,THRESHOLD)
      USE ISO_C_BINDING,ONLY:C_DOUBLE,C_INT
#include "implicit_f.inc"
      INTEGER(C_INT),INTENT(IN)::FLAGS(1)
      REAL(C_DOUBLE),INTENT(IN)::WEIGHTS(1)
      REAL(C_DOUBLE),INTENT(INOUT)::PARENT
      REAL(C_DOUBLE),OPTIONAL,INTENT(IN)::THRESHOLD
      INTEGER,PARAMETER::NEL=1,NPTT=1,IL=1,IFL=1
      INTEGER IPT,IEL,IPOS,FOFF(1)
      MY_REAL PTHK,GEO(43),P_THICKG,PTHKF(1,1),THFACT(1),
     . NPFAIL(1),OFF(1),THKLY(1)
      LOGICAL PRINT_FAIL(1)
      PTHK=ZERO
      GEO=ZERO
#include "extracted/PropertyDefault.inc"
      P_THICKG=GEO(42)
      PTHKF=ONE
      IF(PRESENT(THRESHOLD)) PTHKF=THRESHOLD
#include "extracted/ParentThreshold.inc"
      OFF=PARENT
      THKLY=WEIGHTS
      THFACT=ZERO
      NPFAIL=ZERO
      PRINT_FAIL=.TRUE.
      DO IPT=1,NPTT
        FOFF=FLAGS(IPT)
#include "extracted/FailedThickness.inc"
      ENDDO
#include "extracted/ParentCriterion.inc"
              ENDIF
            ENDIF
          ENDDO
      PARENT=OFF(1)
      END SUBROUTINE
      END MODULE
