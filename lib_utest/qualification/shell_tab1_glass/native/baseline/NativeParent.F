! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact selected starter default and positive-threshold FAIL_SETOFF_C blocks.
      SUBROUTINE LAYERED_FAILURE_PARENT(FLAGS,WEIGHTS,PARENT)
     . BIND(C,NAME='layered_failure_parent')
      USE ISO_C_BINDING,ONLY:C_DOUBLE,C_INT
#include "implicit_f.inc"
      INTEGER(C_INT),INTENT(IN)::FLAGS(3)
      REAL(C_DOUBLE),INTENT(IN)::WEIGHTS(3)
      REAL(C_DOUBLE),INTENT(INOUT)::PARENT
      INTEGER,PARAMETER::NEL=1,NPTT=3,IL=1,IFL=1
      INTEGER IPT,IEL,IPOS,FOFF(1)
      MY_REAL PTHK,GEO(43),P_THICKG,PTHKF(1,1),THFACT(1),
     . NPFAIL(1),OFF(1),THKLY(3)
      LOGICAL PRINT_FAIL(1)
      PTHK=ZERO
      GEO=ZERO
#include "PropertyDefault.inc"
      P_THICKG=GEO(42)
      PTHKF=ONE
#include "ParentThreshold.inc"
      OFF=PARENT
      THKLY=WEIGHTS
      THFACT=ZERO
      NPFAIL=ZERO
      PRINT_FAIL=.TRUE.
      DO IPT=1,NPTT
        FOFF=FLAGS(IPT)
#include "FailedThickness.inc"
      ENDDO
#include "ParentCriterion.inc"
              ENDIF
            ENDIF
          ENDDO
      PARENT=OFF(1)
      END SUBROUTINE
