! SPDX-License-Identifier: AGPL-3.0-or-later
! C packet adapter; constructs its own native startup and current geometry.
      SUBROUTINE LAW90_CURRENT_NATIVE(X0,RHO,X,V,STEP_DT,VALUES,STATUS)
     . BIND(C,NAME="law90_current_native")
      USE ISO_C_BINDING
      USE LAW90_ENGINE_PACKETS
#include "implicit_f.inc"
      REAL(C_DOUBLE),INTENT(IN) :: X0(3,8),RHO,X(3,8),V(3,8),STEP_DT
      REAL(C_DOUBLE),INTENT(OUT) :: VALUES(991)
      INTEGER(C_INT),INTENT(OUT) :: STATUS
      TYPE(NATIVE_GEOMETRY) :: G
      REAL(C_DOUBLE) INITIAL(755),SAVED(21),PIJ(72,8),XN(3,8),VN(3,8)
      REAL(C_DOUBLE) DISP(MVSIZ,8,3),WORLD(MVSIZ,8,9),MF(MVSIZ,8,9),B(MVSIZ,8,6)
      REAL(C_DOUBLE) RATE(6)
      INTEGER(C_INT) PERM(8),TAGS(4)
      INTEGER N,K,A,IP,J,COLUMN,ROW,IINT,JCVT,ICP,ISMSTR,MTN,ISELECT,ISEL_V
      INTERFACE
       SUBROUTINE LAW90_REFERENCE_NATIVE(X,RHO,VALUES,PERM,TAGS,STATUS)
     . BIND(C,NAME="law90_reference_native")
        USE ISO_C_BINDING
        REAL(C_DOUBLE),INTENT(IN) :: X(3,8),RHO
        REAL(C_DOUBLE),INTENT(OUT) :: VALUES(755)
        INTEGER(C_INT),INTENT(OUT) :: PERM(8),TAGS(4),STATUS
       END SUBROUTINE
      END INTERFACE
      IINT=2
      JCVT=1
      ICP=0
      ISMSTR=10
      MTN=90
#include "selection.inc"
      IF(ISELECT/=1.OR.ISEL_V/=0)THEN
        STATUS=997
        RETURN
      ENDIF
      VALUES=ZERO
      CALL LAW90_REFERENCE_NATIVE(X0,RHO,INITIAL,PERM,TAGS,STATUS)
      IF(STATUS/=0)RETURN
      DO N=1,7
       SAVED(N)=INITIAL(735+3*(N-1))
       SAVED(N+7)=INITIAL(736+3*(N-1))
       SAVED(N+14)=INITIAL(737+3*(N-1))
      ENDDO
      DO IP=1,8
       PIJ(:,IP)=INITIAL(159+72*(IP-1):230+72*(IP-1))
      ENDDO
      DO N=1,8
       XN(:,N)=X(:,PERM(N)+1)
       VN(:,N)=V(:,PERM(N)+1)
      ENDDO
      CALL LAW90_ENGINE_GEOMETRY(XN,VN,SAVED,G,STATUS)
      IF(STATUS/=0)RETURN
      CALL LAW90_ENGINE_TOTAL(X,PERM,SAVED,PIJ,G,DISP,WORLD,MF,B)
      K=0
C     SORTHO3 output order is E1X,E2X,E3X,E1Y,...: already row-major.
      VALUES(1:9)=G%FRAME(1,:)
      K=9
      DO N=1,8
       VALUES(K+1:K+3)=G%X(1,N,:)
       K=K+3
      ENDDO
      DO N=1,8
       VALUES(K+1:K+3)=G%V(1,N,:)
       K=K+3
      ENDDO
      DO A=1,3
       VALUES(K+1:K+4)=G%CENTER_P(1,:,A)
       K=K+4
      ENDDO
      VALUES(K+1)=G%CENTER_VOLUME(1)
      VALUES(K+2)=G%SMAX(1)
      K=K+2
      DO IP=1,8
       VALUES(K+1)=G%VOLUME(1,IP)
       K=K+1
       VALUES(K+1:K+9)=G%INVERSE(1,IP,:)
       K=K+9
       DO A=1,3
        VALUES(K+1:K+8)=G%P(1,IP,:,A)
        K=K+8
       ENDDO
       DO A=1,6
        VALUES(K+1:K+8)=G%SHEAR(1,IP,:,A)
        K=K+8
       ENDDO
      ENDDO
      DO N=1,8
       VALUES(K+1:K+3)=DISP(1,N,:)
       K=K+3
      ENDDO
      DO IP=1,8
       VALUES(K+1:K+9)=WORLD(1,IP,:)
       K=K+9
       VALUES(K+1:K+9)=MF(1,IP,:)
       K=K+9
       VALUES(K+1:K+6)=B(1,IP,:)
       K=K+6
       CALL LAW90_ENGINE_RATE(G,IP,STEP_DT,INITIAL(54+10*IP),RATE,STATUS)
       IF(STATUS/=0)RETURN
       VALUES(K+1:K+6)=RATE
       K=K+6
      ENDDO
      IF(K/=991)STATUS=998
      END
