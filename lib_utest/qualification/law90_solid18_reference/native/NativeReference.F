! SPDX-License-Identifier: AGPL-3.0-or-later
! Selected INITIA SGSAVINI -> S8ZINIT3 LLPIJ72, with complete native leaves.
      SUBROUTINE LAW90_REFERENCE_NATIVE(X,RHO0,VALUES,PERM,TAGS,STATUS)
     . BIND(C,NAME="law90_reference_native")
      USE ISO_C_BINDING
      USE LAW90_START_ELEMENT_MOD,ONLY:NIXS
      USE LAW90_START_MESSAGE_MOD,ONLY:GEOMETRY_ERRORS
      USE LAW90_START_ELBUFDEF_MOD
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "param_c.inc"
#include "com01_c.inc"
#include "vect01_c.inc"
#include "scr05_c.inc"
      REAL(C_DOUBLE),INTENT(IN) :: X(3,8),RHO0
      REAL(C_DOUBLE),INTENT(OUT) :: VALUES(755)
      INTEGER(C_INT),INTENT(OUT) :: PERM(8),TAGS(4),STATUS
      TYPE(ELBUF_STRUCT_),TARGET :: ELBUF_TAB(1)
      TYPE(G_BUFEL_),POINTER :: GBUF
      TYPE(LAYER_),POINTER :: BUFLY
      TYPE(L_BUFEL_),POINTER :: LBUF
      INTEGER,PARAMETER :: NEL=1
      INTEGER I,N,K,IR,IS,IT,IP,ISNOD,JHBE,IINT,ICPRE,MLW,NG,IGTYP
      INTEGER IXS(NIXS,1),MXT(1),NGEO(1),NGL(1),CHECKPERM(8)
      REAL(C_DOUBLE) CJ(MVSIZ,9),H(4,MVSIZ,3),CP(MVSIZ,4,3),NU(MVSIZ)
      REAL(C_DOUBLE) PM(21,1),ZR,ZS,ZT,WI,WT,W_GAUSS(9,9),A_GAUSS(9,9)
      REAL(C_DOUBLE) LAW90_START_CHECKVOLUME_8N,XL,YL,ZL
      REAL(C_DOUBLE) X1(MVSIZ),X2(MVSIZ),X3(MVSIZ),X4(MVSIZ),X5(MVSIZ),X6(MVSIZ),X7(MVSIZ),X8(MVSIZ)
      REAL(C_DOUBLE) Y1(MVSIZ),Y2(MVSIZ),Y3(MVSIZ),Y4(MVSIZ),Y5(MVSIZ),Y6(MVSIZ),Y7(MVSIZ),Y8(MVSIZ)
      REAL(C_DOUBLE) Z1(MVSIZ),Z2(MVSIZ),Z3(MVSIZ),Z4(MVSIZ),Z5(MVSIZ),Z6(MVSIZ),Z7(MVSIZ),Z8(MVSIZ)
      REAL(C_DOUBLE) XD1(MVSIZ),XD2(MVSIZ),XD3(MVSIZ),XD4(MVSIZ),XD5(MVSIZ),XD6(MVSIZ),XD7(MVSIZ),XD8(MVSIZ)
      REAL(C_DOUBLE) YD1(MVSIZ),YD2(MVSIZ),YD3(MVSIZ),YD4(MVSIZ),YD5(MVSIZ),YD6(MVSIZ),YD7(MVSIZ),YD8(MVSIZ)
      REAL(C_DOUBLE) ZD1(MVSIZ),ZD2(MVSIZ),ZD3(MVSIZ),ZD4(MVSIZ),ZD5(MVSIZ),ZD6(MVSIZ),ZD7(MVSIZ),ZD8(MVSIZ)
      REAL(C_DOUBLE) RX(MVSIZ),RY(MVSIZ),RZ(MVSIZ)
      REAL(C_DOUBLE) SX(MVSIZ),SY(MVSIZ),SZ(MVSIZ)
      REAL(C_DOUBLE) TX(MVSIZ),TY(MVSIZ),TZ(MVSIZ)
      REAL(C_DOUBLE) E1X(MVSIZ),E1Y(MVSIZ),E1Z(MVSIZ),E2X(MVSIZ),E2Y(MVSIZ),E2Z(MVSIZ),E3X(MVSIZ),E3Y(MVSIZ),E3Z(MVSIZ)
      REAL(C_DOUBLE) F1X(MVSIZ),F1Y(MVSIZ),F1Z(MVSIZ),F2X(MVSIZ),F2Y(MVSIZ),F2Z(MVSIZ)
      INTEGER IX1(1),IX2(1),IX3(1),IX4(1),IX5(1),IX6(1),IX7(1),IX8(1)
      INTERFACE
       SUBROUTINE SOLID18_REFERENCE_NATIVE(X,RHO,VALUES,PERM,STATUS)
     . BIND(C,NAME="solid18_reference_native")
        USE ISO_C_BINDING
        REAL(C_DOUBLE),INTENT(IN) :: X(3,8),RHO
        REAL(C_DOUBLE),INTENT(OUT) :: VALUES(148)
        INTEGER(C_INT),INTENT(OUT) :: PERM(8),STATUS
       END SUBROUTINE
      END INTERFACE
#include "gauss_data.inc"
      LFT=1
      LLT=1
      NFT=0
      ISORTH=0
      ISROT=0
      JHBE=17
      MTN=90
      JCVT=1
      NPT=222
      JALE=0
      JEUL=0
      JTHE=0
      NPROPG=12
      NPROPGI=1
      STATUS=0
      VALUES=ZERO
      PERM=0
      TAGS=0
      IXS=1
      DO N=1,8
        IXS(N+1,1)=N
      ENDDO
      JLAG=0
      IGTYP=1
      ISMSTR=10
      ITY=1
      ISNOD=8
      IINT=2
      ICPRE=0
      MLW=90
      NG=1
      GBUF=>ELBUF_TAB(1)%GBUF
#include "saved_tags.inc"
#include "allocation_tags.inc"
#include "allocate_global_saved.inc"
#include "allocate_global_jac.inc"
      BUFLY=>ELBUF_TAB(1)%BUFLY(1)
      DO IR=1,2
       DO IS=1,2
        DO IT=1,2
         LBUF=>BUFLY%LBUF(IR,IS,IT)
#include "allocate_local_jac.inc"
#include "allocate_local_pij.inc"
        ENDDO
       ENDDO
      ENDDO
      TAGS=[GBUF%G_JAC_I,BUFLY%L_JAC_I,BUFLY%L_PIJ,GBUF%G_ETOTSH]
      CALL LAW90_START_SGSAVINI(8,X,IXS,GBUF%SMSTR,NEL)
      IF(LAW90_START_CHECKVOLUME_8N(X,IXS(:,1))==ZERO)THEN
        STATUS=1
        GOTO 900
      ENDIF
#include "connectivity.inc"
#include "gather.inc"
#include "double_coordinates.inc"
      GEOMETRY_ERRORS=0
      CALL LAW90_START_S8ZJAC_IC(
     . XD1,XD2,XD3,XD4,
     . XD5,XD6,XD7,XD8,
     . YD1,YD2,YD3,YD4,
     . YD5,YD6,YD7,YD8,
     . ZD1,ZD2,ZD3,ZD4,
     . ZD5,ZD6,ZD7,ZD8,
     . CJ(1,1),CJ(1,2),CJ(1,3),CJ(1,4),
     . CJ(1,5),CJ(1,6),CJ(1,7),CJ(1,8),
     . CJ(1,9),H(1,1,1),H(1,1,2),H(1,1,3),
     . GBUF%JAC_I)
#include "frame.inc"
#include "project.inc"
      CALL LAW90_START_S8ZPIJ_IC(
     . XD1,XD2,XD3,XD4,
     . XD5,XD6,XD7,XD8,
     . YD1,YD2,YD3,YD4,
     . YD5,YD6,YD7,YD8,
     . ZD1,ZD2,ZD3,ZD4,
     . ZD5,ZD6,ZD7,ZD8,
     . CJ(1,1),CJ(1,2),CJ(1,3),CJ(1,4),
     . CJ(1,5),CJ(1,6),CJ(1,7),CJ(1,8),
     . CJ(1,9),H(1,1,1),H(1,1,2),H(1,1,3),
     . CP(1,1,1),CP(1,2,1),CP(1,3,1),CP(1,4,1),
     . CP(1,1,2),CP(1,2,2),CP(1,3,2),CP(1,4,2),
     . CP(1,1,3),CP(1,2,3),CP(1,3,3),CP(1,4,3))
      DO IR=1,2
       DO IS=1,2
        DO IT=1,2
         ZR=A_GAUSS(IR,2)
         ZS=A_GAUSS(IS,2)
         ZT=A_GAUSS(IT,2)
         WT=W_GAUSS(IT,2)
         WI=W_GAUSS(IR,2)*W_GAUSS(IS,2)*WT
         LBUF=>BUFLY%LBUF(IR,IS,IT)
      CALL LAW90_START_S8ZJAC_I3(
     . ZR,ZS,ZT,WI,
     . H(1,1,1),H(1,1,2),H(1,1,3),CJ(1,1),
     . CJ(1,2),CJ(1,3),CJ(1,4),CJ(1,5),
     . CJ(1,6),CJ(1,7),CJ(1,8),CJ(1,9),
     . LBUF%JAC_I,BUFLY%L_PIJ,LBUF%PIJ,LLT)
        ENDDO
       ENDDO
      ENDDO
      PM=ZERO
      DO I=LFT,LLT
       NU(I)=MIN(HALF,PM(21,MXT(I)))
      ENDDO
      CALL LAW90_START_S8E_PIJ(
     . 2,2,2,8,
     . LLT,CP(1,1,1),CP(1,2,1),CP(1,3,1),
     . CP(1,4,1),CP(1,1,2),CP(1,2,2),CP(1,3,2),
     . CP(1,4,2),CP(1,1,3),CP(1,2,3),CP(1,3,3),
     . CP(1,4,3),E1X,E1Y,E1Z,
     . E2X,E2Y,E2Z,E3X,
     . E3Y,E3Z,NU,ELBUF_TAB(1))
      IF(GEOMETRY_ERRORS/=0)THEN
        STATUS=GEOMETRY_ERRORS
        GOTO 900
      ENDIF
C     Reuse qualified native selected geometry/SVALUE0/SMASS3 after PIJ startup.
      CALL SOLID18_REFERENCE_NATIVE(X,RHO0,VALUES(1:148),CHECKPERM,STATUS)
      IF(STATUS/=0)GOTO 900
      DO N=1,8
        PERM(N)=IXS(N+1,1)-1
        IF(PERM(N)/=CHECKPERM(N))THEN
          STATUS=999
          GOTO 900
        ENDIF
      ENDDO
      VALUES(149:158)=GBUF%JAC_I
      DO IR=1,2
       DO IS=1,2
        DO IT=1,2
         IP=IR+2*(IS-1)+4*(IT-1)
         VALUES(159+72*(IP-1):230+72*(IP-1))=BUFLY%LBUF(IR,IS,IT)%PIJ
        ENDDO
       ENDDO
      ENDDO
      DO N=1,7
        VALUES(735+3*(N-1))=GBUF%SMSTR(N)
        VALUES(736+3*(N-1))=GBUF%SMSTR(N+7)
        VALUES(737+3*(N-1))=GBUF%SMSTR(N+14)
      ENDDO
 900  CONTINUE
      DO IR=1,2
       DO IS=1,2
        DO IT=1,2
         LBUF=>BUFLY%LBUF(IR,IS,IT)
         DEALLOCATE(LBUF%JAC_I,LBUF%PIJ)
        ENDDO
       ENDDO
      ENDDO
      DEALLOCATE(GBUF%SMSTR,GBUF%JAC_I)
      END
