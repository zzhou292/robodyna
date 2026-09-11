! SPDX-License-Identifier: AGPL-3.0-or-later
      SUBROUTINE SOLID6Z_REFERENCE_NATIVE(X,RHO0,IMAS,VALUES,PERM,STATUS)
     . BIND(C,NAME="solid6z_reference_native")
      USE ISO_C_BINDING
      USE, INTRINSIC :: IEEE_ARITHMETIC
      USE SOLID6Z_REF_ELEMENT_MOD, ONLY: NIXS
      USE SOLID6Z_REF_MESSAGE_MOD, ONLY: GEOMETRY_ERRORS
      USE SOLID6Z_REF_S6ZCOOR3_MOD
      USE SOLID6Z_REF_S6ZRCOOR3_MOD
      USE SOLID6Z_REF_S6ZJACIDP_MOD
      USE SOLID6Z_REF_S6ZDERI3_MOD
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "com01_c.inc"
#include "com04_c.inc"
#include "vect01_c.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,6),RHO0
      INTEGER(C_INT), INTENT(IN) :: IMAS
      REAL(C_DOUBLE), INTENT(OUT) :: VALUES(47)
      INTEGER(C_INT), INTENT(OUT) :: PERM(6),STATUS
      INTEGER I,N,K,NEL,IXS(NIXS,1),MXT(1),NGEO(1),NGL(1),NC(MVSIZ,6),IPART(1)
      REAL(C_DOUBLE) R(1,9),E(1,9),F(1,6),XYZ(1,18),XD(1,18),JAC(10,1)
      REAL(C_DOUBLE) TEMP0(1),TEMP(6),VOL(1),VZL(1),DX(1),DET(1)
      REAL(C_DOUBLE) RHO(1),V(3,6),PARTSAV(20,1),MS(6),MSS(8,1)
      REAL(C_DOUBLE) RHOCP(1),MCP(6),MCPS(8,1),MSSA(1),FILL(1)
      REAL(C_DOUBLE), EXTERNAL :: SOLID6Z_REF_CHECKVOLUME_6N
      INTERFACE
        SUBROUTINE SOLID6Z_REF_S6MASS3(RHO,MS,PARTSAV,X,V,IPART,MSS,
     .    RHOCP,MCP,MCPS,MSSA,FILL,VOLU,N1,N2,N3,N4,N5,N6,IMAS_DS)
          USE ISO_C_BINDING
          INTEGER IMAS_DS,IPART(*),N1(*),N2(*),N3(*),N4(*),N5(*),N6(*)
          REAL(C_DOUBLE) RHO(*),MS(*),PARTSAV(20,*),X(3,*),V(3,*),MSS(8,*)
          REAL(C_DOUBLE) RHOCP(*),MCP(*),MCPS(8,*),MSSA(*),FILL(*),VOLU(*)
        END SUBROUTINE
      END INTERFACE
      LFT=1
      LLT=1
      JTHE=0
      IREST_MSELT=0
      NEL=1
      NUMNOD=6
      STATUS=0
      VALUES=ZERO
      PERM=0
      GEOMETRY_ERRORS=0
      IXS=1
      IXS(2:9,1)=[1,2,3,1,4,5,6,4]
      R=ZERO
      E=ZERO
      F=ZERO
      XYZ=ZERO
      XD=ZERO
      TEMP0=ZERO
      TEMP=ZERO
      IF(.NOT.ALL(IEEE_IS_FINITE(X)).OR..NOT.IEEE_IS_FINITE(RHO0)
     .   .OR.RHO0<=ZERO)THEN
        STATUS=1
        RETURN
      ENDIF
      IF(SOLID6Z_REF_CHECKVOLUME_6N(X,IXS(:,1))==ZERO)THEN
        STATUS=2
        RETURN
      ENDIF
      CALL SOLID6Z_REF_S6ZCOOR3(X,IXS,NGL,MXT,NGEO,
     . R(:,1),R(:,2),R(:,3),R(:,4),R(:,5),R(:,6),R(:,7),R(:,8),R(:,9),
     . E(:,1),E(:,2),E(:,3),E(:,4),E(:,5),E(:,6),E(:,7),E(:,8),E(:,9),
     . F(:,1),F(:,2),F(:,3),F(:,4),F(:,5),F(:,6),
     . TEMP0,TEMP,0,
     . NC(:,1),NC(:,2),NC(:,3),NC(:,4),NC(:,5),NC(:,6),
     . XYZ(:,1),XYZ(:,2),XYZ(:,3),XYZ(:,4),XYZ(:,5),XYZ(:,6),XYZ(:,7),XYZ(:,8),XYZ(:,9),XYZ(:,10),XYZ(:,11),XYZ(:,12),XYZ(:,13),XYZ(:,14),XYZ(:,15),XYZ(:,16),XYZ(:,17),XYZ(:,18),
     . XD(:,1),XD(:,2),XD(:,3),XD(:,4),XD(:,5),XD(:,6),XD(:,7),XD(:,8),XD(:,9),XD(:,10),XD(:,11),XD(:,12),XD(:,13),XD(:,14),XD(:,15),XD(:,16),XD(:,17),XD(:,18),
     . NEL,0,6)
      CALL SOLID6Z_REF_S6ZJACIDP(XD(:,1),XD(:,2),XD(:,3),XD(:,4),XD(:,5),XD(:,6),XD(:,7),XD(:,8),XD(:,9),XD(:,10),XD(:,11),XD(:,12),XD(:,13),XD(:,14),XD(:,15),XD(:,16),XD(:,17),XD(:,18),
     . JAC,NEL)
      CALL SOLID6Z_REF_S6ZRCOOR3(X,IXS,NGL,MXT,NGEO,
     . R(:,1),R(:,2),R(:,3),R(:,4),R(:,5),R(:,6),R(:,7),R(:,8),R(:,9),
     . E(:,1),E(:,2),E(:,3),E(:,4),E(:,5),E(:,6),E(:,7),E(:,8),E(:,9),
     . F(:,1),F(:,2),F(:,3),F(:,4),F(:,5),F(:,6),
     . TEMP0,TEMP,0,
     . NC(:,1),NC(:,2),NC(:,3),NC(:,4),NC(:,5),NC(:,6),
     . XYZ(:,1),XYZ(:,2),XYZ(:,3),XYZ(:,4),XYZ(:,5),XYZ(:,6),XYZ(:,7),XYZ(:,8),XYZ(:,9),XYZ(:,10),XYZ(:,11),XYZ(:,12),XYZ(:,13),XYZ(:,14),XYZ(:,15),XYZ(:,16),XYZ(:,17),XYZ(:,18),
     . XD(:,1),XD(:,2),XD(:,3),XD(:,4),XD(:,5),XD(:,6),XD(:,7),XD(:,8),XD(:,9),XD(:,10),XD(:,11),XD(:,12),XD(:,13),XD(:,14),XD(:,15),XD(:,16),XD(:,17),XD(:,18),
     . NEL,6,0)
      CALL SOLID6Z_REF_S6ZDERI3(NEL,VOL,VZL,NGL,DX,DET,
     . XD(:,1),XD(:,2),XD(:,3),XD(:,4),XD(:,5),XD(:,6),XD(:,7),XD(:,8),XD(:,9),XD(:,10),XD(:,11),XD(:,12),XD(:,13),XD(:,14),XD(:,15),XD(:,16),XD(:,17),XD(:,18))
      IF(GEOMETRY_ERRORS/=0.OR..NOT.ALL(IEEE_IS_FINITE(XD)))THEN
        STATUS=3
        RETURN
      ENDIF
      RHO=RHO0
      V=ZERO
      PARTSAV=ZERO
      MS=ZERO
      MSS=ZERO
      RHOCP=ZERO
      MCP=ZERO
      MCPS=ZERO
      MSSA=ZERO
      FILL=ONE
      IPART=1
      CALL SOLID6Z_REF_S6MASS3(RHO,MS,PARTSAV,X,V,IPART,MSS,RHOCP,MCP,MCPS,MSSA,FILL,DET,
     . NC(:,1),NC(:,2),NC(:,3),NC(:,4),NC(:,5),NC(:,6),
     . IMAS)
C     Observations only: row-major world-frame columns and node-major positions.
      DO K=1,3
        DO N=1,3
          VALUES(3*(K-1)+N)=E(1,3*(N-1)+K)
        ENDDO
      ENDDO
      DO N=1,6
        DO K=1,3
          VALUES(9+3*(N-1)+K)=XD(1,6*(K-1)+N)
        ENDDO
      ENDDO
      VALUES(28:37)=JAC(:,1)
      VALUES(38)=VOL(1)
      VALUES(39)=VZL(1)
      VALUES(40)=DX(1)
      DO N=1,6
        I=N
        IF(N>3)I=N+1
        PERM(N)=NC(1,N)-1
        VALUES(40+NC(1,N))=MSS(I,1)
      ENDDO
      VALUES(47)=PARTSAV(1,1)
      IF(.NOT.ALL(IEEE_IS_FINITE(VALUES)))STATUS=4
      END SUBROUTINE
