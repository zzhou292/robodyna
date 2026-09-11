! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact SRCOOR3 selected statements around complete native geometry/mass leaves.
      SUBROUTINE SOLID24_REFERENCE_NATIVE(X,RHO0,VALUES,PERM,STATUS)
     . BIND(C,NAME="solid24_reference_native")
      USE ISO_C_BINDING
      USE SOLID24_REF_ELEMENT_MOD, ONLY: NIXS
      USE SOLID24_REF_MESSAGE_MOD, ONLY: GEOMETRY_ERRORS
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "param_c.inc"
#include "com01_c.inc"
#include "vect01_c.inc"
#include "scr05_c.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,8),RHO0
      REAL(C_DOUBLE), INTENT(OUT) :: VALUES(44)
      INTEGER(C_INT), INTENT(OUT) :: PERM(8),STATUS
      INTEGER I,J,K,N,IXS(NIXS,1),MXT(1),NGEO(1),NGL(1)
      INTEGER IPART(1),IGEO(1,1),JHBE
      REAL(C_DOUBLE) PX(4,1),PY(4,1),PZ(4,1),VOLDP(1)
      REAL(C_DOUBLE) INER(8),VR(3,8),INS(8,1),RHOF(1),FRAC(1),FILL(1)
      REAL(C_DOUBLE) SOLID24_REF_CHECKVOLUME_8N,XL,YL,ZL
      REAL(C_DOUBLE) V(3,8),RHO(1),VOLU(1),VEUL(51,1),DET(1)
      REAL(C_DOUBLE) DELTAX(1),GEO(12,1),PARTSAV(20,1),MSS(8,1)
      REAL(C_DOUBLE) MS(8),MSNF(8),MSSF(8,1),WMA(8),RHOCP(1)
      REAL(C_DOUBLE) MCP(8),MCPS(8,1),MSSA(8)
      REAL(C_DOUBLE) X1(1),X2(1),X3(1),X4(1),X5(1),X6(1),X7(1),X8(1)
      REAL(C_DOUBLE) XD1(1),XD2(1),XD3(1),XD4(1),XD5(1),XD6(1),XD7(1),XD8(1)
      REAL(C_DOUBLE) Y1(1),Y2(1),Y3(1),Y4(1),Y5(1),Y6(1),Y7(1),Y8(1)
      REAL(C_DOUBLE) YD1(1),YD2(1),YD3(1),YD4(1),YD5(1),YD6(1),YD7(1),YD8(1)
      REAL(C_DOUBLE) Z1(1),Z2(1),Z3(1),Z4(1),Z5(1),Z6(1),Z7(1),Z8(1)
      REAL(C_DOUBLE) ZD1(1),ZD2(1),ZD3(1),ZD4(1),ZD5(1),ZD6(1),ZD7(1),ZD8(1)
      INTEGER IX1(1),IX2(1),IX3(1),IX4(1),IX5(1),IX6(1),IX7(1),IX8(1)
      REAL(C_DOUBLE) RX(1),RY(1),RZ(1),SX(1),SY(1),SZ(1),TX(1),TY(1),TZ(1)
      REAL(C_DOUBLE) E1X(1),E1Y(1),E1Z(1),E2X(1),E2Y(1),E2Z(1),E3X(1),E3Y(1),E3Z(1)
      REAL(C_DOUBLE) F1X(1),F1Y(1),F1Z(1),F2X(1),F2Y(1),F2Z(1)
      LFT=1
      LLT=1
      NFT=0
      JALE=0
      JEUL=0
      JTHE=0
      ISROT=0
      IRODDL=0
      ISORTH=0
      IREST_MSELT=0
      JHBE=24
      NPROPG=12
      NPROPGI=1
      IXS=1
      DO N=1,8
        IXS(N+1,1)=N
      ENDDO
      STATUS=0
      VALUES=ZERO
      PERM=0
      IF(SOLID24_REF_CHECKVOLUME_8N(X,IXS(:,1))==ZERO)THEN
        STATUS=1
        RETURN
      ENDIF
#include "connectivity.inc"
#include "gather.inc"
#include "double_coordinates.inc"
#include "frame.inc"
#include "project.inc"
      GEO=ZERO
      GEO(12,1)=14
      IGEO=0
      VEUL=ZERO
      GEOMETRY_ERRORS=0
      CALL SOLID24_REF_SZDERI3(VOLU,VEUL,GEO,IGEO,
     . XD1,XD2,XD3,XD4,XD5,XD6,XD7,XD8,
     . YD1,YD2,YD3,YD4,YD5,YD6,YD7,YD8,
     . ZD1,ZD2,ZD3,ZD4,ZD5,ZD6,ZD7,ZD8,
     . PX(1,1),PX(2,1),PX(3,1),PX(4,1),
     . PY(1,1),PY(2,1),PY(3,1),PY(4,1),
     . PZ(1,1),PZ(2,1),PZ(3,1),PZ(4,1),
     . RX,RY,RZ,SX,SY,SZ,TZ,NGL,NGEO,DET,VOLDP,1,0,0)
      CALL SOLID24_REF_SDLEN3(
     . X1,X2,X3,X4,X5,X6,X7,X8,
     . Y1,Y2,Y3,Y4,Y5,Y6,Y7,Y8,
     . Z1,Z2,Z3,Z4,Z5,Z6,Z7,Z8,DELTAX,DET)
      IF(GEOMETRY_ERRORS/=0)THEN
        STATUS=GEOMETRY_ERRORS
        RETURN
      ENDIF
      RHO(1)=RHO0
      V=ZERO
      PARTSAV=ZERO
      MS=ZERO
      MSS=ZERO
      MSNF=ZERO
      MSSF=ZERO
      WMA=ZERO
      RHOCP=ZERO
      MCP=ZERO
      MCPS=ZERO
      MSSA=ZERO
      IPART=1
      INER=ZERO
      VR=ZERO
      INS=ZERO
      RHOF=ZERO
      FRAC=ZERO
      FILL=ONE
      CALL SOLID24_REF_SMASS3(RHO,MS,PARTSAV,X,V,IPART,MSS,DET,
     . MSNF,MSSF,INER,VR,INS,WMA,RHOCP,MCP,MCPS,MSSA,RHOF,FRAC,FILL,
     . IX1,IX2,IX3,IX4,IX5,IX6,IX7,IX8)
C     Observation packing only. Frame is row-major with native axes in columns.
      VALUES(1)=E1X(1)
      VALUES(2)=E2X(1)
      VALUES(3)=E3X(1)
      VALUES(4)=E1Y(1)
      VALUES(5)=E2Y(1)
      VALUES(6)=E3Y(1)
      VALUES(7)=E1Z(1)
      VALUES(8)=E2Z(1)
      VALUES(9)=E3Z(1)
      VALUES(10)=X1(1)
      VALUES(11)=Y1(1)
      VALUES(12)=Z1(1)
      VALUES(13)=X2(1)
      VALUES(14)=Y2(1)
      VALUES(15)=Z2(1)
      VALUES(16)=X3(1)
      VALUES(17)=Y3(1)
      VALUES(18)=Z3(1)
      VALUES(19)=X4(1)
      VALUES(20)=Y4(1)
      VALUES(21)=Z4(1)
      VALUES(22)=X5(1)
      VALUES(23)=Y5(1)
      VALUES(24)=Z5(1)
      VALUES(25)=X6(1)
      VALUES(26)=Y6(1)
      VALUES(27)=Z6(1)
      VALUES(28)=X7(1)
      VALUES(29)=Y7(1)
      VALUES(30)=Z7(1)
      VALUES(31)=X8(1)
      VALUES(32)=Y8(1)
      VALUES(33)=Z8(1)
      VALUES(34)=VOLU(1)
      VALUES(35)=DELTAX(1)
      DO N=1,8
        J=IXS(N+1,1)
        PERM(N)=J-1
        VALUES(35+J)=MSS(N,1)
      ENDDO
      VALUES(44)=PARTSAV(1,1)
      END SUBROUTINE
