! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact SRCOOR3 selected statements around complete native geometry/mass leaves.
      SUBROUTINE SOLID18_REFERENCE_NATIVE(X,RHO0,VALUES,PERM,STATUS)
     . BIND(C,NAME="solid18_reference_native")
      USE ISO_C_BINDING
      USE SOLID18_REF_ELEMENT_MOD, ONLY: NIXS
      USE SOLID18_REF_MESSAGE_MOD, ONLY: GEOMETRY_ERRORS
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "param_c.inc"
#include "com01_c.inc"
#include "vect01_c.inc"
#include "scr05_c.inc"
      REAL(C_DOUBLE), INTENT(IN) :: X(3,8),RHO0
      REAL(C_DOUBLE), INTENT(OUT) :: VALUES(148)
      INTEGER(C_INT), INTENT(OUT) :: PERM(8),STATUS
      INTEGER I,J,K,N,IXS(NIXS,1),MXT(1),NGEO(1),NGL(1)
      INTEGER IPART(1),IGEO(1,1),JHBE,IR,IS,IT,IP
      REAL(C_DOUBLE) SOLID18_REF_CHECKVOLUME_8N,XL,YL,ZL
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
      REAL(C_DOUBLE) CJ(1,9),H(4,MVSIZ,3),JP(MVSIZ,8,9),PV(1,8),PVDP(1,8)
      REAL(C_DOUBLE) SMAX(1),VC(1),TOTALV,SIG(1,6),EINT(1),OFF(1),DTX(1)
      REAL(C_DOUBLE) RHOG(1),SIGG(1,6),EINTG(1),OFFG(1),DTXG(1)
      REAL(C_DOUBLE) INERTIA(8),VR(3,8),INS(8,1),RHOF(1),FRAC(1),FILL(1)
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
      JHBE=17
      NPROPG=12
      NPROPGI=1
      IXS=1
      DO N=1,8
        IXS(N+1,1)=N
      ENDDO
      STATUS=0
      VALUES=ZERO
      PERM=0
      IF(SOLID18_REF_CHECKVOLUME_8N(X,IXS(:,1))==ZERO)THEN
        STATUS=1
        RETURN
      ENDIF
#include "connectivity.inc"
#include "gather.inc"
#include "double_coordinates.inc"
#include "frame.inc"
#include "project.inc"
      GEO=ZERO
      IGEO=0
      VEUL=ZERO
      GEOMETRY_ERRORS=0
      DELTAX=EP30
C     Selected INITIA:1235 -> S8ZINIT3:443,478-521,574,592.
      CALL SOLID18_REF_S8ZDERIC3(VC,H(1,1,1),H(1,1,2),H(1,1,3),
     . CJ(1,1),CJ(1,2),CJ(1,3),CJ(1,4),CJ(1,5),CJ(1,6),
     . CJ(1,7),CJ(1,8),CJ(1,9),SMAX,VOLU,NGL,
     . XD1,XD2,XD3,XD4,XD5,XD6,XD7,XD8,
     . YD1,YD2,YD3,YD4,YD5,YD6,YD7,YD8,
     . ZD1,ZD2,ZD3,ZD4,ZD5,ZD6,ZD7,ZD8)
      IF(GEOMETRY_ERRORS/=0)THEN
        STATUS=GEOMETRY_ERRORS
        RETURN
      ENDIF
      CALL SOLID18_REF_S8EJACIP3(H(1,1,1),H(1,1,2),H(1,1,3),
     . CJ(1,1),CJ(1,2),CJ(1,3),CJ(1,4),CJ(1,5),CJ(1,6),
     . CJ(1,7),CJ(1,8),CJ(1,9),
     . JP(1,1,1),JP(1,1,2),JP(1,1,3),JP(1,1,4),JP(1,1,5),JP(1,1,6),
     . JP(1,1,7),JP(1,1,8),JP(1,1,9))
      CALL SOLID18_REF_SCZERO3(RHOG,SIGG,EINTG,1)
      SIG=ZERO
      EINT=ZERO
      OFF=ONE
      OFFG=ONE
      DTX=ZERO
      DTXG=ZERO
      RHO=RHO0
      TOTALV=ZERO
      DO IR=1,2
        DO IS=1,2
          DO IT=1,2
            IP=IR+((IS-1)+(IT-1)*2)*2
            CALL SOLID18_REF_S8EDERI3(PV(1,IP),VEUL,GEO,ONE,
     .        JP(1,IP,1),JP(1,IP,2),JP(1,IP,3),JP(1,IP,4),JP(1,IP,5),
     .        JP(1,IP,6),JP(1,IP,7),JP(1,IP,8),JP(1,IP,9),
     .        SMAX,DELTAX,NGL,PVDP(1,IP))
C           The selected binary64 startup supplies one exact VOL/VOL0DP value.
            IF(PV(1,IP)/=PVDP(1,IP))THEN
              STATUS=999
              RETURN
            ENDIF
            CALL SOLID18_REF_SVALUE0(RHO,PV(1,IP),OFF,SIG,EINT,DTX,
     .        RHOG,VC,OFFG,SIGG,EINTG,DTXG,1)
            TOTALV=TOTALV+PV(1,IP)
          ENDDO
        ENDDO
      ENDDO
      IF(GEOMETRY_ERRORS/=0)THEN
        STATUS=GEOMETRY_ERRORS
        RETURN
      ENDIF
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
      INERTIA=ZERO
      VR=ZERO
      INS=ZERO
      RHOF=ZERO
      FRAC=ZERO
      FILL=ONE
      IPART=1
      CALL SOLID18_REF_SMASS3(RHOG,MS,PARTSAV,X,V,IPART,MSS,VOLU,
     . MSNF,MSSF,INERTIA,VR,INS,WMA,RHOCP,MCP,MCPS,MSSA,RHOF,FRAC,FILL,
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
      K=33
      DO J=1,9
        K=K+1
        VALUES(K)=CJ(1,J)
      ENDDO
      DO N=1,4
        DO J=1,3
          K=K+1
          VALUES(K)=H(N,1,J)
        ENDDO
      ENDDO
      DO IP=1,8
        DO J=1,9
          K=K+1
          VALUES(K)=JP(1,IP,J)
        ENDDO
        K=K+1
        VALUES(K)=PV(1,IP)
      ENDDO
      VALUES(135)=VC(1)
      VALUES(136)=TOTALV
      VALUES(137)=SMAX(1)
      VALUES(138)=DELTAX(1)
      DO N=1,8
        J=IXS(N+1,1)
        PERM(N)=J-1
        VALUES(138+J)=MSS(N,1)
      ENDDO
      VALUES(147)=PARTSAV(1,1)
      VALUES(148)=RHOG(1)
      END SUBROUTINE
