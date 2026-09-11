! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete SMASS3 slot values, then the exact selected SPMD_MSIN node scatter.
      SUBROUTINE REAR18_SCATTER(X,RHO,VOLUME,NODES,SLOT_MASS,MASS)
      USE ISO_C_BINDING, ONLY: C_DOUBLE
      USE SOLID18_REF_ELEMENT_MOD, ONLY: NIXS
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "param_c.inc"
#include "com01_c.inc"
#include "vect01_c.inc"
#include "scr05_c.inc"
      REAL(C_DOUBLE),INTENT(IN) :: X(3,8),RHO,VOLUME
      INTEGER,INTENT(IN) :: NODES(8)
      REAL(C_DOUBLE),INTENT(OUT) :: SLOT_MASS(8),MASS(8)
      REAL(C_DOUBLE) MS(8)
      REAL(C_DOUBLE) V(3,8),PARTSAV(20,1),MSS(8,1)
      REAL(C_DOUBLE) MSNF(8),MSSF(8,1),WMA(8),RHOCP(1)
      REAL(C_DOUBLE) MCP(8),MCPS(8,1),MSSA(8),INERTIA(8)
      REAL(C_DOUBLE) VR(3,8),INS(8,1),RHOF(1),FRAC(1),FILL(1)
      REAL(C_DOUBLE) RHOG(1),VOLU(1)
      INTEGER IPART(1),NC(1,8)
      INTEGER IXS(NIXS,1),INDEX(1),ITHERM_FE,I,J,K,N
      INTERFACE
        SUBROUTINE SOLID18_REF_SMASS3(RHO,MS,PARTSAV,X,V,IPART,
     .    MSS,VOLU,MSNF,MSSF,IN,VR,INS,WMA,RHOCP,MCP,MCPS,MSSA,
     .    RHOF,FRAC,FILL,NC1,NC2,NC3,NC4,NC5,NC6,NC7,NC8)
          IMPORT C_DOUBLE
          INTEGER IPART(*),NC1(*),NC2(*),NC3(*),NC4(*)
          INTEGER NC5(*),NC6(*),NC7(*),NC8(*)
          REAL(C_DOUBLE) RHO(*),MS(*),PARTSAV(20,*),X(3,*),V(3,*)
          REAL(C_DOUBLE) MSS(8,*),VOLU(*),MSNF(*),MSSF(8,*),IN(*)
          REAL(C_DOUBLE) VR(3,*),INS(8,*),WMA(*),RHOCP(*),MCP(*)
          REAL(C_DOUBLE) MCPS(8,*),MSSA(*),RHOF(*),FRAC(*),FILL(*)
        END SUBROUTINE
      END INTERFACE
      LFT=1
      LLT=1
      NFT=0
      JALE=0
      JEUL=0
      JTHE=0
      ISROT=0
      IRODDL=0
      IREST_MSELT=0
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
      RHOG=RHO
      VOLU=VOLUME
      NC(1,:)=NODES
      CALL SOLID18_REF_SMASS3(RHOG,MS,PARTSAV,X,V,IPART,MSS,VOLU,
     . MSNF,MSSF,INERTIA,VR,INS,WMA,RHOCP,MCP,MCPS,MSSA,RHOF,FRAC,FILL,
     . NC(1,1),NC(1,2),NC(1,3),NC(1,4),NC(1,5),NC(1,6),NC(1,7),NC(1,8))
      SLOT_MASS=MSS(:,1)
C     INITIA calls SPMD_MSIN after SMASS3. This packet has exactly one solid;
C     native element sorting therefore gives INDEX(1)=1. No other family or
C     ADMAS/thermal contribution is supplied. Keep the native K=1..8 loop.
      NUMELS=1
      INDEX=1
      IXS=1
      IXS(2:9,1)=NODES
      ITHERM_FE=0
#include "spmd_mass.inc"
      ENDIF
      MASS=MS
      END SUBROUTINE
