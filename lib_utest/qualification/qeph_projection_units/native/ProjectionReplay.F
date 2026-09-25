! SPDX-License-Identifier: AGPL-3.0-or-later
! Qualification adapter only. Calls existing private original native routines;
! no numerical body is translated or copied here.
      MODULE QE_PROJECTION_REPLAY_MOD
      USE ISO_C_BINDING,ONLY:C_DOUBLE,C_INT
      USE QE_Q1_GEOMETRY_MOD,ONLY:QE_GEOMETRY_
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      CONTAINS
      SUBROUTINE LOAD_GEOMETRY(P,G)
      REAL(C_DOUBLE),INTENT(IN)::P(29)
      TYPE(QE_GEOMETRY_),INTENT(OUT)::G
      INTEGER I,J,N,K
      G%AREA=ZERO
      G%AREA_I=ZERO
      G%X13=ZERO
      G%X24=ZERO
      G%Y13=ZERO
      G%Y24=ZERO
      G%MX13=ZERO
      G%MY13=ZERO
      G%Z1=ZERO
      G%LL=ZERO
      G%L13=ZERO
      G%L24=ZERO
      G%COREL=ZERO
      G%VQ=ZERO
      G%V13=ZERO
      G%V24=ZERO
      G%VHI=ZERO
      G%RLXYZ=ZERO
      G%DI=ZERO
      G%DB=ZERO
      G%VQN=ZERO
      G%RLZ=ZERO
      G%DIZ=ZERO
      G%PLAT=.TRUE.
      G%AREA(1)=P(1)
      G%AREA_I(1)=P(2)
      G%X13(1)=P(3)
      G%X24(1)=P(4)
      G%Y13(1)=P(5)
      G%Y24(1)=P(6)
      G%MX13(1)=P(7)
      G%MY13(1)=P(8)
      G%Z1(1)=P(9)
      G%LL(1)=P(10)
      G%L13(1)=P(11)
      G%L24(1)=P(12)
      DO N=1,4
        G%COREL(1,:,N)=P(13+2*(N-1):14+2*(N-1))
      ENDDO
      DO I=1,3
        DO J=1,3
          G%VQ(1,I,J)=P(21+3*(I-1)+J-1)
        ENDDO
      ENDDO
      END SUBROUTINE

      SUBROUTINE QE_RAW_PROJECTION_RATE(P,R,OMEGA,C,TOL,OUT,PLANAR)
     . BIND(C,NAME="qeph_projection_native_rates")
      USE QE_Q1_ELEMENT_MOD,ONLY:NIXC
#include "scr05_c.inc"
#include "impl1_c.inc"
      REAL(C_DOUBLE),INTENT(IN)::P(29),R(17),OMEGA(3,4),TOL
      INTEGER(C_INT),INTENT(IN)::C(6)
      REAL(C_DOUBLE),INTENT(OUT)::OUT(48)
      INTEGER(C_INT),INTENT(OUT)::PLANAR
      TYPE(QE_GEOMETRY_) G
      INTEGER IXC(NIXC,1),N,K
      REAL(C_DOUBLE) LOCAL_TOL
      IRESP=C(4)
      IMPL_S=C(5)
      IKPROJ=C(6)
      LOCAL_TOL=TOL
      CALL LOAD_GEOMETRY(P,G)
      IXC=1
      IXC(2:5,1)=[1,2,3,4]
      G%V13(1,:)=R(1:3)
      G%V24(1,:)=R(4:6)
      G%VHI(1,:)=R(7:9)
      DO N=1,4
        G%RLXYZ(1,:,N)=R(10+2*(N-1):11+2*(N-1))
      ENDDO
      CALL CZCORP5(4,1,1,OMEGA,C(1),LOCAL_TOL,IXC,G%PLAT,G%AREA,G%AREA_I,
     . G%V13,G%V24,G%VHI,G%RLXYZ,G%VQN,G%VQ,G%X13,G%X24,G%Y13,G%Y24,
     . G%MX13,G%MY13,G%Z1,G%DI,G%DB,G%COREL,G%RLZ,G%LL,G%L13,G%L24,C(2),G%DIZ)
      OUT=0D0
      OUT(1:3)=G%V13(1,:)
      OUT(4:6)=G%V24(1,:)
      OUT(7:9)=G%VHI(1,:)
      DO N=1,4
        OUT(10+2*(N-1):11+2*(N-1))=G%RLXYZ(1,:,N)
      ENDDO
      OUT(18)=G%Z1(1)
      PLANAR=0
      IF(G%PLAT(1)) PLANAR=1
      IF(.NOT.G%PLAT(1))THEN
        OUT(19:24)=G%DI(1,:)
        DO N=1,4
          K=25+3*(N-1)
          OUT(K:K+2)=G%DB(1,:,N)
          K=37+3*(N-1)
          OUT(K:K+2)=G%VQN(1,:,N)
        ENDDO
      ENDIF
      END SUBROUTINE

      SUBROUTINE QE_RAW_PROJECTION_FORCE(P,C,PLANAR,Z,PR,FV,MV,OUT)
     . BIND(C,NAME="qeph_projection_native_forces")
#include "scr05_c.inc"
#include "impl1_c.inc"
      REAL(C_DOUBLE),INTENT(IN)::P(29),Z,PR(30),FV(3,4),MV(2,4)
      INTEGER(C_INT),INTENT(IN)::C(6),PLANAR
      REAL(C_DOUBLE),INTENT(OUT)::OUT(24)
      TYPE(QE_GEOMETRY_) G
      REAL(C_DOUBLE) VF(MVSIZ,3,4),VM(MVSIZ,2,4),FZERO(3,4,1),VMZ(MVSIZ,4)
      REAL(C_DOUBLE) F(MVSIZ,3,4),M(MVSIZ,3,4)
      INTEGER N,K
      IRESP=C(4)
      IMPL_S=C(5)
      IKPROJ=C(6)
      CALL LOAD_GEOMETRY(P,G)
      G%PLAT(1)=PLANAR/=0
      G%Z1(1)=Z
      IF(.NOT.G%PLAT(1))THEN
        G%DI(1,:)=PR(1:6)
        DO N=1,4
          K=7+3*(N-1)
          G%DB(1,:,N)=PR(K:K+2)
          K=19+3*(N-1)
          G%VQN(1,:,N)=PR(K:K+2)
        ENDDO
      ENDIF
      VF=0D0
      VM=0D0
      FZERO=0D0
      VMZ=0D0
      F=0D0
      M=0D0
      VF(1,:,:)=FV
      VM(1,:,:)=MV
      CALL CZPROJN(1,1,G%VQN,G%VQ,VF,VM,G%PLAT,
     . F(1,1,1),F(1,1,2),F(1,1,3),F(1,1,4),
     . F(1,2,1),F(1,2,2),F(1,2,3),F(1,2,4),
     . F(1,3,1),F(1,3,2),F(1,3,3),F(1,3,4),
     . M(1,1,1),M(1,1,2),M(1,1,3),M(1,1,4),
     . M(1,2,1),M(1,2,2),M(1,2,3),M(1,2,4),
     . M(1,3,1),M(1,3,2),M(1,3,3),M(1,3,4),
     . FZERO,G%Z1,G%COREL,G%DI,G%DB,C(3),C(2),G%DIZ,VMZ)
      DO N=1,4
        K=1+3*(N-1)
        OUT(K:K+2)=F(1,:,N)
        K=13+3*(N-1)
        OUT(K:K+2)=M(1,:,N)
      ENDDO
      END SUBROUTINE
      END MODULE
