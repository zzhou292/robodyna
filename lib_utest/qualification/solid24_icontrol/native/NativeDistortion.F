! SPDX-License-Identifier: AGPL-3.0-or-later
      SUBROUTINE IC1_DISTORTION(PM,M,G,XWORLD,VWORLD,STEP,E_DISTOR,F,STI)
      USE HEPH_NATIVE_PACKETS
      USE IC1_NATIVE_OBSERVATIONS
      USE IC1_NATIVE_INTERFACES
      USE IC1_NATIVE_SDISTOR_INI_MOD,ONLY:IC1_NATIVE_SDISTOR_INI
      IMPLICIT NONE
      REAL(KIND=8),INTENT(IN)::PM(110,1),M(33),XWORLD(3,8),VWORLD(3,8),STEP
      TYPE(GEOMETRY_PACKET),INTENT(IN)::G
      REAL(KIND=8),INTENT(INOUT)::E_DISTOR(1),F(MVSIZ,8,3),STI(MVSIZ)
      REAL(KIND=8)::STI_C(MVSIZ),SIG(1,6),RHO(1),CXX(MVSIZ),OFF(MVSIZ),OFFG(1)
      REAL(KIND=8)::LL(MVSIZ),FLD(MVSIZ),MU,FQMAX,X(MVSIZ,8,3),V(MVSIZ,8,3)
      INTEGER::MAT(MVSIZ),ISTAB(MVSIZ),N,K
      SIG(1,:)=M(1:6)
      RHO=M(7)
      CXX=ZERO
      CXX(1)=M(20)
      OFF=ONE
      OFFG=ONE
      MAT=1
      ISTAB=0
      LL=ZERO
      FLD=ZERO
      STI_C=ZERO
      X=ZERO
      V=ZERO
      DO N=1,8
        DO K=1,3
          X(1,N,K)=XWORLD(K,N)
          V(1,N,K)=VWORLD(K,N)
        ENDDO
      ENDDO
      CALL IC1_NATIVE_SDISTOR_INI(1,STI_C,110,1,10,MAT,ISTAB,PM,SIG,RHO,CXX,OFF,
     . OFFG,LL,G%VOLUME,FLD,MU,FQMAX)
      DISTORTION_SIGMA=SIG(1,:)
      DISTORTION_PARAMETERS=[STI_C(1),FLD(1),LL(1),MU,G%VOLUME(1)]
      DISTORTION_FLAG=ISTAB(1)
      CALL IC1_NATIVE_S8FOR_DISTOR(
     . X(1,1,1),X(1,2,1),X(1,3,1),X(1,4,1),
     . X(1,5,1),X(1,6,1),X(1,7,1),X(1,8,1),
     . X(1,1,2),X(1,2,2),X(1,3,2),X(1,4,2),
     . X(1,5,2),X(1,6,2),X(1,7,2),X(1,8,2),
     . X(1,1,3),X(1,2,3),X(1,3,3),X(1,4,3),
     . X(1,5,3),X(1,6,3),X(1,7,3),X(1,8,3),
     . V(1,1,1),V(1,2,1),V(1,3,1),V(1,4,1),
     . V(1,5,1),V(1,6,1),V(1,7,1),V(1,8,1),
     . V(1,1,2),V(1,2,2),V(1,3,2),V(1,4,2),
     . V(1,5,2),V(1,6,2),V(1,7,2),V(1,8,2),
     . V(1,1,3),V(1,2,3),V(1,3,3),V(1,4,3),
     . V(1,5,3),V(1,6,3),V(1,7,3),V(1,8,3),
     . F(1,1,1),F(1,2,1),F(1,3,1),F(1,4,1),
     . F(1,5,1),F(1,6,1),F(1,7,1),F(1,8,1),
     . F(1,1,2),F(1,2,2),F(1,3,2),F(1,4,2),
     . F(1,5,2),F(1,6,2),F(1,7,2),F(1,8,2),
     . F(1,1,3),F(1,2,3),F(1,3,3),F(1,4,3),
     . F(1,5,3),F(1,6,3),F(1,7,3),F(1,8,3),
     . STI,STI_C,FLD,MU,
     . LL,ISTAB,FQMAX,1,
     . E_DISTOR,STEP)
      END
