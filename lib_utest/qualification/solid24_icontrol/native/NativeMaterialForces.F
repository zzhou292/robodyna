! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact SFINT3/SRROTA3 call bindings reused from the qualified IC0 native adapter.
      SUBROUTINE IC1_MATERIAL_FORCES(M,G,F)
      USE HEPH_NATIVE_PACKETS
      USE HEPH_NATIVE_INTERFACES
      USE HEPH_NATIVE_ELEMENT_MOD,ONLY:NIXS
      IMPLICIT NONE
      REAL(KIND=8),INTENT(IN)::M(33)
      TYPE(GEOMETRY_PACKET),INTENT(IN)::G
      REAL(KIND=8),INTENT(INOUT)::F(MVSIZ,8,3)
      REAL(KIND=8)::SIGN(1,6),Q(1),SVIS(MVSIZ,6),NORMAL(MVSIZ,6,3)
      REAL(KIND=8)::ZERO_BUFFER(MVSIZ),DFE(MVSIZ,3),PBAK(MVSIZ)
      INTEGER::IXS(NIXS,1),N
      SIGN(1,:)=M(1:6)
      Q=M(9)
      SVIS=ZERO
      NORMAL=ZERO
      ZERO_BUFFER=ZERO
      DFE=ZERO
      PBAK=ZERO
      IXS=1
      DO N=1,8
        IXS(N+1,1)=N
      END DO
      CALL HEPH_NATIVE_SFINT3(
     . SIGN,G%P(1,1,1),G%P(1,2,1),G%P(1,3,1),
     . G%P(1,4,1),G%P(1,1,2),G%P(1,2,2),G%P(1,3,2),
     . G%P(1,4,2),G%P(1,1,3),G%P(1,2,3),G%P(1,3,3),
     . G%P(1,4,3),ZERO_BUFFER,ZERO_BUFFER,ZERO_BUFFER,
     . ZERO_BUFFER,ZERO_BUFFER,ZERO_BUFFER,ZERO_BUFFER,
     . ZERO_BUFFER,ZERO_BUFFER,ZERO_BUFFER,ZERO_BUFFER,
     . ZERO_BUFFER,F(1,1,1),F(1,1,2),F(1,1,3),
     . F(1,2,1),F(1,2,2),F(1,2,3),F(1,3,1),
     . F(1,3,2),F(1,3,3),F(1,4,1),F(1,4,2),
     . F(1,4,3),F(1,5,1),F(1,5,2),F(1,5,3),
     . F(1,6,1),F(1,6,2),F(1,6,3),F(1,7,1),
     . F(1,7,2),F(1,7,3),F(1,8,1),F(1,8,2),
     . F(1,8,3),G%VOLUME,Q,NORMAL(1,1,1),
     . NORMAL(1,2,1),NORMAL(1,3,1),NORMAL(1,4,1),NORMAL(1,5,1),
     . NORMAL(1,6,1),NORMAL(1,1,2),NORMAL(1,2,2),NORMAL(1,3,2),
     . NORMAL(1,4,2),NORMAL(1,5,2),NORMAL(1,6,2),NORMAL(1,1,3),
     . NORMAL(1,2,3),NORMAL(1,3,3),NORMAL(1,4,3),NORMAL(1,5,3),
     . NORMAL(1,6,3),DFE,PBAK,IXS,
     . 1,0,0,0,
     . SVIS)
      CALL HEPH_NATIVE_SRROTA3(
     . G%FRAME(1,1),G%FRAME(1,4),G%FRAME(1,7),G%FRAME(1,2),
     . G%FRAME(1,5),G%FRAME(1,8),G%FRAME(1,3),G%FRAME(1,6),
     . G%FRAME(1,9),F(1,1,1),F(1,2,1),F(1,3,1),
     . F(1,4,1),F(1,5,1),F(1,6,1),F(1,7,1),
     . F(1,8,1),F(1,1,2),F(1,2,2),F(1,3,2),
     . F(1,4,2),F(1,5,2),F(1,6,2),F(1,7,2),
     . F(1,8,2),F(1,1,3),F(1,2,3),F(1,3,3),
     . F(1,4,3),F(1,5,3),F(1,6,3),F(1,7,3),
     . F(1,8,3),1)
      END
