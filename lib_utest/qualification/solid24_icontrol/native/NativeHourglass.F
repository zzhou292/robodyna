! SPDX-License-Identifier: AGPL-3.0-or-later
      SUBROUTINE IC1_HOURGLASS(PM,M,G,STEP,VOLUME0,OLD,HOUR,ENERGY,F,STI)
      USE HEPH_NATIVE_PACKETS
      USE IC1_NATIVE_INTERFACES
      IMPLICIT NONE
      REAL(KIND=8),INTENT(IN)::PM(110,1),M(33),STEP,VOLUME0,OLD(3,4)
      TYPE(GEOMETRY_PACKET),INTENT(IN)::G
      REAL(KIND=8),INTENT(OUT)::HOUR(1,3,4),ENERGY(MVSIZ),F(MVSIZ,8,3),STI(MVSIZ)
      REAL(KIND=8)::RHO(1),VOL0(MVSIZ),OFF(MVSIZ),CXX(MVSIZ),STRHG(1,18)
      INTEGER::MAT(MVSIZ)
      RHO=M(7)
      VOL0=ZERO
      VOL0(1)=VOLUME0
      OFF=ONE
      CXX=ZERO
      CXX(1)=M(20)
      MAT=1
      HOUR(1,:,:)=OLD
      ENERGY=ZERO
      ENERGY(1)=M(8)
      F=ZERO
      STI=ZERO
      STI(1)=M(33)
      STRHG=ZERO
      CALL IC1_NATIVE_SZHOUR_CTL(
     . PM,RHO,OFF,G%V(1,1,1),
     . G%V(1,2,1),G%V(1,3,1),G%V(1,4,1),G%V(1,5,1),
     . G%V(1,6,1),G%V(1,7,1),G%V(1,8,1),G%V(1,1,2),
     . G%V(1,2,2),G%V(1,3,2),G%V(1,4,2),G%V(1,5,2),
     . G%V(1,6,2),G%V(1,7,2),G%V(1,8,2),G%V(1,1,3),
     . G%V(1,2,3),G%V(1,3,3),G%V(1,4,3),G%V(1,5,3),
     . G%V(1,6,3),G%V(1,7,3),G%V(1,8,3),F(1,1,1),
     . F(1,1,2),F(1,1,3),F(1,2,1),F(1,2,2),
     . F(1,2,3),F(1,3,1),F(1,3,2),F(1,3,3),
     . F(1,4,1),F(1,4,2),F(1,4,3),F(1,5,1),
     . F(1,5,2),F(1,5,3),F(1,6,1),F(1,6,2),
     . F(1,6,3),F(1,7,1),F(1,7,2),F(1,7,3),
     . F(1,8,1),F(1,8,2),F(1,8,3),G%PH(1,1,1),
     . G%PH(1,1,2),G%PH(1,1,3),G%PH(1,2,1),G%PH(1,2,2),
     . G%PH(1,2,3),G%PH(1,3,1),G%PH(1,3,2),G%PH(1,3,3),
     . G%PH(1,4,1),G%PH(1,4,2),G%PH(1,4,3),G%VOLUME,
     . HOUR,42,STEP,MAT,
     . CXX,ENERGY,110,1,
     . STRHG,G%JAC(1,1),G%JAC(1,5),G%JAC(1,7),
     . VOL0,ZEP1,STI,1)
      END
