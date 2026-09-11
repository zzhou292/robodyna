C SPDX-License-Identifier: AGPL-3.0-or-later
C Exact complete R2LEN3 endpoint loop. Current coefficients are explicit packet
C inputs; RBY_NODE is zero for the independently qualified non-rigid profile.
      SUBROUTINE TYPE13_ENDPOINT_STIFFNESS(KT,KR,CT,CR,MASS,JIN,
     . ELEMENT_M,ELEMENT_J,ACTIVE,RESULT)
     . BIND(C,NAME='type13_native_endpoint_stiffness')
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN)::KT,KR,CT,CR,MASS(2),JIN(2)
      REAL(C_DOUBLE),INTENT(IN)::ELEMENT_M,ELEMENT_J,ACTIVE
      REAL(C_DOUBLE),INTENT(OUT)::RESULT(4)
      INTEGER,PARAMETER::JFT=1,JLT=1
      REAL(C_DOUBLE),PARAMETER::ZERO=0D0,TWO=2D0,FOUR=4D0
      INTEGER I,IRB1,IRB2,NC1(1),NC2(1)
      REAL(C_DOUBLE) OFF(1),STI(3,1),STIR(3,1),MS(2),IN(2)
      REAL(C_DOUBLE) RBY_NODE(2),XM(1),XIN(1),XKM(1),XKR(1)
      REAL(C_DOUBLE) XCM(1),XCR(1),MASS2,IN2,A
      NC1=1
      NC2=2
      MS=MASS
      IN=JIN
      RBY_NODE=ZERO
      OFF=ACTIVE
      XKM=KT
      XKR=KR
      XCM=CT
      XCR=CR
      XM=ELEMENT_M
      XIN=ELEMENT_J
      STI=ZERO
      STIR=ZERO
#include "EndpointStiffness.inc"
      RESULT=(/STI(1,1),STI(2,1),STIR(1,1),STIR(2,1)/)
      END
