C     Independent exact native accumulation, including repeated J4=J3.
      SUBROUTINE TL_TIED_ASSEMBLY(FORCES,INITIAL,REPEATED,RESULT)
     . BIND(C,NAME="tl_tied_patch_assembly")
      USE ISO_C_BINDING
      IMPLICIT NONE
      REAL(C_DOUBLE),INTENT(IN) :: FORCES(3,4),INITIAL(3,4)
      REAL(C_DOUBLE),INTENT(OUT) :: RESULT(3,4)
      INTEGER(C_INT),INTENT(IN) :: REPEATED
      INTEGER,PARAMETER :: J1=1,J2=2,J3=3
      INTEGER J4
      REAL(C_DOUBLE) FX(4),FY(4),FZ(4),A(3,4)
      A=INITIAL
      FX=FORCES(1,:);FY=FORCES(2,:);FZ=FORCES(3,:)
      J4=4
      IF (REPEATED==1) J4=3
#include "ForceAssembly.inc"
      RESULT=A
      END
