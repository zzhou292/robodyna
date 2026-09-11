      SUBROUTINE SPECTRUM_NATIVE(INPUT,RATE,COUNT,VALUES,STATUS)
     & BIND(C,NAME='spectrum_native')
      USE ISO_C_BINDING
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      INTEGER(C_INT), INTENT(IN) :: COUNT
      INTEGER(C_INT), INTENT(OUT) :: STATUS
      REAL(C_DOUBLE), INTENT(IN) :: INPUT(6,*),RATE(6,*)
      REAL(C_DOUBLE), INTENT(OUT) :: VALUES(15,*)
      my_real SIG(MVSIZ,6), EVV(MVSIZ,3), DIRPRV(MVSIZ,3,3)
      my_real EPSPXX(MVSIZ),EPSPYY(MVSIZ),EPSPZZ(MVSIZ)
      my_real EPSPXY(MVSIZ),EPSPYZ(MVSIZ),EPSPZX(MVSIZ)
      my_real EP1,EP2,EP3,EP4,EP5,EP6,EPSP(3)
      my_real ERT11,ERT12,ERT13,ERT21,ERT22,ERT23,ERT31,ERT32,ERT33
      INTEGER I,II,K,R,C
      STATUS=1
      IF(COUNT<1.OR.COUNT>MVSIZ) RETURN
      DO I=1,COUNT
        DO K=1,6
          SIG(I,K)=INPUT(K,I)
        ENDDO
        EPSPXX(I)=RATE(1,I)
        EPSPYY(I)=RATE(2,I)
        EPSPZZ(I)=RATE(3,I)
        EPSPXY(I)=RATE(4,I)
        EPSPYZ(I)=RATE(5,I)
        EPSPZX(I)=RATE(6,I)
      ENDDO
      CALL SPECTRUM_REF_VALPVEC_V(SIG,EVV,DIRPRV,COUNT)
      DO I=1,COUNT
        II=I
#include "projection.inc"
        DO K=1,3
          VALUES(K,I)=EVV(I,K)
          VALUES(12+K,I)=EPSP(K)
        ENDDO
        DO R=1,3
          DO C=1,3
            VALUES(3+3*(R-1)+C,I)=DIRPRV(I,R,C)
          ENDDO
        ENDDO
      ENDDO
      STATUS=0
      END

      SUBROUTINE SPECTRUM_CONSTANTS(VALUES)
     & BIND(C,NAME='spectrum_constants')
      USE ISO_C_BINDING
#include "implicit_f.inc"
      REAL(C_DOUBLE),INTENT(OUT) :: VALUES(4)
      my_real CS(2),FLM
      REAL FLMIN
      CALL SPECTRUM_REF_FLOATMIN(CS(1),CS(2),FLMIN)
      FLM=TWO*SQRT(FLMIN)
      VALUES(1)=FLMIN
      VALUES(2)=FLM
      VALUES(3)=EM10
      VALUES(4)=EM20
      END
