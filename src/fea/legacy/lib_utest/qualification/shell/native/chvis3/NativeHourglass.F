C     Qualification wrapper only; original CHVIS3 arithmetic is not edited.
C     Exact upstream includes define COMMON layout. Every context symbol read
C     by the admitted CHVIS3 branch is initialized explicitly for each call.
      FUNCTION CRASH_CHVIS3_NATIVE_IMPL(OR_N,OR_MODE,OR_CTRL,
     .  OR_FIELDS,OR_VIN,OR_RIN,OR_HIN,OR_HOUT,OR_FOUT,OR_COUT,
     .  OR_WOUT) RESULT(OR_STATUS)
     .  BIND(C,NAME="crash_chvis3_native_impl")
      USE ISO_C_BINDING, ONLY: C_INT,C_DOUBLE
      USE, INTRINSIC :: IEEE_ARITHMETIC, ONLY: IEEE_IS_FINITE
      USE ELEMENT_MOD, ONLY: NIXC
#include "implicit_f.inc"
#include "comlock.inc"
#include "mvsiz_p.inc"
#include "com04_c.inc"
#include "com06_c.inc"
#include "param_c.inc"
#include "remesh_c.inc"
#include "scr02_c.inc"
#include "scr06_c.inc"
#include "scr14_c.inc"
#include "scr16_c.inc"
#include "scr17_c.inc"
#include "sms_c.inc"
      INTEGER(C_INT),VALUE :: OR_N,OR_MODE
      INTEGER(C_INT) :: OR_STATUS
      REAL(C_DOUBLE),INTENT(IN) :: OR_CTRL(3),OR_FIELDS(OR_N,20)
      REAL(C_DOUBLE),INTENT(IN) :: OR_VIN(OR_N,12),OR_RIN(OR_N,12)
      REAL(C_DOUBLE),INTENT(IN) :: OR_HIN(OR_N,5)
      REAL(C_DOUBLE),INTENT(OUT) :: OR_HOUT(OR_N,5)
      REAL(C_DOUBLE),INTENT(OUT) :: OR_FOUT(OR_N,12)
      REAL(C_DOUBLE),INTENT(OUT) :: OR_COUT(OR_N,12),OR_WOUT(OR_N)
      INTEGER OR_I,OR_J,OR_IXC(NIXC,MVSIZ),OR_PART(MVSIZ)
      DOUBLE PRECISION OR_F(MVSIZ,20),OR_V(MVSIZ,12)
      DOUBLE PRECISION OR_R(MVSIZ,12),OR_H(MVSIZ,5)
      DOUBLE PRECISION OR_HF(MVSIZ,9),OR_HB(MVSIZ,8)
      DOUBLE PRECISION OR_OFF(MVSIZ),OR_THK2(MVSIZ)
      DOUBLE PRECISION OR_A11(MVSIZ),OR_G(MVSIZ),OR_ALPE(MVSIZ)
      DOUBLE PRECISION OR_STI(MVSIZ),OR_STIR(MVSIZ)
      DOUBLE PRECISION OR_VISC(MVSIZ),OR_Z2(MVSIZ)
      DOUBLE PRECISION OR_ANI(MVSIZ),OR_SAVED(8,16),OR_CHECK
C     The public C++ shim validates all pointers, sizes and physical scalars.
      OR_STATUS=1
      IF(OR_N<1.OR.OR_N>16.OR.OR_N>MVSIZ) RETURN
      IF(OR_MODE/=1.AND.OR_MODE/=2) RETURN
      IF(KIND(ZERO)/=C_DOUBLE.OR.NIXC/=7) RETURN
      HVISC=OR_CTRL(1)
      HVLIN=OR_CTRL(2)
      HELAS=OR_CTRL(3)
      NODADT=0
      IDT1SH=0
      IDTMINS=0
      NADMESH=0
      NUMELS=0
      NPSAV=8
      EHOUR=ZERO
      OR_F=ZERO
      OR_V=ZERO
      OR_R=ZERO
      OR_H=ZERO
      OR_HF=ZERO
      OR_HB=ZERO
      OR_OFF=ONE
      OR_THK2=ZERO
      OR_A11=ZERO
      OR_G=ZERO
      OR_ALPE=ONE
      OR_STI=ZERO
      OR_STIR=ZERO
      OR_VISC=ONE
      OR_Z2=ZERO
      OR_ANI=ZERO
      OR_SAVED=ZERO
      OR_IXC=0
      OR_PART=0
      OR_F(1:OR_N,:)=OR_FIELDS
      OR_V(1:OR_N,:)=OR_VIN
      OR_R(1:OR_N,:)=OR_RIN
      OR_H(1:OR_N,:)=OR_HIN
      DO OR_I=1,OR_N
        OR_IXC(2:5,OR_I)=(/1,2,3,4/)
        OR_PART(OR_I)=OR_I
        OR_THK2(OR_I)=OR_F(OR_I,8)**2
        OR_A11(OR_I)=OR_F(OR_I,9)/(ONE-OR_F(OR_I,10)**2)
        OR_G(OR_I)=OR_F(OR_I,9)/(TWO*(ONE+OR_F(OR_I,10)))
      ENDDO
C     Arguments preserve native ordering; arrays with native MVSIZ dimensions
C     are padded. HOUR's leading dimension is passed as NEL=MVSIZ, while the
C     active loop remains JFT=1,JLT=OR_N. No rounded or guessed COMMON layout.
      CALL CHVIS3(
     .  1,OR_N,OR_F(1,8),OR_H,OR_OFF,
     .  OR_F(1,1),OR_F(1,2),OR_F(1,3),OR_F(1,4),
     .  OR_IXC,OR_F(1,20),OR_F(1,12),OR_F(1,11),OR_STI,
     .  OR_V(1,1),OR_V(1,4),OR_V(1,7),OR_V(1,10),
     .  OR_V(1,2),OR_V(1,5),OR_V(1,8),OR_V(1,11),
     .  OR_V(1,3),OR_V(1,6),OR_V(1,9),OR_V(1,12),
     .  OR_F(1,5),OR_F(1,8),OR_F(1,6),OR_F(1,7),
     .  OR_F(1,13),OR_Z2,OR_ANI,OR_STIR,OR_VISC,OR_G,OR_A11,
     .  OR_F(1,14),OR_F(1,15),OR_F(1,16),OR_F(1,9),OR_F(1,10),
     .  OR_THK2,OR_ALPE,
     .  OR_HF(1,1),OR_HF(1,2),OR_HF(1,3),
     .  OR_HF(1,4),OR_HF(1,5),OR_HF(1,6),
     .  OR_HF(1,7),OR_HF(1,8),OR_HF(1,9),
     .  OR_HB(1,1),OR_HB(1,2),OR_HB(1,3),OR_HB(1,4),
     .  OR_HB(1,5),OR_HB(1,6),OR_HB(1,7),OR_HB(1,8),
     .  OR_R(1,1),OR_R(1,4),OR_R(1,7),OR_R(1,10),
     .  OR_R(1,2),OR_R(1,5),OR_R(1,8),OR_R(1,11),
     .  OR_PART,OR_SAVED,1,0,OR_MODE,0,
     .  OR_F(1,17),OR_F(1,18),OR_F(1,19),0,0,OR_A11,MVSIZ)
      OR_STATUS=2
      IF(.NOT.ALL(IEEE_IS_FINITE(OR_H(1:OR_N,:)))) RETURN
      IF(.NOT.ALL(IEEE_IS_FINITE(OR_HF(1:OR_N,:)))) RETURN
      IF(.NOT.ALL(IEEE_IS_FINITE(OR_HB(1:OR_N,:)))) RETURN
      IF(.NOT.ALL(IEEE_IS_FINITE(OR_ANI(1:OR_N)))) RETURN
      OR_CHECK=SUM(OR_ANI(1:OR_N))
      IF(.NOT.IEEE_IS_FINITE(EHOUR)) RETURN
      IF(ABS(OR_CHECK-EHOUR)>1D-12*MAX(ONE,ABS(OR_CHECK))) RETURN
      IF(ANY(ABS(OR_SAVED(8,1:OR_N)-OR_ANI(1:OR_N))>
     .    1D-12*MAX(ONE,ABS(OR_ANI(1:OR_N))))) RETURN
      OR_HOUT=OR_H(1:OR_N,:)
      OR_FOUT=ZERO
      OR_COUT=ZERO
      DO OR_I=1,OR_N
        DO OR_J=1,3
          OR_FOUT(OR_I,OR_J)=-OR_HF(OR_I,3*(OR_J-1)+1)
          OR_FOUT(OR_I,3+OR_J)=-OR_HF(OR_I,3*(OR_J-1)+2)
          OR_FOUT(OR_I,6+OR_J)=-OR_HF(OR_I,3*(OR_J-1)+3)
          OR_FOUT(OR_I,9+OR_J)=-SUM(OR_FOUT(OR_I,
     .      (/OR_J,3+OR_J,6+OR_J/)))
        ENDDO
        DO OR_J=1,4
          OR_COUT(OR_I,3*(OR_J-1)+1)=-OR_HB(OR_I,OR_J)
          OR_COUT(OR_I,3*(OR_J-1)+2)=-OR_HB(OR_I,4+OR_J)
        ENDDO
      ENDDO
      OR_WOUT=OR_ANI(1:OR_N)
      OR_STATUS=0
      END
