C     Qualification adapter. Original six routines and COMMON includes are
C     compiled unchanged. The caller supplies a proper planar local frame.
C     One active element per native call honors CCOEF3's homogeneous material
C     batch assumption; the public wrapper handles at most two such calls.
      FUNCTION CRASH_SHELL_PHASE_NATIVE_IMPL(OR_MODE,OR_ITHK,
     . OR_XIN,OR_VIN,OR_WIN,OR_FIN,OR_CFG,OR_SIN,OR_OUT)
     . RESULT(OR_STATUS) BIND(C,NAME="crash_shell_phase_native_impl")
      USE ISO_C_BINDING, ONLY: C_INT,C_DOUBLE
      USE, INTRINSIC :: IEEE_ARITHMETIC, ONLY: IEEE_IS_FINITE
      USE ELEMENT_MOD, ONLY: NIXC
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "param_c.inc"
#include "com08_c.inc"
#include "impl1_c.inc"
#include "com01_c.inc"
#include "scr14_c.inc"
      INTEGER(C_INT),VALUE :: OR_MODE,OR_ITHK
      INTEGER(C_INT) :: OR_STATUS
      REAL(C_DOUBLE),INTENT(IN) :: OR_XIN(3,4),OR_VIN(3,4)
      REAL(C_DOUBLE),INTENT(IN) :: OR_WIN(3,4),OR_FIN(9)
      REAL(C_DOUBLE),INTENT(IN) :: OR_CFG(14),OR_SIN(15)
      REAL(C_DOUBLE),INTENT(OUT) :: OR_OUT(73)
      INTEGER OR_J,OR_K,OR_IXC(NIXC,1),OR_MAT(MVSIZ)
      INTEGER OR_PID(MVSIZ),OR_NGL(MVSIZ),OR_IGEO(98,1)
      DOUBLE PRECISION OR_GEO(38,1),OR_PM(27,1),OR_STACK(20,1)
      DOUBLE PRECISION OR_OFFG(MVSIZ),OR_OFF(MVSIZ),OR_SIGY(MVSIZ)
      DOUBLE PRECISION OR_DT(MVSIZ),OR_WORLD(MVSIZ,12)
      DOUBLE PRECISION OR_VW(MVSIZ,3,4,2),OR_FRAME(MVSIZ,9)
      DOUBLE PRECISION OR_SMSTR(MVSIZ,6),OR_GSTR(MVSIZ,8)
      DOUBLE PRECISION OR_GEOM(MVSIZ,8),OR_LX(MVSIZ,7)
      DOUBLE PRECISION OR_STI(MVSIZ),OR_STIR(MVSIZ)
      DOUBLE PRECISION OR_U(MVSIZ,8),OR_COEF(MVSIZ,23)
      DOUBLE PRECISION OR_LV(MVSIZ,12),OR_LW(MVSIZ,12)
      DOUBLE PRECISION OR_D(MVSIZ,8),OR_WXY(MVSIZ),OR_GSTRW(MVSIZ)
      DOUBLE PRECISION OR_FOR(MVSIZ,5),OR_MOM(MVSIZ,3)
      DOUBLE PRECISION OR_EPSDOT(6,MVSIZ),OR_DEF(MVSIZ,8)
C     C++ validation runs before any native call. All COMMON symbols read by
C     this admitted branch are assigned here; full upstream layouts retained.
      OR_STATUS=1
      IF(OR_MODE/=1.AND.OR_MODE/=2) RETURN
      IF(OR_ITHK/=0.AND.OR_ITHK/=1) RETURN
      IF(KIND(ZERO)/=C_DOUBLE.OR.NIXC/=7) RETURN
      NPROPM=27
      NPROPG=38
      NPROPGI=98
      DT1=OR_CFG(1)
      IMPL_S=0
      ISHFRAM=0
      IEPSDOT=0
      OR_IXC(:,1)=(/1,1,2,3,4,1,1/)
      OR_MAT=0
      OR_PID=0
      OR_NGL=0
      OR_IGEO=0
      OR_GEO=ZERO
      OR_PM=ZERO
      OR_STACK=ZERO
      OR_OFFG=ONE
      OR_OFF=ZERO
      OR_SIGY=ZERO
      OR_DT=ZERO
      OR_WORLD=ZERO
      OR_VW=ZERO
      OR_FRAME=ZERO
      OR_SMSTR=ZERO
      OR_GSTR=ZERO
      OR_GEOM=ZERO
      OR_LX=ZERO
      OR_STI=ZERO
      OR_STIR=ZERO
      OR_U=ZERO
      OR_COEF=ZERO
      OR_LV=ZERO
      OR_LW=ZERO
      OR_D=ZERO
      OR_WXY=ZERO
      OR_GSTRW=ZERO
      OR_FOR=ZERO
      OR_MOM=ZERO
      OR_EPSDOT=ZERO
      OR_DEF=ZERO
      OR_OFFG(1)=OR_SIN(1)
      OR_SMSTR(1,:)=OR_SIN(2:7)
      OR_GSTR(1,:)=OR_SIN(8:15)
      OR_FRAME(1,:)=OR_FIN
      OR_COEF(1,23)=OR_CFG(2)
      OR_COEF(1,9)=OR_CFG(3)
C     Native PM slots are initialized from independently specified isotropic
C     material constants; these are synthetic elastic parameters, not LAW24.
      OR_PM(1,1)=OR_CFG(6)
      OR_PM(20,1)=OR_CFG(4)
      OR_PM(21,1)=OR_CFG(5)
      OR_PM(22,1)=OR_CFG(4)/(TWO*(ONE+OR_CFG(5)))
      OR_PM(24,1)=OR_CFG(4)/(ONE-OR_CFG(5)**2)
      OR_PM(25,1)=OR_PM(24,1)*OR_CFG(5)
      OR_PM(27,1)=OR_CFG(7)
      OR_GEO(13:15,1)=OR_CFG(9:11)
      OR_GEO(18:20,1)=OR_CFG(12:14)
      OR_GEO(37,1)=ZERO
      OR_GEO(38,1)=OR_CFG(8)
      CALL CCOOR3(1,1,OR_XIN,OR_IXC,OR_GEO,OR_OFFG,
     . OR_OFF,OR_SIGY,OR_PID,OR_VIN,OR_WIN,
     . OR_VW(1,1,1,1),OR_VW(1,1,2,1),OR_VW(1,1,3,1),OR_VW(1,1,4,1),
     . OR_VW(1,1,1,2),OR_VW(1,1,2,2),OR_VW(1,1,3,2),OR_VW(1,1,4,2),
     . OR_MAT,OR_DT,OR_COEF(1,23),OR_COEF(1,1),OR_NGL,
     . OR_WORLD(1,1),OR_WORLD(1,4),OR_WORLD(1,7),OR_WORLD(1,10),
     . OR_WORLD(1,2),OR_WORLD(1,5),OR_WORLD(1,8),OR_WORLD(1,11),
     . OR_WORLD(1,3),OR_WORLD(1,6),OR_WORLD(1,9),OR_WORLD(1,12))
      CALL CDERI3(1,1,OR_SMSTR,OR_OFFG,OR_STI,OR_STIR,
     . OR_GEOM(1,5),OR_GEOM(1,1),OR_GEOM(1,2),OR_GEOM(1,3),OR_GEOM(1,4),
     . OR_LX(1,1),OR_LX(1,2),OR_LX(1,3),OR_LX(1,4),OR_LX(1,5),OR_LX(1,6),OR_LX(1,7),
     . OR_WORLD(1,1),OR_WORLD(1,4),OR_WORLD(1,7),OR_WORLD(1,10),
     . OR_WORLD(1,2),OR_WORLD(1,5),OR_WORLD(1,8),OR_WORLD(1,11),
     . OR_WORLD(1,3),OR_WORLD(1,6),OR_WORLD(1,9),OR_WORLD(1,12),
     . OR_FRAME(1,1),OR_FRAME(1,2),OR_FRAME(1,3),OR_FRAME(1,4),OR_FRAME(1,5),OR_FRAME(1,6),
     . OR_FRAME(1,7),OR_FRAME(1,8),OR_FRAME(1,9),OR_GEOM(1,6),OR_GEOM(1,7),OR_GEOM(1,8),
     . OR_U(1,1),OR_U(1,2),OR_U(1,3),OR_U(1,4),OR_U(1,5),OR_U(1,6),OR_U(1,7),OR_U(1,8),
     . MVSIZ,OR_MODE)
      CALL CCOEF3(1,1,OR_PM,OR_MAT,OR_GEO,OR_PID,OR_OFF,
     . OR_GEOM(1,5),OR_STI,OR_STIR,OR_COEF(1,3),OR_COEF(1,1),OR_COEF(1,2),
     . OR_COEF(1,4),OR_COEF(1,5),OR_COEF(1,6),OR_COEF(1,7),OR_COEF(1,8),OR_COEF(1,9),
     . OR_COEF(1,10),OR_COEF(1,11),OR_COEF(1,12),OR_COEF(1,13),OR_COEF(1,14),
     . OR_COEF(1,15),OR_COEF(1,16),OR_COEF(1,17),OR_COEF(1,18),
     . 1,OR_ITHK,OR_MODE,3,0,OR_COEF(1,19),OR_COEF(1,20),OR_COEF(1,21),
     . OR_IGEO,OR_COEF(1,22),1,OR_STACK)
      CALL CDEFO3(1,1,
     . OR_VW(1,1,1,1),OR_VW(1,1,2,1),OR_VW(1,1,3,1),OR_VW(1,1,4,1),
     . OR_DT,OR_GEOM(1,1),OR_GEOM(1,2),OR_GEOM(1,3),OR_GEOM(1,4),OR_GEOM(1,5),
     . OR_D(1,1),OR_D(1,2),OR_D(1,3),OR_D(1,5),OR_D(1,4),
     . OR_LX(1,1),OR_LX(1,2),OR_LX(1,3),OR_LX(1,4),OR_LX(1,5),OR_LX(1,6),OR_LX(1,7),
     . OR_LV(1,1),OR_LV(1,4),OR_LV(1,7),OR_LV(1,10),
     . OR_LV(1,2),OR_LV(1,5),OR_LV(1,8),OR_LV(1,11),
     . OR_LV(1,3),OR_LV(1,6),OR_LV(1,9),OR_LV(1,12),
     . OR_FRAME(1,1),OR_FRAME(1,2),OR_FRAME(1,3),OR_FRAME(1,4),OR_FRAME(1,5),OR_FRAME(1,6),
     . OR_FRAME(1,7),OR_FRAME(1,8),OR_FRAME(1,9),1)
      CALL CCURV3(1,1,
     . OR_VW(1,1,1,2),OR_VW(1,1,2,2),OR_VW(1,1,3,2),OR_VW(1,1,4,2),
     . OR_GEOM(1,1),OR_GEOM(1,2),OR_GEOM(1,3),OR_GEOM(1,4),OR_GEOM(1,5),
     . OR_LW(1,1),OR_LW(1,4),OR_LW(1,7),OR_LW(1,10),
     . OR_LW(1,2),OR_LW(1,5),OR_LW(1,8),OR_LW(1,11),
     . OR_FRAME(1,1),OR_FRAME(1,2),OR_FRAME(1,3),OR_FRAME(1,4),OR_FRAME(1,5),OR_FRAME(1,6),
     . OR_FRAME(1,7),OR_FRAME(1,8),OR_FRAME(1,9),
     . OR_D(1,6),OR_D(1,7),OR_D(1,8),OR_D(1,5),OR_D(1,4),OR_WXY,OR_MODE)
      CALL CSTRA3(1,1,OR_GSTR,OR_COEF(1,3),OR_GEOM(1,5),
     . OR_D(1,1),OR_D(1,2),OR_D(1,3),OR_D(1,5),OR_D(1,4),
     . OR_D(1,6),OR_D(1,7),OR_D(1,8),OR_DT,OR_EPSDOT,OR_FOR,OR_MOM,
     . OR_MODE,1,1,0,1,
     . OR_U(1,1),OR_U(1,2),OR_U(1,3),OR_U(1,4),OR_U(1,5),OR_U(1,6),OR_U(1,7),OR_U(1,8),
     . OR_GEOM(1,1),OR_GEOM(1,2),OR_GEOM(1,3),OR_GEOM(1,4),
     . OR_WXY,OR_GSTRW,OR_DEF,MVSIZ)
C     Preserve native component ordering. The public output uses GSTR order,
C     so transverse yz precedes xz. CCURV3 only projects tangential rotations;
C     its unused normal projection below is diagnostic, not native arithmetic.
      DO OR_J=1,4
        OR_LW(1,3*OR_J)=DOT_PRODUCT(OR_WIN(:,OR_J),OR_FIN(7:9))
      ENDDO
      OR_OUT(1)=OR_OFFG(1)
      OR_OUT(2:7)=OR_SMSTR(1,:)
      OR_OUT(8:15)=OR_GSTR(1,:)
      OR_OUT(16:19)=OR_GEOM(1,1:4)
      OR_OUT(20:22)=OR_GEOM(1,5:7)
      OR_OUT(23:25)=OR_COEF(1,1:3)
      OR_OUT(26:27)=OR_COEF(1,15:16)
      OR_OUT(28:34)=OR_COEF(1,(/11,6,4,5,7,8,10/))
      OR_OUT(35:40)=OR_COEF(1,(/12,13,14,19,20,21/))
      OR_OUT(41:48)=OR_D(1,:)
      OR_OUT(49:60)=OR_LV(1,:)
      OR_OUT(61:72)=OR_LW(1,:)
      OR_OUT(73)=OR_OFF(1)
      OR_STATUS=2
      IF(.NOT.ALL(IEEE_IS_FINITE(OR_OUT))) RETURN
      OR_STATUS=0
      END
