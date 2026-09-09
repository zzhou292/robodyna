! SPDX-License-Identifier: AGPL-3.0-or-later
! OpenRadioss (C) 2026 Siemens; complete C3STRA3 and SIGEPS01G leaves.
! Selected C3FORC3:553-564 EPSD and MULAWGLC:156-174,196-238,518-583
! driver expressions only. No full all-material driver or QEPH wrapper.
      MODULE T3_ENGINE_LAW1_MOD
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
      USE, INTRINSIC :: IEEE_ARITHMETIC, ONLY: IEEE_IS_FINITE
      USE T3_ENGINE_GEOMETRY_MOD, ONLY: T3_GEOMETRY_
      USE T3_ENGINE_MATERIAL_MOD, ONLY: T3_MATERIAL_
      USE T3_ENGINE_HISTORY_MOD, ONLY: T3_HISTORY_
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      PRIVATE
      PUBLIC :: T3_UPDATE_LAW1
      CONTAINS
      SUBROUTINE T3_UPDATE_LAW1(G,M,H,STEP,VISCMX,STATUS)
#include "scr14_c.inc"
      TYPE(T3_GEOMETRY_),INTENT(INOUT) :: G
      TYPE(T3_MATERIAL_),INTENT(IN) :: M
      TYPE(T3_HISTORY_),INTENT(INOUT) :: H
      REAL(C_DOUBLE),INTENT(IN) :: STEP
      MY_REAL,INTENT(OUT) :: VISCMX(MVSIZ)
      INTEGER(C_INT),INTENT(OUT) :: STATUS
      INTEGER MAT(MVSIZ),PID(MVSIZ),I
      MY_REAL F_DEF(MVSIZ,8),EPSDOT(6,1),GSTRW(MVSIZ),THKN(MVSIZ),
     . SIGN(1,5),MOMN(1,3),DEGMB,DEGFX,VISC,DTINV,VOL2,EPS_K2,EPS_M2
      IEPSDOT=0
      MAT=1
      PID=1
      F_DEF=ZERO
      EPSDOT=ZERO
      GSTRW=17.25D0
      SIGN=ZERO
      MOMN=ZERO
      VISCMX=ZERO
! C3STRA3 takes ZX before YZ; public/native material packet stores YZ/ZX.
      CALL C3STRA3(1,1,M%PM,MAT,G%AREA,
     . G%DEF(1,1),G%DEF(1,2),G%DEF(1,3),G%DEF(1,5),G%DEF(1,4),
     . G%DEF(1,6),G%DEF(1,7),G%DEF(1,8),M%GEO,PID,M%NU,M%SHF,
     . H%STRA,M%SSP,M%RHO,EPSDOT,0,1,-1,
     . G%U(1,1,1),G%U(1,1,2),G%U(1,1,3),
     . G%U(1,2,1),G%U(1,2,2),G%U(1,2,3),
     . G%D(1,1),G%D(1,2),G%D(1,3),1,F_DEF,G%WXY,GSTRW,1)
      STATUS=2
      IF(GSTRW(1)/=17.25D0) RETURN
      DTINV=STEP/MAX(STEP**2,EM20)
      EPS_K2=(G%DEF(1,6)**2+G%DEF(1,7)**2+G%DEF(1,6)*G%DEF(1,7)
     .       +FOURTH*G%DEF(1,8)**2)*ONE_OVER_9*H%THK(1)**2
      EPS_M2=FOUR_OVER_3*(G%DEF(1,1)**2+G%DEF(1,2)**2
     .       +G%DEF(1,1)*G%DEF(1,2)+FOURTH*G%DEF(1,3)**2)
! ASRATE1 and the source old-value term retain the driver's overwrite order.
      H%EPSD=ONE*(SQRT(EPS_K2+EPS_M2)*DTINV)+(ONE-ONE)*H%EPSD
      DEGMB=H%FOR(1,1)*G%DEF(1,1)+H%FOR(1,2)*G%DEF(1,2)
     . +H%FOR(1,3)*G%DEF(1,3)+H%FOR(1,4)*G%DEF(1,4)+H%FOR(1,5)*G%DEF(1,5)
      DEGFX=H%MOM(1,1)*G%DEF(1,6)+H%MOM(1,2)*G%DEF(1,7)+H%MOM(1,3)*G%DEF(1,8)
      THKN=H%THK
      H%FOR=H%FOR_G
      CALL SIGEPS01G(1,1,M%G,THKN,G%OFF,M%GS,M%A11,M%A12,M%NU,M%THK0,
     . 1,F_DEF,-1,G%DEF(1,1),G%DEF(1,2),G%DEF(1,3),G%DEF(1,4),G%DEF(1,5),
     . G%DEF(1,6),G%DEF(1,7),G%DEF(1,8),
     . H%FOR(1,1),H%FOR(1,2),H%FOR(1,3),H%FOR(1,4),H%FOR(1,5),
     . H%MOM(1,1),H%MOM(1,2),H%MOM(1,3),
     . SIGN(1,1),SIGN(1,2),SIGN(1,3),SIGN(1,4),SIGN(1,5),
     . MOMN(1,1),MOMN(1,2),MOMN(1,3),ZERO)
      H%FOR_G=SIGN+ZERO
      H%MOM=MOMN
      H%FOR=H%FOR_G
      IF(.NOT.IEEE_IS_FINITE(THKN(1))) RETURN
      IF(.NOT.(THKN(1)>=EM30)) RETURN
      H%THK(1)=MAX(THKN(1),EM30)
      IF(M%DM>ZERO) THEN
        VISCMX(1)=MAX(VISCMX(1),M%DM)
        VISC=ONEP414*M%DM*M%RHO(1)*M%SSP(1)*SQRT(G%AREA(1))*DTINV
        H%FOR(1,1)=H%FOR(1,1)+VISC*(G%DEF(1,1)+HALF*G%DEF(1,2))
        H%FOR(1,2)=H%FOR(1,2)+VISC*(G%DEF(1,2)+HALF*G%DEF(1,1))
        H%FOR(1,3)=H%FOR(1,3)+VISC*G%DEF(1,3)*THIRD
      ENDIF
      DO I=1,5
        H%FOR(1,I)=H%FOR(1,I)*G%OFF(1)
      ENDDO
      DO I=1,3
        H%MOM(1,I)=H%MOM(1,I)*G%OFF(1)
      ENDDO
      DEGMB=DEGMB+H%FOR(1,1)*G%DEF(1,1)+H%FOR(1,2)*G%DEF(1,2)
     . +H%FOR(1,3)*G%DEF(1,3)+H%FOR(1,4)*G%DEF(1,4)+H%FOR(1,5)*G%DEF(1,5)
      DEGFX=DEGFX+H%MOM(1,1)*G%DEF(1,6)+H%MOM(1,2)*G%DEF(1,7)+H%MOM(1,3)*G%DEF(1,8)
      VOL2=HALF*M%THK0(1)*G%AREA(1)*G%OFF(1)
      H%EINT(1,1)=H%EINT(1,1)+DEGMB*VOL2
      H%EINT(1,2)=H%EINT(1,2)+DEGFX*M%THK0(1)*VOL2
      STATUS=0
      END SUBROUTINE
      END MODULE
