! Fixed CMAIN3/MULAWGLC selected branch, with complete CZSTRA3/SIGEPS01G leaves.
! Adapted from OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later.
! Retained originals: MULAWGLC:156-174,196-238,518-583 and CMAIN3 dispatch.
! No layers, thermal model, material rotation, yield or external material calls.
      MODULE QE_Q2_LAW1_MOD
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
      USE QE_Q1_GEOMETRY_MOD, ONLY: QE_GEOMETRY_
      USE QE_Q2_MATERIAL_MOD, ONLY: QE_MATERIAL_
      USE QE_Q2_HISTORY_MOD, ONLY: QE_HISTORY_
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      PRIVATE
      PUBLIC :: QE_UPDATE_LAW1
      CONTAINS
      SUBROUTINE QE_UPDATE_LAW1(G,M,H,STEP,VISCMX,ZCFAC,STATUS)
      TYPE(QE_GEOMETRY_),INTENT(IN) :: G
      TYPE(QE_MATERIAL_),INTENT(IN) :: M
      TYPE(QE_HISTORY_),INTENT(INOUT) :: H
      REAL(C_DOUBLE),INTENT(IN) :: STEP
      MY_REAL,INTENT(OUT) :: VISCMX(MVSIZ),ZCFAC(MVSIZ,2)
      INTEGER(C_INT),INTENT(OUT) :: STATUS
      MY_REAL DX(MVSIZ,8),F_DEF(MVSIZ,8),U(MVSIZ,2,4),
     . EPSDOT(6,1),WXY(MVSIZ),GSTRW(MVSIZ),THKN(MVSIZ),
     . SIGN(1,5),MOMN(1,3),DEGMB,DEGFX,VISC,DTINV,VOL2
      INTEGER I
      DX=ZERO
      F_DEF=ZERO
      U=ZERO
      EPSDOT=ZERO
      WXY=ZERO
      GSTRW=ZERO
      SIGN=ZERO
      MOMN=ZERO
      VISCMX=ZERO
! Full CZSTRA3 performs the Q1 XZ/YZ -> material YZ/ZX mapping and STRA update.
      CALL CZSTRA3(1,1,0,G%VDEF,H%STRA,
     . DX(1,1),DX(1,2),DX(1,3),DX(1,5),DX(1,4),
     . DX(1,6),DX(1,7),DX(1,8),M%DT1C,EPSDOT,0,1,
     . U(1,1,1),U(1,1,2),U(1,1,3),U(1,1,4),
     . U(1,2,1),U(1,2,2),U(1,2,3),U(1,2,4),
     . G%Y24,G%PX2,G%PY1,G%PY2,G%AREA,-1,1,WXY,F_DEF,GSTRW,1)
! Preserve prior total stress for work BEFORE the material-only replacement.
      DEGMB=H%FOR(1,1)*DX(1,1)+H%FOR(1,2)*DX(1,2)+H%FOR(1,3)*DX(1,3)
     .     +H%FOR(1,4)*DX(1,4)+H%FOR(1,5)*DX(1,5)
      DEGFX=H%MOM(1,1)*DX(1,6)+H%MOM(1,2)*DX(1,7)+H%MOM(1,3)*DX(1,8)
      THKN=H%THK
      H%FOR=H%FOR_G
      DTINV=STEP/MAX(STEP**2,EM20)
! IGTYP1 directly packs global-integration material components; no ROTO/UROTO.
      CALL SIGEPS01G(1,1,M%G,THKN,G%OFF,M%GS,M%A11,M%A12,M%NU,M%THK0,
     . 1,F_DEF,-1,DX(1,1),DX(1,2),DX(1,3),DX(1,4),DX(1,5),
     . DX(1,6),DX(1,7),DX(1,8),
     . H%FOR(1,1),H%FOR(1,2),H%FOR(1,3),H%FOR(1,4),H%FOR(1,5),
     . H%MOM(1,1),H%MOM(1,2),H%MOM(1,3),
     . SIGN(1,1),SIGN(1,2),SIGN(1,3),SIGN(1,4),SIGN(1,5),
     . MOMN(1,1),MOMN(1,2),MOMN(1,3),ZERO)
      H%FOR_G=SIGN+ZERO
      H%MOM=MOMN
      H%FOR=H%FOR_G
! The source MAX protects invalid negative/vanished reported thickness. This
! narrow reference rejects activation of that clamp rather than silently
! publishing a repaired finite state; the original expression remains below.
      STATUS=1
      IF(.NOT.(THKN(1)>=EM30)) RETURN
      H%THK(1)=MAX(THKN(1),EM30)
! ETSE remains zero in LAW1; preserve the source ZCFAC result used by CZFINTN1.
      ZCFAC=ZERO
      IF(M%DM>ZERO) THEN
        VISCMX(1)=MAX(VISCMX(1),M%DM)
        VISC=ONEP414*M%DM*M%RHO(1)*M%SSP(1)*SQRT(G%AREA(1))*DTINV
        H%FOR(1,1)=H%FOR(1,1)+VISC*(DX(1,1)+HALF*DX(1,2))
        H%FOR(1,2)=H%FOR(1,2)+VISC*(DX(1,2)+HALF*DX(1,1))
        H%FOR(1,3)=H%FOR(1,3)+VISC*DX(1,3)*THIRD
      ENDIF
      DO I=1,5
        H%FOR(1,I)=H%FOR(1,I)*G%OFF(1)
      END DO
      DO I=1,3
        H%MOM(1,I)=H%MOM(1,I)*G%OFF(1)
      END DO
      DEGMB=DEGMB+H%FOR(1,1)*DX(1,1)+H%FOR(1,2)*DX(1,2)
     . +H%FOR(1,3)*DX(1,3)+H%FOR(1,4)*DX(1,4)+H%FOR(1,5)*DX(1,5)
      DEGFX=DEGFX+H%MOM(1,1)*DX(1,6)+H%MOM(1,2)*DX(1,7)+H%MOM(1,3)*DX(1,8)
      VOL2=HALF*M%THK0(1)*G%AREA(1)*G%OFF(1)
      H%EINT(1,1)=H%EINT(1,1)+DEGMB*VOL2
      H%EINT(1,2)=H%EINT(1,2)+DEGFX*M%THK0(1)*VOL2
      STATUS=0
      END SUBROUTINE
      END MODULE
