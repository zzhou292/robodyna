! Explicit fixed LAW1/property inputs plus complete CNCOEF3B execution.
! Adapted startup packing: OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later.
! Exact sources: hm_read_mat01.F:116-140, hm_read_mat.F90:1612-1618,
! hm_read_prop01.F:196-245; see original/ and source-manifest.json.
      MODULE QE_Q2_MATERIAL_MOD
      USE ISO_C_BINDING, ONLY: C_DOUBLE
#include "implicit_f.inc"
#include "mvsiz_p.inc"
      PRIVATE
      PUBLIC :: QE_MATERIAL_,QE_PREPARE_MATERIAL
      TYPE QE_MATERIAL_
        MY_REAL THK0(MVSIZ),THK02(MVSIZ),NU(MVSIZ),G(MVSIZ),YM(MVSIZ),
     . A11(MVSIZ),A12(MVSIZ),SSP(MVSIZ),RHO(MVSIZ),VOL0(MVSIZ),GS(MVSIZ),
     . DT1C(MVSIZ),AMU(MVSIZ),GSR(MVSIZ),A11SR(MVSIZ),A12SR(MVSIZ),
     . NUSR(MVSIZ),SHF(MVSIZ),SHFSR(MVSIZ),KRZ(MVSIZ),A11R(MVSIZ),
     . ZOFFSET(MVSIZ),FAC58(MVSIZ,2),PM(200,1),DM
      END TYPE
      CONTAINS
      SUBROUTINE QE_PREPARE_MATERIAL(INPUT,AREA,THK,STEP,M)
#include "param_c.inc"
      REAL(C_DOUBLE),INTENT(IN) :: INPUT(4),AREA(MVSIZ),THK(MVSIZ),STEP
      TYPE(QE_MATERIAL_),INTENT(OUT) :: M
      INTEGER MAT(MVSIZ),PID(MVSIZ),IGEO(100,1)
      MY_REAL GEO(200,1),THKE(MVSIZ),PM_STACK(20,1),UPARAM(14),
     . DIRA(MVSIZ,2),DIRB(MVSIZ,2),UVAR(MVSIZ,11),THK_COPY(MVSIZ)
! Indexed table extents match the real dummy declarations. Unselected table
! entries are zero, with explicit resolved centered/isotropic branch values.
      NPROPM=200
      NPROPG=200
      NPROPGI=100
      NPSAV=8
      MAT=1
      PID=1
      IGEO=0
      IGEO(11,1)=1
      GEO=ZERO
      GEO(13,1)=ZEP01+FIVEEM3
      GEO(17,1)=ONE
      GEO(38,1)=FIVE_OVER_6
      GEO(100,1)=SQRT(GEO(38,1))
      M%PM=ZERO
      M%PM(1,1)=INPUT(1)
      M%PM(20,1)=INPUT(2)
      M%PM(21,1)=INPUT(3)
      M%PM(22,1)=INPUT(2)/(TWO*(ONE+INPUT(3)))
      M%PM(24,1)=INPUT(2)/(ONE-INPUT(3)**2)
      M%PM(25,1)=INPUT(3)*M%PM(24,1)
      M%PM(26,1)=FIVE_OVER_6
      M%PM(27,1)=SQRT(INPUT(2)/MAX(INPUT(1),EM20))
      M%PM(12,1)=SQRT(MAX(ZERO,M%PM(22,1)))
      M%PM(13,1)=SQRT(MAX(ZERO,M%PM(24,1)))
      M%PM(14,1)=SQRT(MAX(ZERO,M%PM(25,1)))
      M%PM(190,1)=SQRT(MAX(ZERO,M%PM(21,1)))
      M%DM=ZEP01+FIVEEM3
      THKE=INPUT(4)
      THK_COPY=THK
      PM_STACK=ZERO
      UPARAM=ZERO
      DIRA=ZERO
      DIRB=ZERO
      UVAR=ZERO
      M%THK0=ZERO
      M%THK02=ZERO
      M%NU=ZERO
      M%G=ZERO
      M%YM=ZERO
      M%A11=ZERO
      M%A12=ZERO
      M%SSP=ZERO
      M%RHO=ZERO
      M%VOL0=ZERO
      M%GS=ZERO
      M%DT1C=ZERO
      M%AMU=ZERO
      M%GSR=ZERO
      M%A11SR=ZERO
      M%A12SR=ZERO
      M%NUSR=ZERO
      M%SHF=ZERO
      M%SHFSR=ZERO
      M%KRZ=ZERO
      M%A11R=ZERO
      M%ZOFFSET=ZERO
      M%FAC58=ZERO
      CALL CNCOEF3B(1,1,M%PM,MAT,GEO,PID,AREA,M%SHF,M%THK0,
     . M%THK02,M%NU,M%G,M%YM,M%A11,M%A12,THK_COPY,THKE,M%SSP,
     . M%RHO,M%VOL0,M%GS,1,0,0,M%DT1C,STEP,23,M%AMU,
     . M%GSR,M%A11SR,M%A12SR,M%NUSR,M%SHFSR,M%KRZ,IGEO,
     . M%A11R,1,PM_STACK,UPARAM,DIRA,DIRB,UVAR,M%FAC58,1,M%ZOFFSET)
      END SUBROUTINE
      END MODULE
