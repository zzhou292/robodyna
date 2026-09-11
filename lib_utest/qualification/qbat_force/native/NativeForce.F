! SPDX-License-Identifier: AGPL-3.0-or-later
! Selected CBAFORC3 stage coordinator. Physics resides in retained native
! geometry/strain/force/viscosity leaves and the shared complete material leaves.
      SUBROUTINE QBF_NATIVE_FORCE(MFUNC,NPTS,ISRATE,CURVE,BASIC,
     . LINEAR,CONTROLS,DM,DECLARED_DN,D1,DT1,TIME,X,V,VR,STATE,OUTPUT)
     . BIND(C,NAME="qbf_native_force")
      USE ISO_C_BINDING, ONLY:C_DOUBLE,C_INT
      USE MEMBRANE_LAW44_CALLER, ONLY: MembranePoint,MembraneForceWork
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "param_c.inc"
      INTEGER(C_INT),VALUE :: MFUNC,NPTS,ISRATE
      REAL(C_DOUBLE),INTENT(IN) :: CURVE(2,NPTS+1),BASIC(3),
     . LINEAR(2),CONTROLS(3),DM,DECLARED_DN,D1,DT1,TIME,
     . X(3,4),V(3,4),VR(3,4)
      REAL(C_DOUBLE),INTENT(INOUT) :: STATE(116)
      REAL(C_DOUBLE),INTENT(OUT) :: OUTPUT(266)
      TYPE QBF_GLOBAL
        MY_REAL THK(1),EPSD(1)
      END TYPE
      TYPE(QBF_GLOBAL) GBUF
      MY_REAL VQ(MVSIZ,3,3),VCORE(MVSIZ,3,4),JAC(MVSIZ,4),
     . BMS(MVSIZ,8,4),BM(MVSIZ,36),VXYZ(MVSIZ,3,4),
     . RXYZ(MVSIZ,2,4),VDEFS(MVSIZ,8,4),VDEF(MVSIZ,8),
     . AREA(MVSIZ),LL(MVSIZ),FACN(MVSIZ,2),GEOM(84),
     . VF(MVSIZ,3,4),VM(MVSIZ,2,4),FOR(1,5,4),MOM(1,3),
     . AGG(1,5),STRAIN(1,8),PGSTRAIN(1,8,4),POINTS(7,4),
     . FAILURES(3,4),SURFACES(4),OFF(1),OLD_OFF,
     . EINT(1,2),WPLA,EVIS(8,1),POINT_VALUES(13,4),
     . VOLG(1),VOL0(1),THK0(1),CDET(1),RHO(1),G(1),NU(1),
     . SSP(1),A1(1),SHF(1),AMU(1),DN,DT1C(1),
     . PM(27,1),GEO(200,1),THK02(1),YM(1),A11(1),A12(1),KRZ(1),KFAC,GS(1),
     . EXX(1),EYY(1),EXY(1),EXZ(1),EYZ(1),KXX(1),KYY(1),KXY(1),
     . EPSDOT(6,1),F_DEF(MVSIZ,8),ZERO_V(1),X13(1),X24(1),Y13(1),Y24(1),
     . EPSD_PG(1),EPSD_GLOB(1),EPS_K2,EPS_M2,DTINV,ASRATE,
     . DX(8),BEFORE,MATERIAL_THK,POINT_THK(4,4),
     . VQN(MVSIZ,9,4),COREL(MVSIZ,3,4),DI(MVSIZ,6),VMZ(MVSIZ,4),
     . F(MVSIZ,3,4),M(MVSIZ,3,4),ALDT(1),VISCMX(1),
     . ALPE(1),F_DTE(1),F_OSET(1),STI(1),STIR(1),DTEL(1),DIVM
      INTEGER I,J,K,NG,IR,IS,JFT,JLT,NEL,NPG,IPLAT(MVSIZ),
     . IPARTC(1),FLAGS(4),INDEX,MAT(1),PID(1),MX,IPID,NPT,MTN,IMPL_S,IHBE
      JFT=1
      JLT=1
      NEL=1
      NPG=4
! Match the existing independent shell wrappers: storage is fixed before
! entry; native leaves receive their common part-observation stride here.
      NPSAV=8
      IPLAT=1
      IPARTC=1
      VF=ZERO;VM=ZERO;MOM=ZERO;BM=ZERO
      VQN=ZERO;COREL=ZERO;DI=ZERO;VMZ=ZERO
      EPSDOT=ZERO;F_DEF=ZERO;ZERO_V=ZERO
      EVIS=ZERO
      OUTPUT=ZERO
      DO NG=1,4
        INDEX=24*(NG-1)
        POINTS(:,NG)=STATE(INDEX+1:INDEX+7)
        FAILURES(:,NG)=STATE(INDEX+8:INDEX+10)
        SURFACES(NG)=STATE(INDEX+11)
        FOR(1,:,NG)=STATE(INDEX+12:INDEX+16)
        PGSTRAIN(1,:,NG)=STATE(INDEX+17:INDEX+24)
      ENDDO
      AGG(1,:)=STATE(97:101)
      STRAIN(1,:)=STATE(102:109)
      GBUF%THK=STATE(110)
      EINT(1,:)=STATE(111:112)
      WPLA=STATE(113)
      EVIS(8,1)=STATE(114)
      GBUF%EPSD=STATE(115)
      OFF=STATE(116)
      CALL QBF_KINEMATICS(X,V,VR,DT1,VQ,VCORE,JAC,BMS,VXYZ,
     . RXYZ,VDEFS,AREA,LL,FACN,GEOM)
! Material preparation owns PM; the original CNCOEF3 selects its actual GS.
      PM=ZERO
      PM(1,1)=BASIC(1)
      PM(20,1)=BASIC(2)
      PM(21,1)=BASIC(3)
      PM(22,1)=BASIC(2)/TWO/(ONE+BASIC(3))
      PM(24,1)=BASIC(2)/(ONE-BASIC(3)*BASIC(3))
      PM(25,1)=PM(24,1)*BASIC(3)
      PM(27,1)=SQRT(PM(24,1)/BASIC(1))
      GEO=ZERO
      GEO(13,1)=DECLARED_DN
      MAT=1;PID=1;NPT=1;MTN=44;IMPL_S=0;IHBE=11;KFAC=ZERO
      THK0=GBUF%THK
#include "extracted/CurrentMaterialCoefficients.inc"
#include "extracted/CurrentShearCoefficients.inc"
#include "extracted/CurrentDn.inc"
      A1=A11
      X13=VCORE(1,1,2);X24=-VCORE(1,3,1)
      Y13=-VCORE(1,2,1);Y24=VCORE(1,1,1)
      VDEF=VDEFS(:,:,1)
      CALL QBF_CBAENERS(JFT,JLT,OFF,AREA,THK0,VDEF,AGG,EINT,DT1,NEL)
      EPSD_GLOB=ZERO
      DO IS=1,2
       DO IR=1,2
        NG=2*(IS-1)+IR
        CDET=JAC(1,NG)
        VOL0=THK0*CDET
        BEFORE=THK0(1)
        VDEF=VDEFS(:,:,NG)
        BM(:,1:8)=BMS(:,:,NG)
        CALL QBF_CBASTRA3(STRAIN,PGSTRAIN(:,:,NG),JFT,JLT,0,NPG,VDEF,
     .    EXX,EYY,EXY,EXZ,EYZ,KXX,KYY,KXY,DT1C,EPSDOT,0,1,
     .    ZERO_V,ZERO_V,ZERO_V,ZERO_V,ZERO_V,ZERO_V,ZERO_V,ZERO_V,
     .    X13,X24,Y13,Y24,BM,2,44,1,IPLAT,0,ZERO_V,F_DEF,ZERO_V,NEL)
        CALL QBF_CBAENER(FOR(:,:,NG),EINT,JFT,JLT,OFF,VOL0,EXY,NEL)
#include "extracted/Rate.inc"
        DX=[EXX(1),EYY(1),EXY(1),EYZ(1),EXZ(1),KXX(1),KYY(1),KXY(1)]
        OLD_OFF=OFF(1)
        CALL MembranePoint(MFUNC,NPTS,CURVE,BASIC,LINEAR,CONTROLS,
     .    ISRATE,D1,DT1,TIME,DX(1:5),EPSD_PG(1),GS(1),THK0(1),CDET,
     .    OFF(1),POINTS(:,NG),FAILURES(:,NG),POINT_VALUES(:,NG),WPLA)
        FLAGS=INT(FAILURES(3,:))
        CALL QBF_PARENT(IR,IS,FLAGS,SURFACES,OFF)
        CALL MembraneForceWork(DX,THK0,CDET,DM,DT1,SSP,RHO,OLD_OFF,
     .    OFF(1),POINT_VALUES(:,NG),FOR(:,:,NG),MOM,GBUF%THK,EINT)
        CALL QBF_CBAENER(FOR(:,:,NG),EINT,JFT,JLT,OFF,VOL0,EXY,NEL)
        MATERIAL_THK=GBUF%THK(1)
        I=1
#include "extracted/QuarterThickness.inc"
        POINT_THK(:,NG)=[BEFORE,MATERIAL_THK,THK0(1),VOL0(1)]
        CALL QBF_CBAVISC(JFT,JLT,VDEF,AMU,OFF,SHF,NU,RHO,SSP,CDET,
     .    GBUF%THK,FOR(:,:,NG),MOM,1,44,IPARTC,EVIS,DT1,NEL)
        CALL QBF_CBAFORI1(JFT,JLT,FOR(:,:,NG),BM,VF,1,IPLAT,VOL0,NEL)
        OUTPUT(126+(NG-1)*8:133+(NG-1)*8)=VDEF(1,:)
        OUTPUT(158+(NG-1)*8:165+(NG-1)*8)=DX
       ENDDO
      ENDDO
#include "extracted/RatePublish.inc"
      DO J=1,5
        AGG(1,J)=FOURTH*(FOR(1,J,1)+FOR(1,J,2)+FOR(1,J,3)+FOR(1,J,4))
      ENDDO
      CALL QBF_CBAFORCT(JFT,JLT,VOLG,X13,X24,Y13,Y24,AGG,VF,1,IPLAT,OFF,NEL)
      CALL QBF_CBAENERS(JFT,JLT,OFF,AREA,THK0,VDEF,AGG,EINT,DT1,NEL)
      CALL QBF_CBAVISNP1(JFT,JLT,VXYZ,RXYZ,VCORE,AMU,OFF,RHO,SSP,
     . AREA,THK0,G,DT1,VF,IPARTC,EVIS,0)
      CALL QBF_CBAPROJ(JFT,JLT,VQN,VQ,VF,VM,1,IPLAT,
     . F(:,1,1),F(:,1,2),F(:,1,3),F(:,1,4),
     . F(:,2,1),F(:,2,2),F(:,2,3),F(:,2,4),
     . F(:,3,1),F(:,3,2),F(:,3,3),F(:,3,4),
     . M(:,1,1),M(:,1,2),M(:,1,3),M(:,1,4),
     . M(:,2,1),M(:,2,2),M(:,2,3),M(:,2,4),
     . M(:,3,1),M(:,3,2),M(:,3,3),M(:,3,4),COREL,DI,VMZ,0,OFF)
      ALDT=LL(1);VISCMX=DM;ALPE=ONE;F_DTE=ONE;F_OSET=ONE
#include "extracted/CurrentViscosity.inc"
#include "extracted/CurrentTimestep.inc"
      VOL0=VOLG
#include "extracted/CurrentStiffness.inc"
      DO NG=1,4
        INDEX=24*(NG-1)
        STATE(INDEX+1:INDEX+7)=POINTS(:,NG)
        STATE(INDEX+8:INDEX+10)=FAILURES(:,NG)
        STATE(INDEX+11)=SURFACES(NG)
        STATE(INDEX+12:INDEX+16)=FOR(1,:,NG)
        STATE(INDEX+17:INDEX+24)=PGSTRAIN(1,:,NG)
        OUTPUT(1+(NG-1)*3:3+(NG-1)*3)=F(1,:,NG)
        OUTPUT(13+(NG-1)*3:15+(NG-1)*3)=M(1,:,NG)
        OUTPUT(190+(NG-1)*13:202+(NG-1)*13)=POINT_VALUES(:,NG)
        OUTPUT(242+(NG-1)*4:245+(NG-1)*4)=POINT_THK(:,NG)
      ENDDO
      STATE(97:101)=AGG(1,:)
      STATE(102:109)=STRAIN(1,:)
      STATE(110)=GBUF%THK(1)
      STATE(111:112)=EINT(1,:)
      STATE(113)=WPLA
      STATE(114)=EVIS(8,1)
      STATE(115)=GBUF%EPSD(1)
      STATE(116)=OFF(1)
      OUTPUT(25:108)=GEOM
      OUTPUT(109:117)=RESHAPE(VXYZ(1,:,1:3),[9])
      OUTPUT(118:125)=RESHAPE(RXYZ(1,:,:),[8])
      OUTPUT(258:260)=[LL(1),FACN(1,1),FACN(1,2)]
      OUTPUT(261:266)=[DN,SSP(1),VISCMX(1),DTEL(1),STI(1),STIR(1)]
      END
