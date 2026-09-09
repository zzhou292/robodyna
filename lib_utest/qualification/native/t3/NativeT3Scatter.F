! SPDX-License-Identifier: AGPL-3.0-or-later
! Test-only complete C3UPDT3 native signs and shared physical-node accounting.
! OpenRadioss (C) 2026 Siemens; no standalone force or state owner.
      SUBROUTINE T3_R3_SCATTER(NODES,FORCE,COUPLE,STIFFNESS,F,M,STIFN,STIFR)
     . BIND(C,NAME="t3_r3_scatter")
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
      USE ELEMENT_MOD, ONLY: NIXTG
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "param_c.inc"
      INTEGER(C_INT),INTENT(IN) :: NODES(3)
      REAL(C_DOUBLE),INTENT(IN) :: FORCE(3,3),COUPLE(3,3),STIFFNESS(2)
      REAL(C_DOUBLE),INTENT(INOUT) :: F(3,4),M(3,4),STIFN(4),STIFR(4)
      INTEGER IXTG(NIXTG,1),MAT(MVSIZ),IPARTTG(1),J
      MY_REAL OFFG(MVSIZ),OFF(MVSIZ),STI(MVSIZ),STIR(MVSIZ),
     . EF(MVSIZ,3,3),EM(MVSIZ,3,3),THEM(MVSIZ,3),FTHE(4),EINT(1,2),
     . PM(200,1),AREA(1),THK(1),PARTSAV(8,1),CONDN(4),CONDE(1)
      NPROPM=200
      NPSAV=8
      IXTG=1
      IXTG(2:4,1)=NODES
      MAT=1
      IPARTTG=1
      OFFG=ONE
      OFF=ONE
      STI=ZERO
      STIR=ZERO
      STI(1)=STIFFNESS(1)
      STIR(1)=STIFFNESS(2)
      EF=ZERO
      EM=ZERO
      DO J=1,3
        EF(1,:,J)=FORCE(:,J)
        EM(1,:,J)=COUPLE(:,J)
      ENDDO
      THEM=ZERO
      FTHE=ZERO
      EINT=ZERO
      PM=ZERO
      AREA=ONE
      THK=ONE
      PARTSAV=ZERO
      CONDN=ZERO
      CONDE=ZERO
      CALL C3UPDT3(1,1,F,M,0,OFFG,OFF,STI,STIR,STIFN,STIFR,IXTG,0,
     . EF(1,1,1),EF(1,1,2),EF(1,1,3),
     . EF(1,2,1),EF(1,2,2),EF(1,2,3),EF(1,3,1),EF(1,3,2),EF(1,3,3),
     . EM(1,1,1),EM(1,1,2),EM(1,1,3),
     . EM(1,2,1),EM(1,2,2),EM(1,2,3),EM(1,3,1),EM(1,3,2),EM(1,3,3),
     . 0,THEM,FTHE,EINT,PM,AREA,THK,PARTSAV,MAT,IPARTTG,CONDN,CONDE)
      END
