! Four private test nodes; complete CUPDTN3 verifies native internal/RHS sign.
! OpenRadioss (C) 2026 Siemens, AGPL-3.0-or-later. Not a production assembler.
      SUBROUTINE QE_Q2_SCATTER(INTERNAL,COEFFICIENTS,CONNECTIVITY,OUTPUT)
     . BIND(C,NAME="qeph_q2_scatter")
      USE ISO_C_BINDING, ONLY: C_DOUBLE,C_INT
      USE ELEMENT_MOD, ONLY: NIXC
#include "implicit_f.inc"
#include "mvsiz_p.inc"
#include "param_c.inc"
#include "scr18_c.inc"
#include "scr02_c.inc"
      REAL(C_DOUBLE),INTENT(IN) :: INTERNAL(24),COEFFICIENTS(4)
      INTEGER(C_INT),INTENT(IN) :: CONNECTIVITY(4)
      REAL(C_DOUBLE),INTENT(OUT) :: OUTPUT(32)
      MY_REAL FI(MVSIZ,3,4),MI(MVSIZ,3,4),F(3,4),M(3,4),OFFG(MVSIZ),OFF(MVSIZ),
     . STI(MVSIZ),STIR(MVSIZ),STIFN(4),STIFR(4),PM(200,1),AREA(MVSIZ),THK(MVSIZ),
     . EINT(1,2),PARTSAV(8,1),FAC(MVSIZ,2),THEM(MVSIZ,4),FTHE(4),CONDN(4),CONDE(MVSIZ)
      INTEGER IXC(NIXC,MVSIZ),MAT(MVSIZ),IPARTC(MVSIZ),N
      NPROPM=200
      NPSAV=8
      NODADT=1
      FI=ZERO
      MI=ZERO
      F=ZERO
      M=ZERO
      OFFG=ONE
      OFF=ONE
      STI=COEFFICIENTS(1)
      STIR=COEFFICIENTS(2)
      STIFN=ZERO
      STIFR=ZERO
      PM=ZERO
      AREA=ONE
      THK=ONE
      EINT=ZERO
      PARTSAV=ZERO
      FAC(:,1)=COEFFICIENTS(3)
      FAC(:,2)=COEFFICIENTS(4)
      THEM=ZERO
      FTHE=ZERO
      CONDN=ZERO
      CONDE=ZERO
      IXC=1
      MAT=1
      IPARTC=1
      DO N=1,4
        FI(1,:,N)=INTERNAL(3*N-2:3*N)
        MI(1,:,N)=INTERNAL(3*N+10:3*N+12)
        IXC(N+1,1)=CONNECTIVITY(N)
      ENDDO
      CALL CUPDTN3(1,1,F,M,0,OFFG,OFF,STI,STIR,STIFN,STIFR,IXC,PM,AREA,THK,
     . FI(1,1,1),FI(1,1,2),FI(1,1,3),FI(1,1,4),
     . FI(1,2,1),FI(1,2,2),FI(1,2,3),FI(1,2,4),
     . FI(1,3,1),FI(1,3,2),FI(1,3,3),FI(1,3,4),
     . MI(1,1,1),MI(1,1,2),MI(1,1,3),MI(1,1,4),
     . MI(1,2,1),MI(1,2,2),MI(1,2,3),MI(1,2,4),
     . MI(1,3,1),MI(1,3,2),MI(1,3,3),MI(1,3,4),
     . EINT,PARTSAV,MAT,IPARTC,FAC,0,THEM,FTHE,CONDN,CONDE,0)
      DO N=1,4
        OUTPUT(3*N-2:3*N)=F(:,N)
        OUTPUT(3*N+10:3*N+12)=M(:,N)
      ENDDO
      OUTPUT(25:28)=STIFN
      OUTPUT(29:32)=STIFR
      END
