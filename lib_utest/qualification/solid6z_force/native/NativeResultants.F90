! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete native resultants, corrected physical stabilization, and rotation.
subroutine SOLID6Z_FORCE_RESULTANTS(G,PARAMETERS,REFERENCE_VOLUME,BASE,POINT,DT,DN, &
    SSP_SCALE,HISTORY,FORCES,STABILIZATION)
  use SOLID6Z_FORCE_PACKETS
  use SOLID6Z_FORCE_ELBUFDEF_MOD
  use SOLID6Z_FORCE_OBSERVATIONS
  use SOLID6Z_FORCE_S6ZFINT3_MOD
  use SOLID6Z_FORCE_S6ZHOUR3_MOD
  use SOLID6Z_FORCE_S6ZRROTA3_MOD
  implicit none
  type(native_geometry),intent(in) :: G
  real(kind=8),intent(in) :: PARAMETERS(4),REFERENCE_VOLUME,BASE(21),POINT(33),DT,DN,SSP_SCALE
  real(kind=8),intent(out) :: HISTORY(21),FORCES(54),STABILIZATION(28)
  real(kind=8) :: PM(250,1),GEO(1000,1),SIG(1,6),SIGOLD(1,6),SIGMEAN(1,6)
  real(kind=8) :: FHOUR(1,3,4),F(MVSIZ,6,3),OFF(MVSIZ),RHO(MVSIZ),Q(MVSIZ)
  real(kind=8) :: NU(MVSIZ),SSP(MVSIZ),ET(MVSIZ),EINT(MVSIZ),VOL0(MVSIZ)
  real(kind=8) :: SVIS(MVSIZ,6),AUX(MVSIZ,8),G0
  integer :: MAT(MVSIZ),NGEO(MVSIZ),K,N,M
  type(ELBUF_STRUCT_) :: ELBUF
  interface
    subroutine law42_solid_initial_modulus_native(P,G) bind(C,name='law42_solid_initial_modulus_native')
      use iso_c_binding
      real(c_double),intent(in) :: P(4)
      real(c_double),intent(out) :: G
    end subroutine
  end interface
  PM=0
  GEO=0
  SVIS=0
  SIGMEAN=0
  AUX=0
  F=0
  OFF=1
  RHO=POINT(7)
  Q=POINT(9)
  NU=PARAMETERS(2)
  SSP=POINT(20)*SSP_SCALE
  ET=POINT(21)
  EINT=POINT(8)
  VOL0=REFERENCE_VOLUME
  SIG(1,:)=POINT(1:6)
  SIGOLD(1,:)=BASE(1:6)
  MAT=1
  NGEO=1
  call law42_solid_initial_modulus_native(PARAMETERS,G0)
  PM(1,1)=PARAMETERS(3)
  PM(21,1)=PARAMETERS(2)
  PM(22,1)=G0
  GEO(13,1)=DN
  do K=1,3
    do M=1,4
      FHOUR(1,K,M)=BASE(9+4*(K-1)+M)
    end do
  end do
  call SOLID6Z_FORCE_S6ZFINT3( &
    SIG, G%P(:,1,1), G%P(:,2,1), G%P(:,3,1), &
    G%P(:,4,1), G%P(:,5,1), G%P(:,6,1), G%P(:,1,2), &
    G%P(:,2,2), G%P(:,3,2), G%P(:,4,2), G%P(:,5,2), &
    G%P(:,6,2), G%P(:,1,3), G%P(:,2,3), G%P(:,3,3), &
    G%P(:,4,3), G%P(:,5,3), G%P(:,6,3), F(:,1,1), &
    F(:,1,2), F(:,1,3), F(:,2,1), F(:,2,2), &
    F(:,2,3), F(:,3,1), F(:,3,2), F(:,3,3), &
    F(:,4,1), F(:,4,2), F(:,4,3), F(:,5,1), &
    F(:,5,2), F(:,5,3), F(:,6,1), F(:,6,2), &
    F(:,6,3), G%VOLUME, Q, EINT, &
    RHO, Q, AUX(:,1), AUX(:,2), &
    AUX(:,3), SIGMEAN, AUX(:,4), AUX(:,5), &
    AUX(:,6), AUX(:,7), G%VOLUME, OFF, &
    VOL0, VOL0, 0, 0, &
    1, SVIS, 1)
  do N=1,6
    do K=1,3
      FORCES(3*(N-1)+K)=F(1,N,K)
    end do
  end do
  call SOLID6Z_FORCE_S6ZHOUR3( &
    250, 1, PM, RHO, &
    G%VOLUME, SSP, G%X(:,1,1), G%X(:,2,1), &
    G%X(:,3,1), G%X(:,4,1), G%X(:,5,1), G%X(:,6,1), &
    G%X(:,1,2), G%X(:,2,2), G%X(:,3,2), G%X(:,4,2), &
    G%X(:,5,2), G%X(:,6,2), G%X(:,1,3), G%X(:,2,3), &
    G%X(:,3,3), G%X(:,4,3), G%X(:,5,3), G%X(:,6,3), &
    G%V(:,1,1), G%V(:,2,1), G%V(:,3,1), G%V(:,4,1), &
    G%V(:,5,1), G%V(:,6,1), G%V(:,1,2), G%V(:,2,2), &
    G%V(:,3,2), G%V(:,4,2), G%V(:,5,2), G%V(:,6,2), &
    G%V(:,1,3), G%V(:,2,3), G%V(:,3,3), G%V(:,4,3), &
    G%V(:,5,3), G%V(:,6,3), F(:,1,1), F(:,2,1), &
    F(:,3,1), F(:,4,1), F(:,5,1), F(:,6,1), &
    F(:,1,2), F(:,2,2), F(:,3,2), F(:,4,2), &
    F(:,5,2), F(:,6,2), F(:,1,3), F(:,2,3), &
    F(:,3,3), F(:,4,3), F(:,5,3), F(:,6,3), &
    NU, FHOUR, OFF, VOL0, &
    EINT, 1, MAT, 1000, &
    1, GEO, NGEO, DT, &
    ELBUF, 2, 1, 42, &
    AUX(:,1), SIG, SIGOLD, AUX(:,2), &
    10, 1, ET, G%RATE(:,1), &
    G%RATE(:,2), G%RATE(:,3), G%RATE(:,4), G%RATE(:,5), &
    G%RATE(:,6), 35, 1, AUX(:,3))
  HISTORY(1:9)=POINT(1:9)
  HISTORY(8)=EINT(1)
  do K=1,3
    do M=1,4
      HISTORY(9+4*(K-1)+M)=FHOUR(1,K,M)
      STABILIZATION(4*(K-1)+M)=observer_rate(K,M)
      STABILIZATION(12+4*(K-1)+M)=observer_mode(K,M)
    end do
  end do
  STABILIZATION(25:28)=[observer_shear,observer_damping,observer_first_energy,observer_final_energy]
  do N=1,6
    do K=1,3
      FORCES(18+3*(N-1)+K)=F(1,N,K)
    end do
  end do
  call SOLID6Z_FORCE_S6ZRROTA3( &
    G%FRAME(:,1), G%FRAME(:,4), G%FRAME(:,7), G%FRAME(:,2), &
    G%FRAME(:,5), G%FRAME(:,8), G%FRAME(:,3), G%FRAME(:,6), &
    G%FRAME(:,9), F(:,1,1), F(:,2,1), F(:,3,1), &
    F(:,4,1), F(:,5,1), F(:,6,1), F(:,1,2), &
    F(:,2,2), F(:,3,2), F(:,4,2), F(:,5,2), &
    F(:,6,2), F(:,1,3), F(:,2,3), F(:,3,3), &
    F(:,4,3), F(:,5,3), F(:,6,3), 1)
  do N=1,6
    do K=1,3
      FORCES(36+3*(N-1)+K)=F(1,N,K)
    end do
  end do
end subroutine
