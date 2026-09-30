! SPDX-License-Identifier: AGPL-3.0-or-later
! Context around exact authenticated DTNODA positive-coefficient vector kernels.
subroutine cin_native_nodal(count,MS,IN,STIFN,STIFR,factor,values) bind(C)
  use iso_c_binding
  implicit none
  integer(c_int), value :: count
  real(c_double), intent(in) :: MS(count),IN(count),STIFN(count),STIFR(count),factor
  real(c_double), intent(out) :: values(2*count)
  integer :: K,N,KMAX,INDTN(count)
  real(c_double) :: DTFAC1(11),DTN(count),DTNOD
  real(c_double), parameter :: TWO=2d0
  DTFAC1=factor
  KMAX=count
  do K=1,count
    INDTN(K)=K
  end do
  DTNOD=huge(1d0)
@TRANSLATION@
  values(1:count)=DTN
  DTNOD=huge(1d0)
@ROTATION@
  values(count+1:2*count)=DTN
end subroutine

! Per-member native stiffness producer; no RBYM scalar timestep repair.
subroutine cin_native_rigid_terms(count,positions,center,STIFN,STIFR,values) bind(C)
  use iso_c_binding
  implicit none
  integer(c_int), value :: count
  real(c_double), intent(in) :: positions(3,count),center(3),STIFN(count),STIFR(count)
  real(c_double), intent(out) :: values(2*count)
  real(c_double) :: X(3,count+1),F1(count),F2(count),DD
  integer :: I,N,M
  X(:,1:count)=positions
  M=count+1
  X(:,M)=center
  do I=1,count
    N=I
@RIGID_TERMS@
  end do
  values(1:count)=F1
  values(count+1:2*count)=F2
end subroutine
