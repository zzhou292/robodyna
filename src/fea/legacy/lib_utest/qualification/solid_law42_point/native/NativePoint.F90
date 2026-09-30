! SPDX-License-Identifier: AGPL-3.0-or-later
! ABI around complete SIGEPS42; unselected Prony/thermal/implicit branches stay dormant.
subroutine law42_native_point(parameters,strain,density,active,values) bind(C,name='law42_native_point')
  use iso_c_binding, only:c_double
  use LAW42_REF_CONSTANT_MOD
  implicit none
#include "mvsiz_p.inc"
  real(c_double),intent(in)::parameters(4),strain(6)
  real(c_double),value::density,active
  real(c_double),intent(out)::values(13)
  integer::iparam(3),ifunc(1),npf(2)
  real(c_double)::uparam(24),sig(1,6),uvar(1,3),off(1),offg(1),et(1),ssp(1),vis(1)
  real(c_double)::zero1(1),one1(1),rho(1),rho0(1),bulk,gs,mu0,nu
  call law42_native_globals()
  uparam=zero
  gs=parameters(1)*two
  nu=parameters(2)
#include "bulk.inc"
  uparam(1)=parameters(1)
  uparam(11)=two
  uparam(21)=bulk
  uparam(23)=parameters(4)
  uparam(24)=one
  iparam=[1,0,1]
  ifunc=0
  npf=0
  sig=zero
  uvar=zero
  off=active
  offg=one
  et=zero
  zero1=zero
  one1=one
  rho=density
  rho0=parameters(3)
  call LAW42_REF_SIGEPS42(1,24,3,1,ifunc,npf,zero1,zero,zero,uparam,rho0,rho, &
    one1,zero1,uvar,off,offg,ssp,zero1,zero1,zero1,zero1,zero1,zero1, &
    strain(1),strain(2),strain(3),strain(4),strain(5),strain(6), &
    sig(:,1),sig(:,2),sig(:,3),sig(:,4),sig(:,5),sig(:,6), &
    zero1,zero1,zero1,zero1,zero1,zero1,zero1,zero1,zero1,vis,10,et,0,zero1,0,3,iparam)
  values(1:6)=sig(1,:)
  values(7:9)=uvar(1,:)
  values(10)=ssp(1)
  values(11)=et(1)
  values(12)=vis(1)
  values(13)=bulk
end subroutine

subroutine law42_native_spectrum(strain,values) bind(C,name='law42_native_spectrum')
  use iso_c_binding, only:c_double
  implicit none
#include "mvsiz_p.inc"
  real(c_double),intent(in)::strain(6)
  real(c_double),intent(out)::values(12)
  real(c_double)::a(MVSIZ,6),e(MVSIZ,3),v(MVSIZ,3,3)
  a=0
  a(1,:)=strain
  call LAW42_REF_VALPVEC_V(a,e,v,1)
  values(1:3)=e(1,:)
  values(4:12)=reshape(v(1,:,:),[9])
end subroutine
