! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete native SDISTOR_INI; prepared33 comes from independent native LAW90 reader/update.
subroutine law90_control_parameters_native(prepared,sig,kin,out,flag) bind(C,name='law90_control_parameters_native')
  use iso_c_binding,only:c_double,c_int
  use HEPH_NATIVE_PACKETS
  use IC1_NATIVE_SDISTOR_INI_MOD,only:IC1_NATIVE_SDISTOR_INI
  implicit none
  real(c_double),intent(in)::prepared(33),sig(6),kin(3)
  real(c_double),intent(out)::out(5)
  integer(c_int),intent(out)::flag
  real(c_double)::pm(110,1),stress(1,6),rho(1),speed(MVSIZ),off(MVSIZ),offg(1)
  real(c_double)::vol(MVSIZ),sti(MVSIZ),ll(MVSIZ),fld(MVSIZ),mu,fqmax
  integer::imat,indices(MVSIZ),istab(MVSIZ)
  pm=zero;pm(21,1)=prepared(5);pm(22,1)=prepared(29);pm(32,1)=prepared(30)
  pm(100,1)=prepared(7);imat=1
#include "pm107.inc"
  stress(1,:)=sig;rho=kin(1);speed=zero;speed(1)=kin(2)
  vol=zero;vol(1)=kin(3);off=one;offg=one;indices=1;istab=0;sti=zero;ll=zero;fld=zero
  call IC1_NATIVE_SDISTOR_INI(1,sti,110,1,10,indices,istab,pm,stress,rho,speed,off,offg,ll,vol,fld,mu,fqmax)
  out=[sti(1),fld(1),ll(1),mu,fqmax];flag=istab(1)
end subroutine
