! SPDX-License-Identifier: AGPL-3.0-or-later
! Independent source leaf probe; artificial stress is not a material response.
subroutine ic1_native_parameter_probe(parameters,sig,kin,out,flag) bind(C,name='ic1_native_parameter_probe')
  use iso_c_binding,only:c_double,c_int
  use HEPH_NATIVE_PACKETS
  use IC1_NATIVE_SDISTOR_INI_MOD,only:IC1_NATIVE_SDISTOR_INI
  implicit none
  real(c_double),intent(in)::parameters(4),sig(6),kin(3)
  real(c_double),intent(out)::out(4)
  integer(c_int),intent(out)::flag
  interface
    subroutine slots_native(p,s) bind(C,name='ic1_native_slots')
      import c_double
      real(c_double),intent(in)::p(4)
      real(c_double),intent(out)::s(6)
    end subroutine
  end interface
  real(c_double)::pm(110,1),slots(6),stress(1,6),rho(1),speed(MVSIZ),off(MVSIZ),offg(1)
  real(c_double)::vol(MVSIZ),sti(MVSIZ),ll(MVSIZ),fld(MVSIZ),mu,fqmax
  integer::imat(MVSIZ),istab(MVSIZ)
  call slots_native(parameters,slots)
  pm=zero;pm(1,1)=parameters(3)
  pm(20,1)=slots(1);pm(21,1)=slots(2);pm(22,1)=slots(3)
  pm(32,1)=slots(4);pm(100,1)=slots(5);pm(107,1)=slots(6)
  stress(1,:)=sig;rho=kin(1);speed=zero;speed(1)=kin(2)
  vol=zero;vol(1)=kin(3);off=one;offg=one;imat=1;istab=0;sti=zero;ll=zero;fld=zero
  call IC1_NATIVE_SDISTOR_INI(1,sti,110,1,10,imat,istab,pm,stress,rho,speed,off,offg,ll,vol,fld,mu,fqmax)
  out=[sti(1),fld(1),ll(1),mu];flag=istab(1)
end subroutine
