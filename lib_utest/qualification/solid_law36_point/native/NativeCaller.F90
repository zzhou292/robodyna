! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine law36_native_caller(npts,curve,e,nu,rho,base,motion,measures,defaults,values) &
    bind(C,name='law36_native_caller')
  use iso_c_binding, only: c_double,c_int
  use LAW36_REF_CONSTANT_MOD
  implicit none
  integer(c_int),value :: npts,defaults
  real(c_double),value :: e,nu,rho
  real(c_double),intent(in) :: curve(2,npts+1),base(16),motion(7),measures(6)
  real(c_double),intent(out) :: values(26)
  interface
    subroutine point(n,c,e,nu,rho,b,k,mu,def,v) bind(C,name='law36_native_point')
      import c_double,c_int
      integer(c_int),value :: n,def
      real(c_double),value :: e,nu,rho,mu
      real(c_double),intent(in) :: c(2,n+1),b(14),k(7)
      real(c_double),intent(out) :: v(20)
    end subroutine
  end interface
  integer,parameter :: nel=1
  integer :: i
  real(c_double) :: dt1,rho0(1),amu(1),voln(1),dvol(1),vol_avg(1),sig(1,6)
  real(c_double) :: sold1(1),sold2(1),sold3(1),sold4(1),sold5(1),sold6(1)
  real(c_double) :: d1(1),d2(1),d3(1),d4(1),d5(1),d6(1),e7(1),svis(1,6)
  real(c_double) :: e1,e2,e3,e4,e5,e6,p2,einc(1),eint(1),off(1),q(1),qold(1)
  type packet
    real(c_double) :: rho(1),eint(1),vol(1)
  end type
  type(packet) :: lbuf
  rho0=rho
  lbuf%rho=measures(1)
  lbuf%vol=measures(2)
  voln=measures(3)
  dvol=measures(4)
#include "density.inc"
#include "average_volume.inc"
  call point(npts,curve,e,nu,rho,base(1:14),motion,amu(1),defaults,values(1:20))
  sig(1,:)=values(1:6)
  sold1=base(1)
  sold2=base(2)
  sold3=base(3)
  sold4=base(4)
  sold5=base(5)
  sold6=base(6)
  d1=motion(1)
  d2=motion(2)
  d3=motion(3)
  d4=motion(4)
  d5=motion(5)
  d6=motion(6)
  dt1=motion(7)
  qold=measures(5)
  q=measures(6)
  off=one
  svis=zero
  e7=zero
  eint=base(15)*lbuf%vol
#include "internal_work.inc"
  lbuf%eint=eint
#include "stored_energy.inc"
  values(21)=lbuf%eint(1)
  values(22)=base(16)+values(20)*voln(1)
  values(23)=einc(1)
  values(24)=values(20)*voln(1)
  values(25)=amu(1)
  values(26)=vol_avg(1)
end subroutine
