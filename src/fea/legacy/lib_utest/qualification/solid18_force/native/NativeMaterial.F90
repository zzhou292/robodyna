! SPDX-License-Identifier: AGPL-3.0-or-later
! Independent point -> returned SSP/viscosity -> native internal work order.
subroutine SOLID18_FORCE_MATERIAL(npts,curve,e,nu,rho,base,rate,dt, &
    current_density,storage_volume,current_volume,volume_increment,prepared_energy, &
    length,values,qout,dtout,stiout)
  use iso_c_binding,only:c_double,c_int
  use SOLID18_FORCE_CONSTANT_MOD
  implicit none
  integer,intent(in) :: npts
  real(c_double),intent(in) :: curve(2,npts+1),e,nu,rho,base(20),rate(6),dt
  real(c_double),intent(in) :: current_density,storage_volume,current_volume
  real(c_double),intent(in) :: volume_increment,prepared_energy,length
  real(c_double),intent(out) :: values(26),qout,dtout,stiout
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
  real(c_double) :: dt1,rho0(1),amu(1),voln(1),dvol(1),vol_avg(1),sig(1,6),motion(7)
  real(c_double) :: sold1(1),sold2(1),sold3(1),sold4(1),sold5(1),sold6(1)
  real(c_double) :: d1(1),d2(1),d3(1),d4(1),d5(1),d6(1),e7(1),svis(1,6)
  real(c_double) :: e1,e2,e3,e4,e5,e6,p2,einc(1),eint(1),off(1),q(1),qold(1)
  type packet
    real(c_double) :: rho(1),eint(1),vol(1)
  end type
  type(packet) :: lbuf
  rho0=rho
  lbuf%rho=current_density
  lbuf%vol=storage_volume
  voln=current_volume
  dvol=volume_increment
#include "density.inc"
#include "average_volume.inc"
  motion(1:6)=rate
  motion(7)=dt
  call point(npts,curve,e,nu,rho,base(1:14),motion,amu(1),0,values(1:20))
  call SOLID18_FORCE_VISCOSITY(rate,current_density,rho,current_volume,length, &
      values(18),values(19),qout,dtout,stiout)
  sig(1,:)=values(1:6)
  sold1=base(1)
  sold2=base(2)
  sold3=base(3)
  sold4=base(4)
  sold5=base(5)
  sold6=base(6)
  d1=rate(1)
  d2=rate(2)
  d3=rate(3)
  d4=rate(4)
  d5=rate(5)
  d6=rate(6)
  dt1=dt
  qold=base(20)
  q=qout
  off=one
  svis=zero
  e7=zero
  eint=prepared_energy ! Already scaled once by the complete SRHO3 caller slice.
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
