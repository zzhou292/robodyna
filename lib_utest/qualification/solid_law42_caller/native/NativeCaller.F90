! SPDX-License-Identifier: AGPL-3.0-or-later
! One native material call and the exact selected surrounding caller statements.
subroutine law42_solid_caller_native(parameters,base,gradient,rate,step,values,status) &
    bind(C,name='law42_solid_caller_native')
  use iso_c_binding,only:c_double,c_int
  use LAW42_CALLER_CONSTANT_MOD
  implicit none
  real(c_double),intent(in) :: parameters(4),base(9),gradient(9),rate(6),step(4)
  real(c_double),intent(out) :: values(33)
  integer(c_int),intent(out) :: status
  interface
    subroutine point(p,e,rho,off,v) bind(C,name='law42_native_point')
      import c_double
      real(c_double),intent(in) :: p(4),e(6)
      real(c_double),value :: rho,off
      real(c_double),intent(out) :: v(13)
    end subroutine
    subroutine spectrum(e,v) bind(C,name='law42_native_spectrum')
      import c_double
      real(c_double),intent(in) :: e(6)
      real(c_double),intent(out) :: v(12)
    end subroutine
    subroutine LAW42_CALLER_VISCOSITY(rates,rho,rho0,volume,length,sound,vis,q,dt,sti)
      import c_double
      real(c_double),intent(in) :: rates(6),rho,rho0,volume,length,sound,vis
      real(c_double),intent(out) :: q,dt,sti
    end subroutine
  end interface
  integer,parameter :: nel=1
  integer :: i
  real(c_double) :: rho0,rhon(1),voln(1),volo(1),offg(1),off(1),dvol(1),eint(1),vol_avg(1)
  real(c_double) :: mfxx(1),mfxy(1),mfxz(1),mfyx(1),mfyy(1),mfyz(1),mfzx(1),mfzy(1),mfzz(1)
  real(c_double) :: es1(1),es2(1),es3(1),es4(1),es5(1),es6(1),strain(6),observed(13)
  real(c_double) :: sig(1,6),sold1(1),sold2(1),sold3(1),sold4(1),sold5(1),sold6(1)
  real(c_double) :: d1(1),d2(1),d3(1),d4(1),d5(1),d6(1),svis(1,6),e7(1),q(1),qold(1)
  real(c_double) :: dt1,e1,e2,e3,e4,e5,e6,p2,einc(1),dtout,stiout,evv(1,3),ev(1,3),rv(1)
  real(c_double) :: spectrum_values(12),symmetric_strain(6)
  type energy_packet
    real(c_double) :: vol(1),eint(1)
  end type
  type(energy_packet) :: lbuf
  values=zero
  status=0
  rho0=parameters(3)
  rhon=base(7)
  eint=base(8)
  voln=step(2)
  volo=step(3)
  offg=one
  off=one
#include "density_update.inc"
#include "average_volume.inc"
  mfxx=gradient(1)
  mfxy=gradient(2)
  mfxz=gradient(3)
  mfyx=gradient(4)
  mfyy=gradient(5)
  mfyz=gradient(6)
  mfzx=gradient(7)
  mfzy=gradient(8)
  mfzz=gradient(9)
#include "total_strain.inc"
#include "engineering_strain.inc"
  strain=[es1(1),es2(1),es3(1),es4(1),es5(1),es6(1)]
  call point(parameters,strain,rhon(1),one,observed)
  if(observed(9)/=one)then
    status=1 ! Unsupported active-only material cutoff; not a deletion receipt.
    return
  endif
  call LAW42_CALLER_VISCOSITY(rate,rhon(1),rho0,step(2),step(4), &
      observed(10),observed(12),q(1),dtout,stiout)
  sig(1,:)=observed(1:6)
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
  dt1=step(1)
  qold=base(9)
  svis=zero
  e7=zero
#include "internal_work.inc"
  lbuf%vol=volo
  lbuf%eint=eint
#include "stored_energy.inc"
  ! Observe RV through the same complete native eigensolver and exact leaf
  ! expressions. This does not feed material, viscosity, work or future state.
  symmetric_strain=strain
  symmetric_strain(4:6)=half*symmetric_strain(4:6)
  call spectrum(symmetric_strain,spectrum_values)
  evv(1,:)=spectrum_values(1:3)
  i=1
#include "observed_stretches.inc"
#include "observed_volume.inc"
  values(1:6)=sig(1,:)
  values(7:9)=[rhon(1),lbuf%eint(1),q(1)]
  values(10:15)=observed(1:6)
  values(16:22)=[observed(7),observed(8),observed(9),rv(1),observed(10),observed(11),observed(12)]
  values(23:28)=strain
  values(29:33)=[dvol(1),vol_avg(1),einc(1),dtout,stiout]
end subroutine
