! SPDX-License-Identifier: AGPL-3.0-or-later
! Independent complete SIGEPS90 with exact selected surrounding caller regions.
subroutine law90_solid_caller_native(prepared,x,y,n,base,cursors,tensor,rate,step,values,next_cursors,status) &
    bind(C,name='law90_solid_caller_native')
  use iso_c_binding,only:c_double,c_int
  use LAW90_FORCE_CONSTANT_MOD
  implicit none
  integer(c_int),intent(in) :: n,cursors(3)
  real(c_double),intent(in) :: prepared(33),x(n),y(n),base(20),tensor(6),rate(6),step(5)
  real(c_double),intent(out) :: values(37)
  integer(c_int),intent(out) :: next_cursors(3),status
  interface
    subroutine point(p,x,y,n,strain,rate,time,history,cursors,values) bind(C,name='law90_native_point')
      import c_double,c_int
      integer(c_int),intent(in) :: n
      real(c_double),intent(in) :: p(33),x(n),y(n),strain(6),rate(6),time
      real(c_double),intent(inout) :: history(10)
      integer(c_int),intent(inout) :: cursors(3)
      real(c_double),intent(out) :: values(11)
    end subroutine
  end interface
  integer,parameter :: nel=1
  integer :: i,imat,local_cursors(3)
  real(c_double) :: rho0(1),rhon(1),voln(1),volo(1),offg(1),off(1),dvol(1),eint(1),vol_avg(1)
  real(c_double) :: etotsh(1,6),es1(1),es2(1),es3(1),es4(1),es5(1),es6(1)
  real(c_double) :: history(10),strain(6),observed(11),amu(1)
  real(c_double) :: sig(1,6),sold1(1),sold2(1),sold3(1),sold4(1),sold5(1),sold6(1)
  real(c_double) :: d1(1),d2(1),d3(1),d4(1),d5(1),d6(1),svis(1,6),e7(1),q(1),qold(1)
  real(c_double) :: dt1,e1,e2,e3,e4,e5,e6,p2,einc(1),dtout,stiout,pm(89,1)
  type material_parameter
    real(c_double) :: rho
  end type
  type material_table
    type(material_parameter) :: mat_param(1)
  end type
  type(material_parameter) :: matparam
  type(material_table) :: mat_elem
  type storage_packet
    real(c_double) :: vol(1),eint(1),rho(1)
  end type
  type(storage_packet) :: lbuf
  values=zero;next_cursors=0;status=0
  pm=zero;pm(1,1)=prepared(2);pm(89,1)=prepared(1)
  i=1;imat=1;matparam%rho=zero
#include "reference_density_default.inc"
  mat_elem%mat_param(1)=matparam
#include "density_reference.inc"
  rhon=base(17);eint=base(18);voln=step(3);volo=step(4)
  offg=one;off=one
  call LAW90_FORCE_DENSITY(pm(1,1),rhon,voln,volo,offg,dvol,eint)
#include "average_volume.inc"
  lbuf%rho=rhon
#include "density.inc"
  etotsh(1,:)=tensor
#include "selected_total.inc"
#include "engineering_strain.inc"
  strain=[es1(1),es2(1),es3(1),es4(1),es5(1),es6(1)]
  history=base(1:10);local_cursors=cursors
  call point(prepared,x,y,n,strain,rate,step(1),history,local_cursors,observed)
  if(observed(11)/=one.or.observed(10)/=zero)then
    status=1
    return
  endif
  call LAW90_CALLER_VISCOSITY(rate,rhon(1),rho0(1),step(3),step(5), &
      observed(7),observed(10),q(1),dtout,stiout)
  sig(1,:)=observed(1:6)
  sold1=base(11);sold2=base(12);sold3=base(13)
  sold4=base(14);sold5=base(15);sold6=base(16)
  d1=rate(1);d2=rate(2);d3=rate(3);d4=rate(4);d5=rate(5);d6=rate(6)
  dt1=step(2);qold=base(19);svis=zero;e7=zero
#include "internal_work.inc"
  lbuf%vol=volo;lbuf%eint=eint
#include "stored_energy.inc"
  values(1:10)=history;values(11:16)=sig(1,:)
  values(17:20)=[rhon(1),lbuf%eint(1),q(1),observed(8)]
  values(21:31)=observed
  values(32:37)=[dvol(1),vol_avg(1),amu(1),einc(1),dtout,stiout]
  next_cursors=local_cursors
end subroutine
