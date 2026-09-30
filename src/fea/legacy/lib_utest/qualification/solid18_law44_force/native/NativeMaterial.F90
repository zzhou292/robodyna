! SPDX-License-Identifier: AGPL-3.0-or-later
! Existing independent native LAW44 leaf, then exact native work statements.
subroutine REAR18_FORCE_MATERIAL(npts,curve,material,units,base,cursor,rate,dt, &
    current_density,storage_volume,current_volume,volume_increment,prepared_energy, &
    length,values,qout,dtout,stiout,next_cursor,status)
  use iso_c_binding, only:c_double,c_int
  use SOLID18_FORCE_CONSTANT_MOD
  implicit none
  integer,intent(in) :: npts,units,cursor
  real(c_double),intent(in) :: curve(2,npts+1),material(7),base(20),rate(6),dt
  real(c_double),intent(in) :: current_density,storage_volume,current_volume
  real(c_double),intent(in) :: volume_increment,prepared_energy,length
  real(c_double),intent(out) :: values(26),qout,dtout,stiout
  integer,intent(out) :: next_cursor,status
  interface
    subroutine point(n,c,m,u,b,k,d,mu,v,next,p,s) bind(C,name='law44_solid_native')
      import c_double,c_int
      integer(c_int),value :: n,u,k
      real(c_double),intent(in) :: c(2,n+1),m(7),b(14),d(7)
      real(c_double),value :: mu
      real(c_double),intent(out) :: v(19),p(26)
      integer(c_int),intent(out) :: next,s
    end subroutine
    subroutine initial(n,c,m,u,d,mu,v,next,p,s) bind(C,name='law44_solid_initialize_native')
      import c_double,c_int
      integer(c_int),value :: n,u
      real(c_double),intent(in) :: c(2,n+1),m(7),d(7)
      real(c_double),value :: mu
      real(c_double),intent(out) :: v(19),p(26)
      integer(c_int),intent(out) :: next,s
    end subroutine
    subroutine SOLID18_FORCE_VISCOSITY(rate,rho,rho0,vol,length,ssp,vis,q,dt,sti)
      import c_double
      real(c_double),intent(in) :: rate(6),rho,rho0,vol,length,ssp,vis
      real(c_double),intent(out) :: q,dt,sti
    end subroutine
  end interface
  integer,parameter :: nel=1
  integer :: i
  real(c_double) :: motion(7),prepared(26),amu_value,scale_length,scale_stress,scale_density
  real(c_double) :: native_q,native_sti,volume_floor,plastic_increment
  real(c_double) :: vm0(1),vm(1),dpla(1),defp(1),defp0(1)
  real(c_double) :: so1(1),so2(1),so3(1),so4(1),so5(1),so6(1)
  real(c_double) :: s1(1),s2(1),s3(1),s4(1),s5(1),s6(1)
  real(c_double) :: dt1,voln(1),dvol(1),vol_avg(1),sig(1,6)
  real(c_double) :: sold1(1),sold2(1),sold3(1),sold4(1),sold5(1),sold6(1)
  real(c_double) :: d1(1),d2(1),d3(1),d4(1),d5(1),d6(1),e7(1),svis(1,6)
  real(c_double) :: e1,e2,e3,e4,e5,e6,p2,einc(1),eint(1),off(1),q(1),qold(1)
  type packet
    real(c_double) :: eint(1),vol(1),wpla(1)
  end type
  type(packet) :: lbuf
  values=zero
  status=1
  amu_value=current_density/material(3)-one
  motion(1:6)=rate
  motion(7)=dt
  if(dt==zero) then
    call initial(npts,curve,material,units,motion,amu_value, &
                 values(1:19),next_cursor,prepared,status)
  else
    call point(npts,curve,material,units,base(1:14),cursor,motion,amu_value, &
               values(1:19),next_cursor,prepared,status)
  endif
  if(status/=0) return
  voln=current_volume
  defp=values(13)
  defp0=base(13)
  so1=base(1); so2=base(2); so3=base(3)
  so4=base(4); so5=base(5); so6=base(6)
  s1=values(1); s2=values(2); s3=values(3)
  s4=values(4); s5=values(5); s6=values(6)
  lbuf%wpla=zero
#include "plastic_work.inc"
  plastic_increment=lbuf%wpla(1)
  scale_length=one
  scale_stress=one
  scale_density=one
  if(units==1) then
    scale_length=1d-3
    scale_stress=1d6
    scale_density=1d12
  endif
  ! Execute the complete selected MQVISCB statements in declared native units.
  ! Only the independent packet's unit conversion surrounds the shared leaf.
  call SOLID18_FORCE_VISCOSITY(rate,current_density/scale_density, &
      material(3)/scale_density,current_volume/(scale_length**3), &
      length/scale_length,values(17)/scale_length,values(18),native_q,dtout,native_sti)
  qout=native_q*scale_stress
  stiout=native_sti*(scale_stress*scale_length)
  sig(1,:)=values(1:6)
  sold1=base(1); sold2=base(2); sold3=base(3)
  sold4=base(4); sold5=base(5); sold6=base(6)
  d1=rate(1); d2=rate(2); d3=rate(3)
  d4=rate(4); d5=rate(5); d6=rate(6)
  dt1=dt
  qold=base(20)
  q=qout
  off=one
  svis=zero
  e7=zero
  eint=prepared_energy
  dvol=volume_increment
  vol_avg=current_volume-half*volume_increment
#include "internal_work.inc"
  lbuf%eint=eint
  lbuf%vol=storage_volume
  volume_floor=em20*(scale_length*scale_length*scale_length)
  block
    ! MMAIN runs on this SI energy packet, with its declared native volume floor.
    real(c_double) :: em20
    em20=volume_floor
#include "stored_energy.inc"
  end block
  values(20)=plastic_increment/current_volume
  values(21)=lbuf%eint(1)
  values(22)=base(16)+plastic_increment
  values(23)=einc(1)
  values(24)=plastic_increment
  values(25)=amu_value
  values(26)=vol_avg(1)
end subroutine
