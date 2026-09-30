! SPDX-License-Identifier: AGPL-3.0-or-later
module LAW44_SOLID_NATIVE_VALUES
  implicit none
  private
  public :: law44_solid_values
contains
subroutine law44_solid_values(npts,curve,material,units,base,cursor,motion,amu_value, &
                             values,next_cursor,prepared,status,analytic)
  use iso_c_binding, only: c_double,c_int
  use law44_solid_setup
  use law44_solid_interface
  implicit none
#include "mvsiz_p.inc"
  integer(c_int),value :: npts,units,cursor
  real(c_double),intent(in) :: curve(2,npts+1),material(7),base(14),motion(7)
  real(c_double),value :: amu_value
  real(c_double),intent(in),optional :: analytic(5)
  real(c_double),intent(out) :: values(19),prepared(26)
  integer(c_int),intent(out) :: next_cursor,status
  integer,parameter :: nel=1
  integer :: i,npf(2),kfunc(1),ipm(13,1),mat(1),ngl(1),vartmp(1,1),israte,mfunc
  real(c_double) :: raw_analytic(5)
  real(c_double) :: uparam(24),raw(7),omega,scale_s,scale_rho,scale_v,tf(2,npts+1)
  real(c_double) :: sig(1,6),strain(1,6),dt1,asrate,vis(1),ssp(1),epsd(1)
  real(c_double) :: ep1(1),ep2(1),ep3(1),ep4(1),ep5(1),ep6(1)
  real(c_double) :: de1(1),de2(1),de3(1),de4(1),de5(1),de6(1)
  real(c_double) :: so1(1),so2(1),so3(1),so4(1),so5(1),so6(1)
  real(c_double) :: es1(1),es2(1),es3(1),es4(1),es5(1),es6(1)
  real(c_double) :: s1(1),s2(1),s3(1),s4(1),s5(1),s6(1)
  real(c_double) :: sv1(1),sv2(1),sv3(1),sv4(1),sv5(1),sv6(1)
  real(c_double) :: wxx(1),wyy(1),wzz(1),off(1),rho0(1),rho(1)
  real(c_double) :: dpla(mvsiz),defp(1),yld(1),et(1),amu(1),dpdm(mvsiz)
  real(c_double) :: voln(1),eint(1),uvar(1,1),pm(9,1)
  status=1
  if(present(analytic)) then
    if(npts/=0.or.cursor/=0) return
  else
    if(npts<2.or.npts>1024.or.cursor<0.or.cursor>=npts-1) return
  endif
  if(motion(7)<0.or.(units/=0.and.units/=1)) return
  scale_s=one
  scale_rho=one
  scale_v=one
  if(units==1) then
    scale_s=1d6
    scale_rho=1d12
    scale_v=1d-3
  endif
  raw=material
  raw(1)=material(1)/scale_s
  raw(3)=material(3)/scale_rho
  raw(7)=material(7)/scale_s
  call law44_solid_globals()
  mfunc=1
  if(present(analytic)) then
    raw_analytic=analytic
    raw_analytic(1:2)=analytic(1:2)/scale_s
    raw_analytic(4)=analytic(4)/scale_s
    call prepare_law44(raw,uparam,omega,raw_analytic)
    mfunc=0
  else
    call prepare_law44(raw,uparam,omega)
  endif
  prepared(1:24)=uparam
  prepared(25)=omega
  prepared(26)=warning_count
  tf=curve
  tf(2,:)=tf(2,:)/scale_s
  npf=[0,2*(npts+1)]
  kfunc=1
  ipm=0
  ipm(3,1)=1
  mat=1
  ngl=1
  ! TF has an unused leading pair. Its offset is independent of segment history.
  vartmp=cursor+1
  dt1=motion(7)
  sig(1,:)=base(1:6)/scale_s
  strain(1,:)=base(7:12)
  defp=base(13)
  epsd=base(14)
  ep1=motion(1)
  ep2=motion(2)
  ep3=motion(3)
  ep4=motion(4)
  ep5=motion(5)
  ep6=motion(6)
#include "increments.inc"
#include "strains.inc"
  i=1
  pm=zero
  pm(9,1)=omega
  block
    integer :: imat
    imat=1
#include "filter.inc"
  end block
  rho0=raw(3)
  rho=rho0  ! Unconsumed by IEOS0 SIGEPS44; AMU below is the actual supplied value.
  amu=amu_value
  off=one
  dpdm=zero
  voln=one
  eint=zero
  uvar=zero
  sv1=zero
  sv2=zero
  sv3=zero
  sv4=zero
  sv5=zero
  sv6=zero
  ssp=-123d0
  vis=-456d0
  et=-789d0
  yld=zero
  dpla=zero
  call L44S_REF_SIGEPS44(nel,24,1,mfunc,kfunc,npf,tf,zero,dt1,uparam,rho0,rho, &
      voln,eint,0,dpdm,ep1,ep2,ep3,ep4,ep5,ep6,de1,de2,de3,de4,de5,de6, &
      es1,es2,es3,es4,es5,es6,so1,so2,so3,so4,so5,so6, &
      s1,s2,s3,s4,s5,s6,sv1,sv2,sv3,sv4,sv5,sv6,ssp,vis,uvar,off,ngl,0, &
      ipm,mat,epsd,1,yld,defp,dpla,amu,israte,asrate,mfunc,vartmp,et)
  values(1:6)=[s1(1),s2(1),s3(1),s4(1),s5(1),s6(1)]*scale_s
  values(7:12)=strain(1,:)
  values(13)=defp(1)
  values(14)=epsd(1)
  values(15)=dpla(1)
  values(16)=yld(1)*scale_s
  values(17)=ssp(1)*scale_v
  values(18)=vis(1)
  values(19)=et(1)
  next_cursor=vartmp(1,1)-1
  status=0
end subroutine

subroutine law44_solid_native(npts,curve,material,units,base,cursor,motion,amu_value, &
                             values,next_cursor,prepared,status) bind(C,name='law44_solid_native')
  use iso_c_binding, only:c_double,c_int
  implicit none
  integer(c_int),value :: npts,units,cursor
  real(c_double),intent(in) :: curve(2,npts+1),material(7),base(14),motion(7)
  real(c_double),value :: amu_value
  real(c_double),intent(out) :: values(19),prepared(26)
  integer(c_int),intent(out) :: next_cursor,status
  status=1
  if(motion(7)<=0) return
  call law44_solid_values(npts,curve,material,units,base,cursor,motion,amu_value, &
                          values,next_cursor,prepared,status)
end subroutine

! Constructor-only entry: no caller-supplied history or cursor.
subroutine law44_solid_initialize_native(npts,curve,material,units,motion,amu_value, &
    values,next_cursor,prepared,status) bind(C,name='law44_solid_initialize_native')
  use iso_c_binding, only:c_double,c_int
  use ieee_arithmetic, only:ieee_is_finite
  implicit none
  integer(c_int),value :: npts,units
  real(c_double),intent(in) :: curve(2,npts+1),material(7),motion(7)
  real(c_double),value :: amu_value
  real(c_double),intent(out) :: values(19),prepared(26)
  integer(c_int),intent(out) :: next_cursor,status
  real(c_double) :: virgin(14)
  status=1
  if(motion(7)/=0.or..not.all(ieee_is_finite(motion)).or..not.ieee_is_finite(amu_value)) return
  virgin=0d0
  call law44_solid_values(npts,curve,material,units,virgin,0,motion,amu_value, &
                          values,next_cursor,prepared,status)
end subroutine
end module
