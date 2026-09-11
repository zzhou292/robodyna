! SPDX-License-Identifier: AGPL-3.0-or-later
! Independent ABI around complete native leaves and exact MULAW caller slices.
subroutine law36_native_point(npts,curve,e,nu,rho,base,motion,amu_value,defaults,values) &
    bind(C,name='law36_native_point')
  use iso_c_binding, only: c_double,c_int
  use LAW36_REF_CONSTANT_MOD
  implicit none
#include "mvsiz_p.inc"
  integer(c_int),value :: npts,defaults
  real(c_double),value :: e,nu,rho,amu_value
  real(c_double),intent(in) :: curve(2,npts+1),base(14),motion(7)
  real(c_double),intent(out) :: values(20)
  integer,parameter :: nel=1
  integer :: i,npf(2),kfunc(3),ipm(13,1),mat(1),ngl(1),vartmp(1,3)
  real(c_double) :: uparam(31),sig(1,6),strain(1,6),dt1,asrate,vis(1),ssp(1)
  real(c_double) :: ep1(1),ep2(1),ep3(1),ep4(1),ep5(1),ep6(1)
  real(c_double) :: de1(1),de2(1),de3(1),de4(1),de5(1),de6(1)
  real(c_double) :: so1(1),so2(1),so3(1),so4(1),so5(1),so6(1)
  real(c_double) :: es1(1),es2(1),es3(1),es4(1),es5(1),es6(1)
  real(c_double) :: s1(1),s2(1),s3(1),s4(1),s5(1),s6(1)
  real(c_double) :: wxx(1),wyy(1),wzz(1),off(1),rho0(1),epsd(1)
  real(c_double) :: dpla(1),returned_dpla,defp(1),defp0(1),yld(1),et(1),amu(1)
  real(c_double) :: dmg(1),planl(1),sigb(1,6),dpdm(MVSIZ),fac(1),signor(MVSIZ,6),uvar(1,1),al_imp(1)
  real(c_double) :: vm0(1),vm(1),voln(1)
  type packet
    real(c_double) :: wpla(1)
  end type
  type(packet) :: lbuf
  call law36_native_globals()
  call law36_native_setup(e,nu,rho,defaults,uparam)
  ipm=0
  ipm(10,1)=3
  ipm(11,1)=1
  npf=[0,2*(npts+1)]
  kfunc=[1,0,0]
  mat=1
  ngl=1
  vartmp=1
  dt1=motion(7)
  sig(1,:)=base(1:6)
  strain(1,:)=base(7:12)
  defp=base(13)
  defp0=defp
  epsd=base(14)
  ep1=motion(1)
  ep2=motion(2)
  ep3=motion(3)
  ep4=motion(4)
  ep5=motion(5)
  ep6=motion(6)
#include "increments.inc"
#include "strains.inc"
  asrate=zero
  call LAW36_REF_MSTRAIN_RATE(nel,0,asrate,epsd,1,ep1,ep2,ep3,ep4,ep5,ep6)
  rho0=rho
  amu=amu_value
  off=one
  fac=one
  dmg=zero
  planl=zero
  sigb=zero
  dpdm=zero
  et=zero
  signor=zero
  uvar=zero
  al_imp=zero
  yld=zero
  dpla=zero
  call LAW36_REF_SIGEPS36(nel,0,3,kfunc,npf,curve,dt1,uparam,rho0, &
      de1,de2,de3,de4,de5,de6,es1,es2,es3,es4,es5,es6, &
      so1,so2,so3,so4,so5,so6,s1,s2,s3,s4,s5,s6,ssp,vis,uvar,off,ngl,0, &
      ipm,mat,epsd,1,yld,defp,dpla,et,al_imp,signor,amu,dpdm,fac,3,vartmp, &
      dmg,0,planl,sigb(:,1),sigb(:,2),sigb(:,3),sigb(:,4),sigb(:,5),sigb(:,6))
  returned_dpla=dpla(1)
  lbuf%wpla=zero
  voln=one
#include "plastic_work.inc"
  values(1:6)=[s1(1),s2(1),s3(1),s4(1),s5(1),s6(1)]
  values(7:12)=strain(1,:)
  values(13)=defp(1)
  values(14)=epsd(1)
  values(15)=returned_dpla
  values(16)=yld(1)
  values(17)=vm(1)
  values(18)=ssp(1)
  values(19)=vis(1)
  values(20)=lbuf%wpla(1)
end subroutine
