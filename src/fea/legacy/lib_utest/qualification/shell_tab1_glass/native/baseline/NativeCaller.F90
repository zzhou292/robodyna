! SPDX-License-Identifier: AGPL-3.0-or-later
! Selected local LAW44 caller. Complete native law/failure routines own physics;
! all retained MULAWC operations are exact authenticated includes.
module LF_CALLER
  use iso_c_binding, only: c_double,c_int
  use LAW44_POINT_REF_CONSTANT_MOD
  use LAW44_POINT_BRIDGE_MOD, only: LAW44_POINT_PACKET
  use LF_INTERFACES
  implicit none
  type LF_POINT
    real(c_double) :: sig(5)=0,pla(1)=0,epsd(1)=0
  end type
  type LF_WORK
    real(c_double) :: wpla(1)=0
  end type
contains
  subroutine layered_failure_caller(mfunc,npts,curve,basic,linear,rate_control,d1,dt1,time, &
      dx,thk0,area,dm,points,failures,parent,for_g,for,mom,thk,eint,point_values,diag,removed) &
      bind(C,name='layered_failure_caller')
    integer(c_int),value :: mfunc,npts
    real(c_double),intent(in) :: curve(2,npts+1),basic(4),linear(2),rate_control(3), &
        d1,dt1,time,dx(8),thk0(1),area(1),dm
    real(c_double),intent(inout) :: points(7,3),failures(3,3),parent,for_g(1,5),for(1,5), &
        mom(1,3),thk(1),eint(1,2)
    real(c_double),intent(out) :: point_values(13,3),diag(9)
    integer(c_int),intent(out) :: removed
    integer,parameter :: nel=1,jlt=1,mtn=44,ixfem=0,dmg_flag=0
    logical,parameter :: flag_law2=.false.,flag_law25=.false.,flag_zcfac=.true.
    integer :: i,ipt,jpos,nindx,indx(1),idel7nok,ioff_duct(1),foff(1),flags(3)
    logical :: print_fail(1)
    real(c_double) :: exx(1),eyy(1),exy(1),eyz(1),exz(1),kxx(1),kyy(1),kxy(1), &
      degmb(1),degfx(1),vol0(1),thkn(1),yld(1),seq0(1),sigy(1),zcfac(1,2),etse(1), &
      posly(1,3),thkly(3),wm(3),wmc(1),thklyl(1),depsxx(1),depsyy(1),depsxy(1),depsyz(1),depszx(1), &
      signxx(1),signyy(1),signxy(1),signyz(1),signzx(1),sigoxx(1),sigoyy(1),sigoxy(1), &
      pla0(1),vm0(1),vm(1),dpla(1),rho(1),visc(1),off(1),off_old(1),dmg_glob_scale(1), &
      sigoff(1),offl(1),epsd(1),ssp(1),rate(5),base(6),deps(5),values(13), &
      zt,dtinv,fact,vol2,total_rate,mean_pla,max_pla,hardening
    type(LF_POINT) :: lbuf
    type(LF_WORK) :: gbuf
    exx=dx(1);eyy=dx(2);exy=dx(3);eyz=dx(4);exz=dx(5);kxx=dx(6);kyy=dx(7);kxy=dx(8)
    rho=basic(1);off=parent;off_old=off;dmg_glob_scale=one;sigy=zero
    ioff_duct=0;idel7nok=0;print_fail=.false.
    ssp=sqrt(basic(2)/(one-basic(3)*basic(3))/basic(1))
    dtinv=dt1/max(dt1**2,em20)
    total_rate=law44_point_shell_rate(dx,thk(1),dt1)
    call law44_point_section(posly(1,:),thkly,wm)
    mean_pla=zero;max_pla=zero;gbuf%wpla=zero
    hardening=linear(2)*basic(2)/(basic(2)-linear(2))
#include "WorkBefore.inc"
#include "SectionBegin.inc"
#include "Transverse.inc"
    do ipt=1,3
      jpos=ipt
#include "LayerThickness.inc"
      wmc=wm(ipt)
#include "LayerStrain.inc"
      base=points(1:6,ipt);pla0=base(6)
      sigoxx=base(1);sigoyy=base(2);sigoxy=base(3)
      deps=[depsxx(1),depsyy(1),depsxy(1),depsyz(1),depszx(1)]
      rate=[zero,zero,zero,one,zero]
      if(rate_control(1)>zero) rate=[rate_control(1),rate_control(2),total_rate, &
          law44_point_filter(rate_control(3),dt1),points(7,ipt)]
      call LAW44_POINT_PACKET(mfunc,linear(1),hardening,npts,curve,basic(2),basic(3),basic(1), &
          basic(4),base,deps,rate,thklyl(1),thkn(1),values,off(1),time)
      point_values(:,ipt)=values
      thkn=values(9);etse=values(8);sigy=values(10);lbuf%pla=values(6);lbuf%epsd=zero
      signxx=values(1);signyy=values(2);signxy=values(3);signyz=values(4);signzx=values(5)
#include "PointPlasticWork.inc"
#include "CallerFailureIncrement.inc"
      call constant_failure_native(d1,failures(1,ipt),failures(2,ipt),int(failures(3,ipt),c_int), &
          int(off(1),c_int),dpla(1),time,values(1:5),failures(:,ipt))
      foff=int(failures(3,ipt));flags(ipt)=foff(1);offl=one;sigoff=one
#include "PointMask.inc"
#include "SavedStress.inc"
      points(1:5,ipt)=lbuf%sig;points(6,ipt)=values(6);points(7,ipt)=values(13)
#include "LayerResultants.inc"
#include "LayerFactors.inc"
      mean_pla=mean_pla+thkly(ipt)*values(6);max_pla=max(max_pla,values(6))
    enddo
    call layered_failure_parent(flags,thkly,off(1))
#include "ParentPublish.inc"
    ! Material-only observable uses the same final parent mask as the caller.
    for_g=for*off(1)
#include "ThicknessViscosity.inc"
#include "WorkAfter.inc"
#include "WorkAccumulate.inc"
    parent=off(1);removed=idel7nok
    diag=[gbuf%wpla(1),mean_pla,max_pla,zcfac(1,1),zcfac(1,2),yld(1),sigy(1),total_rate,visc(1)]
  end subroutine
end module
