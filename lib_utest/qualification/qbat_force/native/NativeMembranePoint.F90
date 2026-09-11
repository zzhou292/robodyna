! SPDX-License-Identifier: AGPL-3.0-or-later
! Reusable selected one-thickness-point LAW44 caller. Complete SIGEPS44C and
! FAIL_JOHNSON_C leaves; exact MULAWC work/stress/thickness fragments.
module MEMBRANE_LAW44_CALLER
  use iso_c_binding, only: c_double,c_int
  use LAW44_POINT_REF_CONSTANT_MOD
  use LAW44_POINT_BRIDGE_MOD, only: LAW44_POINT_PACKET
  implicit none
  type MEMBRANE_POINT_STORAGE
    real(c_double) :: sig(5)=0,pla(1)=0,epsd(1)=0
  end type
  type MEMBRANE_WORK_STORAGE
    real(c_double) :: wpla(1)=0
  end type
  interface
    subroutine constant_failure_native(d1,damage,told,active,element,dpla,time,stress,values) &
        bind(C,name='constant_failure_native')
      import c_double,c_int
      real(c_double),value :: d1,damage,told,dpla,time
      integer(c_int),value :: active,element
      real(c_double),intent(in) :: stress(5)
      real(c_double),intent(out) :: values(3)
    end subroutine
    function law44_point_filter(cutoff,dt) result(value) bind(C,name='law44_point_filter')
      import c_double
      real(c_double),value :: cutoff,dt
      real(c_double) :: value
    end function
  end interface
contains
  subroutine MembranePoint(mfunc,npts,curve,basic,linear,controls,israte,d1,dt,time, &
      deps,rate_value,gs,thickness,area,off,point,failed,values,wpla)
    integer(c_int),intent(in) :: mfunc,npts,israte
    real(c_double),intent(in) :: curve(2,npts+1),basic(3),linear(2),controls(3), &
      d1,dt,time,deps(5),rate_value,gs,thickness,area(1),off
    real(c_double),intent(inout) :: point(7),failed(3),wpla
    real(c_double),intent(out) :: values(13)
    integer,parameter :: jlt=1,nel=1
    integer :: i
    real(c_double) :: base(6),rate(5),hardening,pla0(1),sigoxx(1),sigoyy(1),sigoxy(1), &
      signxx(1),signyy(1),signxy(1),signyz(1),signzx(1),vm0(1),vm(1),dpla(1), &
      thklyl(1),sigoff(1),fail_values(3),epsd(1)
    type(MEMBRANE_POINT_STORAGE) :: lbuf
    type(MEMBRANE_WORK_STORAGE) :: gbuf
    base=point(1:6)
    pla0=base(6)
    sigoxx=base(1);sigoyy=base(2);sigoxy=base(3)
    rate=[zero,zero,zero,one,zero]
    if(israte>0) rate=[controls(1),controls(2),rate_value,law44_point_filter(controls(3),dt),point(7)]
    hardening=linear(2)*basic(2)/(basic(2)-linear(2))
    call LAW44_POINT_PACKET(mfunc,linear(1),hardening,npts,curve,basic(2),basic(3),basic(1), &
      gs,base,deps,rate,thickness,thickness,values,off,time,israte)
    thklyl=thickness
    gbuf%wpla=wpla
    lbuf%pla=values(6)
    signxx=values(1);signyy=values(2);signxy=values(3);signyz=values(4);signzx=values(5)
#include "extracted/PointPlasticWork.inc"
#include "extracted/CallerFailureIncrement.inc"
    call constant_failure_native(d1,failed(1),failed(2),int(failed(3),c_int),int(off,c_int), &
      dpla(1),time,values(1:5),fail_values)
    failed=fail_values
    sigoff=failed(3)
#include "extracted/SavedStress.inc"
    point(1:5)=lbuf%sig
    point(6)=values(6)
    point(7)=values(13)
    wpla=gbuf%wpla(1)
  end subroutine
  subroutine MembraneForceWork(dx,thk0,area,dm,dt,ssp,rho,old_off,parent,values,for,mom,thk,eint)
    real(c_double),intent(in) :: dx(8),thk0(1),area(1),dm,dt,ssp(1),rho(1),old_off,values(13)
    real(c_double),intent(inout) :: parent
    real(c_double),intent(inout) :: for(1,5),mom(1,3),thk(1),eint(1,2)
    integer,parameter :: nel=1,jlt=1,mtn=44,ixfem=0,dmg_flag=0
    logical,parameter :: flag_law2=.false.,flag_law25=.false.,flag_zcfac=.true.
    integer :: i,jpos,nindx,indx(1),ioff_duct(1)
    logical :: print_fail(1)
    real(c_double) :: exx(1),eyy(1),exy(1),eyz(1),exz(1),kxx(1),kyy(1),kxy(1), &
      degmb(1),degfx(1),vol0(1),thkn(1),yld(1),seq0(1),sigy(1),zcfac(1,2),etse(1), &
      signxx(1),signyy(1),signxy(1),signyz(1),signzx(1),thkly(1),wmc(1),visc(1), &
      off(1),off_old(1),dmg_glob_scale(1),dtinv,fact,vol2
    exx=dx(1);eyy=dx(2);exy=dx(3);eyz=dx(4);exz=dx(5)
    kxx=dx(6);kyy=dx(7);kxy=dx(8)
    off=old_off
#include "extracted/WorkBefore.inc"
#include "extracted/OldOffMask.inc"
#include "extracted/SectionBegin.inc"
    signxx=values(1);signyy=values(2);signxy=values(3);signyz=values(4);signzx=values(5)
    thkn=values(9)
    thkly=one;wmc=zero;jpos=1
#include "extracted/LayerResultants.inc"
    off=parent
    off_old=old_off;ioff_duct=0;print_fail=.false.;dmg_glob_scale=one
#include "extracted/ParentPublish.inc"
    parent=off(1)
    dtinv=dt/max(dt**2,em20)
#include "extracted/ThicknessViscosity.inc"
#include "extracted/WorkAfter.inc"
#include "extracted/WorkAccumulate.inc"
  end subroutine
end module
