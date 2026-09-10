! SPDX-License-Identifier: AGPL-3.0-or-later
! Selected OpenRadioss MULAWC NIP3/LAW44 driver. Complete native point law;
! exact unchanged donor statements in authenticated generated includes.
module LR_SECTION_MOD
  use iso_c_binding, only: c_double,c_int
  use, intrinsic :: ieee_arithmetic, only: ieee_is_finite
  use LAW44_POINT_REF_CONSTANT_MOD
  use LAW44_POINT_BRIDGE_MOD, only: LAW44_POINT_PHYSICAL
  implicit none
  private
  public :: LR_UPDATE_SECTION
  type LR_PLASTIC_WORK
    real(c_double) :: wpla(1)=0
  end type
  type LR_POINT_PLA
    real(c_double) :: pla(1)=0
  end type
  interface
    subroutine law44_point_section(pos,force,moment) bind(C,name='law44_point_section')
      import c_double
      real(c_double),intent(out) :: pos(3),force(3),moment(3)
    end subroutine
    function law44_point_filter(cutoff,dt) result(value) bind(C,name='law44_point_filter')
      import c_double
      real(c_double),value :: cutoff,dt
      real(c_double) :: value
    end function
    function law44_point_shell_rate(dx,thickness,dt) result(value) bind(C,name='law44_point_shell_rate')
      import c_double
      real(c_double),intent(in) :: dx(8)
      real(c_double),value :: thickness,dt
      real(c_double) :: value
    end function
  end interface
contains
  subroutine LR_UPDATE_SECTION(npts,curve,basic,rate_control,dt1,thk0,area,ssp,dm,dx, &
      points,for,for_g,mom,thk,eint,diag,status) bind(C,name='lr_update_section')
    integer(c_int),value :: npts
    real(c_double),intent(in) :: curve(2,npts+1),basic(4),rate_control(3),dt1,thk0(1),area(1),ssp(1),dm,dx(8)
    real(c_double),intent(inout) :: points(7,3),for(1,5),for_g(1,5),mom(1,3),thk(1),eint(1,2)
    real(c_double),intent(out) :: diag(8)
    integer(c_int),intent(out) :: status
    integer,parameter :: nel=1,jlt=1,mtn=44
    logical,parameter :: flag_law2=.false.,flag_law25=.false.,flag_zcfac=.true.
    real(c_double) :: exx(1),eyy(1),exy(1),eyz(1),exz(1),kxx(1),kyy(1),kxy(1), &
      degmb(1),degfx(1),vol0(1),thkn(1),yld(1),seq0(1),sigy(1),zcfac(1,2),etse(1), &
      posly(1,3),thkly(3),wm(3),wmc(1),thklyl(1),depsxx(1),depsyy(1),depsxy(1),depsyz(1),depszx(1), &
      signxx(1),signyy(1),signxy(1),signyz(1),signzx(1),sigoxx(1),sigoyy(1),sigoxy(1), &
      pla0(1),vm0(1),vm(1),dpla(1),rho(1),visc(1),off(1),dmg_glob_scale(1), &
      rate(5),base(6),deps(5),values(13),zt,dtinv,fact,vol2,total_rate,mean_pla,max_pla
    type(LR_PLASTIC_WORK) :: gbuf
    type(LR_POINT_PLA) :: lbuf
    integer :: i,ipt,jpos
    status=2
    if(.not.all(ieee_is_finite(points))) return
    if(.not.(thk0(1)>em20.and.thk(1)>=em30)) return
    exx=dx(1);eyy=dx(2);exy=dx(3);eyz=dx(4);exz=dx(5);kxx=dx(6);kyy=dx(7);kxy=dx(8)
    rho=basic(1);off=one;dmg_glob_scale=one;sigy=zero
    dtinv=dt1/max(dt1**2,em20)
    total_rate=law44_point_shell_rate(dx,thk(1),dt1)
    if(.not.ieee_is_finite(total_rate)) return
    call law44_point_section(posly(1,:),thkly,wm)
    mean_pla=zero;max_pla=zero;gbuf%wpla=zero
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
      call LAW44_POINT_PHYSICAL(npts,curve,basic(2),basic(3),basic(1),basic(4), &
          base,deps,rate,thklyl(1),thkn(1),values)
      if(.not.all(ieee_is_finite(values))) return
      if(values(12)/=one.or.values(6)<base(6).or.values(6)>curve(1,npts+1)) return
      thkn=values(9);etse=values(8);sigy=values(10);lbuf%pla=values(6)
      signxx=values(1);signyy=values(2);signxy=values(3);signyz=values(4);signzx=values(5)
      points(1:5,ipt)=values(1:5);points(6,ipt)=values(6);points(7,ipt)=values(13)
#include "PointPlasticWork.inc"
#include "LayerResultants.inc"
#include "LayerFactors.inc"
      mean_pla=mean_pla+thkly(ipt)*values(6);max_pla=max(max_pla,values(6))
    enddo
    if(.not.(thkn(1)>=em30)) return
    ! Observable material-only resultants, captured before source DM addition.
    for_g=for
#include "ThicknessViscosity.inc"
#include "WorkAfter.inc"
#include "WorkAccumulate.inc"
    diag=[gbuf%wpla(1),mean_pla,max_pla,zcfac(1,1),zcfac(1,2),yld(1),sigy(1),total_rate]
    if(.not.all(ieee_is_finite(diag))) return
    if(.not.all(ieee_is_finite(for)).or..not.all(ieee_is_finite(mom))) return
    if(.not.all(ieee_is_finite(eint)).or..not.all(ieee_is_finite(thk))) return
    status=0
  end subroutine
end module
