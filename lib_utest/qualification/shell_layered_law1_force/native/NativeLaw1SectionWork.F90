! SPDX-License-Identifier: AGPL-3.0-or-later
! Selected native MULAWC work around the independently qualified SIGEPS01C section.
module L1_SECTION_WORK_MOD
  use iso_c_binding,only:c_double,c_int
  use, intrinsic::ieee_arithmetic,only:ieee_is_finite
  use LAW44_POINT_REF_CONSTANT_MOD
  use LAW1_SECTION_MOD,only:law1_section
  implicit none
  private
  public::L1_UPDATE_SECTION
  interface
    function law44_point_shell_rate(dx,thickness,dt) result(value) bind(C,name='law44_point_shell_rate')
      import c_double
      real(c_double),intent(in)::dx(8)
      real(c_double),value::thickness,dt
      real(c_double)::value
    end function
  end interface
contains
  subroutine L1_UPDATE_SECTION(basic,dt1,thk0,area,ssp,dm,dx,points,for,for_g,mom,thk,eint,total_rate,status)
    real(c_double),intent(in)::basic(4),dt1,thk0(1),area(1),ssp(1),dm,dx(8)
    real(c_double),intent(inout)::points(5,3),for(1,5),for_g(1,5),mom(1,3),thk(1),eint(1,2)
    real(c_double),intent(out)::total_rate
    integer(c_int),intent(out)::status
    integer,parameter::nel=1,jlt=1,mtn=1
    integer i
    real(c_double)::exx(1),eyy(1),exy(1),eyz(1),exz(1),kxx(1),kyy(1),kxy(1),degmb(1),degfx(1), &
      vol0(1),thkn(1),rho(1),visc(1),off(1),dmg_glob_scale(1),dtinv,fact,vol2,values(27),material(3)
    status=2
    if(.not.all(ieee_is_finite(points)))return
    if(.not.(thk0(1)>em20.and.thk(1)>=em30))return
    exx=dx(1);eyy=dx(2);exy=dx(3);eyz=dx(4);exz=dx(5);kxx=dx(6);kyy=dx(7);kxy=dx(8)
    rho=basic(1);off=one;dmg_glob_scale=one
    dtinv=dt1/max(dt1**2,em20)
    ! Existing exact C3FORC3/CZFORC3 scalar helper, not a LAW44 material call.
    total_rate=law44_point_shell_rate(dx,thk(1),dt1)
    if(.not.ieee_is_finite(total_rate))return
#include "WorkBefore.inc"
    vol0=area*thk0
    material=[basic(2),basic(3),basic(1)]
    call law1_section(material,basic(4),dx,thk0(1),thk(1),points,values)
    if(.not.all(ieee_is_finite(values)))return
    if(any(values(24:27)<em30))return
    points=reshape(values(1:15),[5,3])
    for(1,:)=values(16:20);mom(1,:)=values(21:23);thkn=values(24)
    for_g=for
#include "ThicknessViscosity.inc"
#include "WorkAfter.inc"
#include "WorkAccumulate.inc"
    if(.not.all(ieee_is_finite(for)).or..not.all(ieee_is_finite(mom)))return
    if(.not.all(ieee_is_finite(eint)).or..not.all(ieee_is_finite(thk)))return
    status=0
  end subroutine
end module
