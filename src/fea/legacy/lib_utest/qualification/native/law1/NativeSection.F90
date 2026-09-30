! SPDX-License-Identifier: AGPL-3.0-or-later
! Native COQINI weights, exact MULAWC NIP3 point/resultant excerpts, SIGEPS01C.
module LAW1_SECTION_MOD
  use iso_c_binding,only:c_double
  implicit none
  interface
    subroutine law1_point_physical(basic,gs,base,inc,layer,thick,values) bind(C,name='law1_point_physical')
      import c_double
      real(c_double),intent(in)::basic(3),base(5),inc(5)
      real(c_double),value::gs,layer,thick
      real(c_double),intent(out)::values(10)
    end subroutine
    subroutine law44_point_section(pos,force,moment) bind(C,name='law44_point_section')
      import c_double
      real(c_double),intent(out)::pos(3),force(3),moment(3)
    end subroutine
  end interface
contains
  subroutine law1_section(basic,gs,dx,thickness,initial_thickness,points,result) bind(C,name='law1_section')
    real(c_double),intent(in)::basic(3),dx(8),points(5,3)
    real(c_double),value::gs,thickness,initial_thickness
    real(c_double),intent(out)::result(27)
    integer,parameter::nel=1,jlt=1
    integer i,ipt,jpos
    real(c_double) zt,thk0(1),thkly(3),thklyl(1),thkn,wm(3),wmc(1),posly(1,3), &
      exx(1),eyy(1),exy(1),kxx(1),kyy(1),kxy(1),depsxx(1),depsyy(1),depsxy(1), &
      signxx(1),signyy(1),signxy(1),signyz(1),signzx(1),for(1,5),mom(1,3),values(10),deps(5)
    call law44_point_section(posly(1,:),thkly,wm)
    thk0=thickness;thkn=initial_thickness
    exx=dx(1);eyy=dx(2);exy=dx(3);kxx=dx(6);kyy=dx(7);kxy=dx(8)
    for=0;mom=0;result=0
    do ipt=1,3
      jpos=ipt
#include "LayerThickness.inc"
#include "LayerStrain.inc"
      deps=[depsxx(1),depsyy(1),depsxy(1),dx(4),dx(5)]
      call law1_point_physical(basic,gs,points(:,ipt),deps,thklyl(1),thkn,values)
      thkn=values(6);wmc=wm(ipt);result(24+ipt)=thkn
      signxx=values(1);signyy=values(2);signxy=values(3);signyz=values(4);signzx=values(5)
      result(5*(ipt-1)+1:5*ipt)=values(1:5)
#include "LayerResultants.inc"
    enddo
    result(16:20)=for(1,:);result(21:23)=mom(1,:);result(24)=thkn
  end subroutine
end module
