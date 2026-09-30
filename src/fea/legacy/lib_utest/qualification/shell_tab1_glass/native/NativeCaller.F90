! SPDX-License-Identifier: AGPL-3.0-or-later
! TAB1 callbacks reuse the sole authenticated native NIP3 caller loop.
module TAB1_CALLER
  use iso_c_binding,only:c_double,c_int
  use LF_CALLER,only:layered_failure_packet
  use LF_PARENT,only:layered_failure_parent_packet
  use TAB1_POINT_BRIDGE,only:tab1_native_point,tab1_native_defaults
  implicit none
contains
  subroutine tab1_layered_caller(basic,linear,rate_control,table_parameters,dt1,time, &
      dx,thk0,area,dm,points,failures,parent,for_g,for,mom,thk,eint,point_values,diag,removed) &
      bind(C,name='tab1_layered_caller')
    real(c_double),intent(in) :: basic(4),linear(2),rate_control(3),table_parameters(4), &
        dt1,time,dx(8),thk0(1),area(1),dm
    real(c_double),intent(inout) :: points(7,3),failures(5,3),parent,for_g(1,5),for(1,5), &
        mom(1,3),thk(1),eint(1,2)
    real(c_double),intent(out) :: point_values(13,3),diag(9)
    integer(c_int),intent(out) :: removed
    real(c_double) :: unused_curve(2,1)
    unused_curve=0
    call layered_failure_packet(0_c_int,0_c_int,unused_curve,basic,linear,rate_control, &
        table_parameters,dt1,time,dx,thk0,area,dm,points,failures,parent,for_g,for,mom,thk,eint, &
        point_values,diag,removed,Tab1Point,Tab1Parent,1_c_int)
  end subroutine
  subroutine Tab1Point(parameters,history,element,dpla,time,stress)
    real(c_double),intent(in) :: parameters(:),dpla,time,stress(5)
    real(c_double),intent(inout) :: history(:)
    integer(c_int),intent(in) :: element
    real(c_double) :: result(6)
    call tab1_native_point(parameters(1:3),parameters(4),history,element,dpla,time,stress,result)
    if(result(6)/=0) error stop 'Unexpected TAB1 softening branch'
    history=result(1:5)
  end subroutine
  subroutine Tab1Parent(flags,weights,parent)
    integer(c_int),intent(in) :: flags(3)
    real(c_double),intent(in) :: weights(3)
    real(c_double),intent(inout) :: parent
    real(c_double) :: parameters(22),threshold
    call tab1_native_defaults(parameters,threshold)
    call layered_failure_parent_packet(flags,weights,parent,threshold)
  end subroutine
end module
