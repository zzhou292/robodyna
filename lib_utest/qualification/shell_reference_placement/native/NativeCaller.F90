! SPDX-License-Identifier: AGPL-3.0-or-later
! Original TAB1 callbacks with the exact native shifted quadrature packet.
module PLACEMENT_CALLER
  use iso_c_binding,only:c_double,c_int
  use PLACEMENT_LF_CALLER,only:placement_layered_failure_packet
  use TAB1_CALLER,only:Tab1Point,Tab1Parent
  implicit none
contains
  subroutine placement_tab1_caller(ipos,basic,linear,rate_control,table_parameters,dt1,time, &
      dx,thk0,area,dm,points,failures,parent,for_g,for,mom,thk,eint,point_values,diag,removed) &
      bind(C,name='placement_tab1_caller')
    integer(c_int),value :: ipos
    real(c_double),intent(in) :: basic(4),linear(2),rate_control(3),table_parameters(4), &
        dt1,time,dx(8),thk0(1),area(1),dm
    real(c_double),intent(inout) :: points(7,3),failures(5,3),parent,for_g(1,5),for(1,5), &
        mom(1,3),thk(1),eint(1,2)
    real(c_double),intent(out) :: point_values(13,3),diag(9)
    integer(c_int),intent(out) :: removed
    real(c_double) :: unused_curve(2,1),position(3),force_weight(3),moment_weight(3)
    interface
      subroutine placement_native_rule(ipos,position,force,moment) bind(C,name='placement_native_rule')
        import c_double,c_int
        integer(c_int),value :: ipos
        real(c_double),intent(out) :: position(3),force(3),moment(3)
      end subroutine
    end interface
    unused_curve=0
    call placement_native_rule(ipos,position,force_weight,moment_weight)
    call placement_layered_failure_packet(0_c_int,0_c_int,unused_curve,basic,linear,rate_control, &
        table_parameters,dt1,time,dx,thk0,area,dm,points,failures,parent,for_g,for,mom,thk,eint, &
        point_values,diag,removed,Tab1Point,Tab1Parent,1_c_int, &
        position_override=position,moment_override=moment_weight)
  end subroutine
end module
