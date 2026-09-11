! SPDX-License-Identifier: AGPL-3.0-or-later
! Actual family sound speed plus exact native reference-plane quadrature.
module PLACED_TAB1_FAMILY_CALLER
  use iso_c_binding,only:c_double,c_int
  use placement_LF_CALLER,only:placement_layered_failure_packet
  use TAB1_CALLER,only:Tab1Point,Tab1Parent
  implicit none
  interface
    subroutine placement_native_rule(ipos,position,force,moment) bind(C,name='placement_native_rule')
      use iso_c_binding,only:c_double,c_int
      integer(c_int),value :: ipos
      real(c_double),intent(out) :: position(3),force(3),moment(3)
    end subroutine
  end interface
contains
  subroutine placed_tab1_family_caller_ssp(ipos,mfunc,npts,curve,basic,linear,rate_control,table_parameters, &
      dt1,time,dx,thk0,area,dm,points,failures,parent,for_g,for,mom,thk,eint,point_values,diag,removed,ssp)
    integer(c_int),intent(in) :: ipos,mfunc,npts
    real(c_double),intent(in) :: curve(2,npts+1),basic(4),linear(2),rate_control(3), &
        table_parameters(4),dt1,time,dx(8),thk0(1),area(1),dm,ssp
    real(c_double),intent(inout) :: points(7,3),failures(5,3),parent,for_g(1,5),for(1,5), &
        mom(1,3),thk(1),eint(1,2)
    real(c_double),intent(out) :: point_values(13,3),diag(9)
    integer(c_int),intent(out) :: removed
    real(c_double) :: position(3),force_weight(3),moment_weight(3)
    if(mfunc/=0.or.npts/=0.or.rate_control(1)/=0.or.rate_control(2)/=1.or. &
        (ipos/=0.and.ipos/=3.and.ipos/=4)) error stop 'Unsupported native placed glass composition'
    call placement_native_rule(ipos,position,force_weight,moment_weight)
    call placement_layered_failure_packet(mfunc,npts,curve,basic,linear,rate_control,table_parameters, &
        dt1,time,dx,thk0,area,dm,points,failures,parent,for_g,for,mom,thk,eint,point_values,diag,removed, &
        Tab1Point,Tab1Parent,1_c_int,ssp,position,moment_weight)
  end subroutine
end module
