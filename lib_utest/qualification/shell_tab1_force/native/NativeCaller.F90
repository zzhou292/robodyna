! SPDX-License-Identifier: AGPL-3.0-or-later
! The complete native family retains its geometry, M%SSP and force driver.
! Only its constitutive/failure callback and distinct point history differ.
module TAB1_FAMILY_CALLER
  use iso_c_binding,only:c_double,c_int
  use LF_CALLER,only:layered_failure_packet
  use TAB1_CALLER,only:Tab1Point,Tab1Parent
  implicit none
contains
  subroutine tab1_family_caller_ssp(mfunc,npts,curve,basic,linear,rate_control,table_parameters, &
      dt1,time,dx,thk0,area,dm,points,failures,parent,for_g,for,mom,thk,eint,point_values,diag,removed,ssp)
    integer(c_int),intent(in) :: mfunc,npts
    real(c_double),intent(in) :: curve(2,npts+1),basic(4),linear(2),rate_control(3), &
        table_parameters(4),dt1,time,dx(8),thk0(1),area(1),dm,ssp
    real(c_double),intent(inout) :: points(7,3),failures(5,3),parent,for_g(1,5),for(1,5), &
        mom(1,3),thk(1),eint(1,2)
    real(c_double),intent(out) :: point_values(13,3),diag(9)
    integer(c_int),intent(out) :: removed
    if(mfunc/=0.or.npts/=0.or.rate_control(1)/=0.or.rate_control(2)/=1) &
        error stop 'Unsupported native glass family composition'
    call layered_failure_packet(mfunc,npts,curve,basic,linear,rate_control,table_parameters, &
        dt1,time,dx,thk0,area,dm,points,failures,parent,for_g,for,mom,thk,eint,point_values,diag,removed, &
        Tab1Point,Tab1Parent,1_c_int,ssp)
  end subroutine
end module
