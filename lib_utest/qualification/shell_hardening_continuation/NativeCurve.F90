! SPDX-License-Identifier: AGPL-3.0-or-later
! Qualification adapter only: call complete authenticated VINTER unchanged.
subroutine continuation_curve(count, points, probes_count, probes, values, slopes, segments) bind(C)
  use iso_c_binding, only: c_int,c_double
  implicit none
  integer(c_int),value :: count,probes_count
  real(c_double),intent(in) :: points(2,count+1),probes(probes_count)
  real(c_double),intent(out) :: values(probes_count),slopes(probes_count)
  integer(c_int),intent(out) :: segments(probes_count)
  integer :: i,iad(1),ipos(1),ilen(1)
  real(c_double) :: x(1),y(1),dy(1)
  iad=1
  ipos=1
  do i=1,probes_count
    ! SIGEPS44C reconstructs the remaining segment count on every call.
    ilen=count-ipos
    x=probes(i)
    call LAW44_POINT_REF_VINTER(points,iad,ipos,ilen,1,x,dy,y)
    values(i)=y(1)
    slopes(i)=dy(1)
    segments(i)=ipos(1)
  enddo
end subroutine
