! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine rd_coating_orientation(points,count,corners,ins,volume) bind(C)
  use iso_c_binding
  implicit none
  real(c_double),intent(in)::points(3,20)
  integer(c_int),intent(in)::count,corners(3)
  integer(c_int),intent(out)::ins
  real(c_double),intent(out)::volume
  integer::nds(20),irect(4),j
  if(count/=8.and.count/=10.and.count/=16.and.count/=20) error stop 'Unselected NDS count'
  if(any(corners<0).or.any(corners>=count)) error stop 'Segment slot out of range'
  do j=1,count
    nds(j)=j
  end do
  irect(1:3)=corners+1
  irect(4)=irect(3)
  volume=12345.0_c_double
  call SEG_INS_OBS(irect,nds,count,ins,points,volume)
end subroutine
