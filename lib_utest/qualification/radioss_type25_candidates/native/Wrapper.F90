! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine rd_pen3_cohort(count,coords,gaps,margin,flags,result) bind(c)
  use iso_c_binding
  implicit none
  integer(c_int), intent(in) :: count, flags(6,2)
  real(c_double), intent(in) :: coords(3,5,2), gaps(2), margin
  real(c_double), intent(out) :: result(2)
  integer :: i, ix1(2),ix2(2),ix3(2),ix4(2),etype(2),ibc(2)
  real(c_double) :: x1(2),x2(2),x3(2),x4(2),y1(2),y2(2),y3(2),y4(2)
  real(c_double) :: z1(2),z2(2),z3(2),z4(2),xi(2),yi(2),zi(2)
  if (count < 1 .or. count > 2) error stop 'Invalid qualification cohort size'
  do i=1,2
    x1(i)=coords(1,1,i); y1(i)=coords(2,1,i); z1(i)=coords(3,1,i)
    x2(i)=coords(1,2,i); y2(i)=coords(2,2,i); z2(i)=coords(3,2,i)
    x3(i)=coords(1,3,i); y3(i)=coords(2,3,i); z3(i)=coords(3,3,i)
    x4(i)=coords(1,4,i); y4(i)=coords(2,4,i); z4(i)=coords(3,4,i)
    xi(i)=coords(1,5,i); yi(i)=coords(2,5,i); zi(i)=coords(3,5,i)
    ix1(i)=flags(1,i); ix2(i)=flags(2,i); ix3(i)=flags(3,i); ix4(i)=flags(4,i)
    etype(i)=flags(5,i); ibc(i)=flags(6,i)
  enddo
  result=0
  call i25pen3(count,margin,x1,x2,x3,x4,y1,y2,y3,y4,z1,z2,z3,z4, &
      xi,yi,zi,result,ix1,ix2,ix3,ix4,gaps,2,etype,ibc)
end subroutine
