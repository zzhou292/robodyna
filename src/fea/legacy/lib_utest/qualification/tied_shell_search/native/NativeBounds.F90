! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact I2BUC1 bounds setup and I2TRIVOX box/filter extracts, working units.
subroutine tied_native_bounds(x,local_thkmain,thksecnd,result) bind(C,name='tied_native_bounds')
  use iso_c_binding,only:c_double
  use constant_mod
  implicit none
  real(c_double),intent(in) :: x(3,5),local_thkmain,thksecnd
  real(c_double),intent(out) :: result(8)
  integer,parameter :: nrtm=1
  integer :: l,i,j,n1,n2,n3,n4,irect(4,1),nodes_id(4),node_id,attempt
  real(c_double) :: segment_data(1,2),dd,dx1,dy1,dz1,dx3,dy3,dz3,dsearch
  real(c_double) :: x_nodes(3,4),x_min(3),x_max(3),x_node(3)
  irect(:,1)=[1,2,3,4]
  segment_data=zero
  dd=zero
#include "SearchDiagonals.inc"
  i=1
#include "SearchThickness.inc"
  dsearch=zero
#include "SearchInflation.inc"
  i=1
#include "SearchBox.inc"
  result(1:3)=x_min
  result(4:6)=x_max
  result(7)=segment_data(1,2)
  result(8)=zero
  node_id=5
  do attempt=1,1
#include "SearchBoxFilter.inc"
    result(8)=one
  enddo
end subroutine
