! SPDX-License-Identifier: AGPL-3.0-or-later
! Original scale expressions and native constants; no production header used.
subroutine solid_startup_native_stiffness(nodes,raw,value,status) bind(C)
  use iso_c_binding,only:c_int,c_double
  use LAW42_CALLER_CONSTANT_MOD,only:FOURTH,THIRD
  implicit none
  integer(c_int),value,intent(in) :: nodes
  real(c_double),value,intent(in) :: raw
  real(c_double),intent(out) :: value
  integer(c_int),intent(out) :: status
  integer,parameter :: nel=1
  integer :: i
  real(c_double) :: sti(nel)
  sti=raw
  status=0
  if(nodes==8)then
#include "brick_stiffness.inc"
  else if(nodes==6)then
#include "wedge_stiffness.inc"
  else
    status=1
    return
  endif
  value=sti(1)
end subroutine
