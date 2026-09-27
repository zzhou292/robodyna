! SPDX-License-Identifier: AGPL-3.0-or-later
module S6_CONTROL_OBSERVATIONS
  implicit none
  real(kind=8)::projection(12)=0
  integer::projection_calls=0
contains
  subroutine S6_CONTROL_RESET()
    projection=0;projection_calls=0
  end subroutine
end module
subroutine S6_CONTROL_PROJECT(index,values)
  use S6_CONTROL_OBSERVATIONS
  implicit none
  integer,intent(in)::index
  real(kind=8),intent(in)::values(12)
  if(index/=1)error stop 'S6 projection observer index'
  projection=values;projection_calls=projection_calls+1
end subroutine
