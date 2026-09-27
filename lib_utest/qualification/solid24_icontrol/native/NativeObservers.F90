! SPDX-License-Identifier: AGPL-3.0-or-later
module IC1_NATIVE_OBSERVATIONS
  implicit none
  real(kind=8)::hourglass_work=0
  integer::hourglass_calls=0
end module
subroutine IC1_NATIVE_HOUR_WORK(index,work)
  use IC1_NATIVE_OBSERVATIONS
  implicit none
  integer,intent(in)::index
  real(kind=8),intent(in)::work
  if(index/=1)error stop 'Controlled HEPH observation index outside one-parent coupon'
  hourglass_work=work
  hourglass_calls=hourglass_calls+1
end subroutine
