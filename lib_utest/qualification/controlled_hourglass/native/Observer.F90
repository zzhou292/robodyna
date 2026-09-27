! SPDX-License-Identifier: AGPL-3.0-or-later
module CONTROLLED_LEAF_OBSERVATIONS
  implicit none
  real(kind=8)::work=0,rate(12)=0,force(12)=0
  integer::work_calls=0,mode_calls=0
end module
subroutine IC1_NATIVE_HOUR_WORK(index,value)
  use CONTROLLED_LEAF_OBSERVATIONS
  implicit none
  integer,intent(in)::index
  real(kind=8),intent(in)::value
  if(index/=1)error stop 'Controlled leaf observer index'
  work=value;work_calls=work_calls+1
end subroutine
subroutine CONTROLLED_LEAF_MODES(index,r,f)
  use CONTROLLED_LEAF_OBSERVATIONS
  implicit none
  integer,intent(in)::index
  real(kind=8),intent(in)::r(12),f(12)
  if(index/=1)error stop 'Controlled leaf mode observer index'
  rate=r;force=f;mode_calls=mode_calls+1
end subroutine
