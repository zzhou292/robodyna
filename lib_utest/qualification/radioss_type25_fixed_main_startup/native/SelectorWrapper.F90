! SPDX-License-Identifier: AGPL-3.0-or-later
! Qualification-only direct original selector; no translated numerical formula.
subroutine rd_startup_selector(counts,x,irect,candidates,self,i1,i2,result,angle,side,em20_value) bind(C)
  use iso_c_binding
  use startup_native_constants
  use startup_native_observation
  implicit none
  integer(c_int),intent(in)::counts(3),irect(4,counts(2)),candidates(counts(3)),self,i1,i2
  real(c_double),intent(in)::x(3,counts(1))
  integer(c_int),intent(out)::result(3)
  real(c_double),intent(out)::angle(counts(3)),side(counts(3)),em20_value
  integer::copy(counts(3)),irr
  call clear_observations()
  copy=candidates;irr=0
  call I25NEIGH_REMOVEALLBUT1(counts(3),copy,self,irect,x,i1,i2,irr)
  if(score_count/=counts(3))error stop 'Incomplete native selector observation'
  result=[copy(1),irr,selector_calls]
  angle=angles(1:counts(3));side=sides(1:counts(3));em20_value=EM20
end subroutine
