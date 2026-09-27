! SPDX-License-Identifier: AGPL-3.0-or-later
! Bounded observer only. No observed operand is changed; inactive by default.
module LAW90_MODULUS_OBSERVER_STORAGE
  use iso_c_binding,only:c_double
  implicit none
  real(c_double)::values(5,8)=0
  integer::count=0
  logical::active=.false.
end module
subroutine law90_modulus_begin() bind(C,name='law90_modulus_begin')
  use LAW90_MODULUS_OBSERVER_STORAGE
  implicit none
  values=0;count=0;active=.true.
end subroutine
subroutine LAW90_MODULUS_OBSERVE(strain,stress,old,young,maximum)
  use LAW90_MODULUS_OBSERVER_STORAGE
  implicit none
  real(c_double),intent(in)::strain,stress,old,young,maximum
  if(.not.active)return
  if(count>=8)error stop 'LAW90 modulus observer capacity'
  count=count+1;values(:,count)=[strain,stress,old,young,maximum]
end subroutine
subroutine law90_modulus_read(output) bind(C,name='law90_modulus_read')
  use LAW90_MODULUS_OBSERVER_STORAGE
  implicit none
  real(c_double),intent(out)::output(5,8)
  if(.not.active.or.count/=8)error stop 'LAW90 modulus observer incomplete caller'
  output=values;active=.false.
end subroutine
