! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine law42_solid_initial_modulus_native(parameters,modulus) &
    bind(C,name='law42_solid_initial_modulus_native')
  use iso_c_binding,only:c_double
  use LAW42_CALLER_CONSTANT_MOD
  implicit none
  real(c_double),intent(in) :: parameters(4)
  real(c_double),intent(out) :: modulus
  integer :: i,norder,ilaw
  real(c_double) :: mu(10),al(10),gs,nu,parmat(128),pm(110,1),young,bulk,g
  mu=zero
  al=zero
  mu(1)=parameters(1)
  al(1)=two
  norder=1
  nu=parameters(2)
  parmat=zero
  pm=zero
#include "initial_gs.inc"
#include "initial_slots.inc"
  ilaw=42
  i=1
#include "initial_pm.inc"
  modulus=pm(22,1)
end subroutine
