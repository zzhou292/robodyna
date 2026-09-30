! SPDX-License-Identifier: AGPL-3.0-or-later
! Same-operand independent native MQVISCB probe. No material/history substitution.
subroutine law90_control_viscosity_native(rate,rho,rho0,volume,length,sound,output) &
    bind(C,name='law90_control_viscosity_native')
  use iso_c_binding,only:c_double
  implicit none
  real(c_double),intent(in)::rate(6),rho,rho0,volume,length,sound
  real(c_double),intent(out)::output(3)
  call LAW90_CALLER_VISCOSITY(rate,rho,rho0,volume,length,sound,0.d0,output(1),output(2),output(3))
end subroutine
