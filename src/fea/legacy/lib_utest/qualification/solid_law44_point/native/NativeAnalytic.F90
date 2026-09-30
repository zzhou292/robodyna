! SPDX-License-Identifier: AGPL-3.0-or-later
! Independent MFUNC0 packet: complete original SIGEPS44 and reader excerpts.
subroutine law44_solid_analytic_native(material,hardening,units,base,motion,amu_value, &
    initialization,values,prepared,status) bind(C,name='law44_solid_analytic_native')
  use iso_c_binding, only:c_double,c_int
  use ieee_arithmetic, only:ieee_is_finite
  use LAW44_SOLID_NATIVE_VALUES, only:law44_solid_values
  implicit none
  integer(c_int),value :: units,initialization
  real(c_double),intent(in) :: material(7),hardening(5),base(14),motion(7)
  real(c_double),value :: amu_value
  real(c_double),intent(out) :: values(19),prepared(26)
  integer(c_int),intent(out) :: status
  real(c_double) :: curve(2,1),history(14)
  integer(c_int) :: cursor
  status=1
  if(initialization/=0.and.initialization/=1) return
  if(.not.all(ieee_is_finite(motion)).or..not.ieee_is_finite(amu_value)) return
  if(initialization==1) then
    if(motion(7)/=0) return
    history=0d0
  else
    if(motion(7)<=0) return
    history=base
  endif
  ! Unconsumed native TF storage is not a material curve; MFUNC and NVARTMP=0.
  curve=0d0
  call law44_solid_values(0,curve,material,units,history,0,motion,amu_value, &
      values,cursor,prepared,status,hardening)
  if(cursor/=0) status=2
end subroutine
