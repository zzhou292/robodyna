! SPDX-License-Identifier: AGPL-3.0-or-later
! Constructor-only native TT0 packet. No caller history or deformed positions.
subroutine SOLID6Z_FORCE_INITIAL_NATIVE(PARAMETERS,XREF,JAC_I,VELOCITY,PROFILE, &
    GEOMETRY,POINT,HISTORY,FORCES,STABILIZATION,STATUS) bind(C,name='solid6z_force_initial_native')
  use iso_c_binding
  use SOLID6Z_FORCE_CALLER,only:EVALUATE_VALUES
  implicit none
  real(c_double),intent(in) :: PARAMETERS(4),XREF(3,6),JAC_I(11),VELOCITY(3),PROFILE(2)
  real(c_double),intent(out) :: GEOMETRY(98),POINT(33),HISTORY(21),FORCES(54),STABILIZATION(28)
  integer(c_int),intent(out) :: STATUS
  real(c_double) :: BASE(21),V(3,6),STEP(3)
  integer :: N
  BASE=0
  BASE(7)=PARAMETERS(3)
  STEP=[0.0_c_double,PROFILE(1),PROFILE(2)]
  do N=1,6
    V(:,N)=VELOCITY
  end do
  call EVALUATE_VALUES(PARAMETERS,XREF,JAC_I,BASE,XREF,V,STEP, &
      GEOMETRY,POINT,HISTORY,FORCES,STABILIZATION,STATUS,.true.)
end subroutine
