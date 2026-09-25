! SPDX-License-Identifier: AGPL-3.0-or-later
! Test-only readback of native local values. Never used by production math.
module selection_observations
  use iso_c_binding
  implicit none
  real(c_double) :: retained_distance_squared(1,4)
  real(c_double) :: continuation_distance_squared(1,4)
  integer :: continuation_ingap(1,4),continuation_axes(3,1)
  integer :: continuation_selected(1),continuation_won(1)
end module
