! SPDX-License-Identifier: AGPL-3.0-or-later
! Test-only readback of native local values. Never used by production math.
module selection_observations
  use iso_c_binding
  implicit none
  real(c_double) :: retained_distance_squared(1,4)
end module
