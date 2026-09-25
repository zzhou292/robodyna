! SPDX-License-Identifier: AGPL-3.0-or-later
! Test-only readback of native local values. Never used by production math.
module selection_observations
  use iso_c_binding
  implicit none
  real(c_double) :: retained_distance_squared(1,4)
  real(c_double) :: continuation_distance_squared(1,4)
  integer :: continuation_ingap(1,4),continuation_axes(3,1)
  integer :: continuation_selected(1),continuation_won(1)
  real(c_double) :: impact_raw_lb(1,4),impact_raw_lc(1,4),impact_distance_squared(1,4)
  real(c_double) :: impact_primary_penetration(1,4),impact_opposite_penetration(1,4)
  integer :: impact_primary_far(1,4),impact_opposite_far(1,4)
  integer :: impact_primary_gap(1,4),impact_opposite_gap(1,4)
  integer :: impact_side_selector(2,1),impact_intersection(2,1),impact_recontact(1)
  integer :: impact_side_choice(1),impact_won(1)
end module
