! SPDX-License-Identifier: AGPL-3.0-or-later
! Private serial reference storage. Only records an original native append's
! input ordinal; it never participates in a numerical/native branch.
module lifecycle_observations
  use iso_c_binding
  implicit none
  integer(c_int), allocatable :: optcd_ordinal(:)
  integer(c_int) :: optcd_required_count=0
end module

! Native barriers and register-synchronization boundary in the single-thread
! local qualification wrapper. No shared native computation exists to join.
subroutine my_barrier()
end subroutine
subroutine sync_data(value)
  integer :: value
  ! Deliberately do not read source scratch passed only as a compiler boundary.
end subroutine
