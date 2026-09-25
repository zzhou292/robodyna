! SPDX-License-Identifier: AGPL-3.0-or-later
! Serial qualification ABI. The C++ adapter admits exact bounded local tables
! before these calls. Native routines retain all writes and capacity semantics.
subroutine rd_lifecycle_prepare1(counts, candidates_n, candidates_e, main_nodes, &
    main_global, normal_refs, markers, metrics, cache_far, cache_penetration, &
    cache_lb, cache_lc, sliding) bind(C)
  use iso_c_binding
  implicit none
  ! Counts: local secondaries, main segments, normal references, occurrences.
  integer(c_int), intent(in) :: counts(4)
  integer(c_int), intent(in) :: candidates_n(counts(4)), candidates_e(counts(4))
  integer(c_int), intent(in) :: main_nodes(4,counts(2)), main_global(counts(2))
  integer(c_int), intent(in) :: normal_refs(4,counts(2)), cache_far(4,counts(4))
  integer(c_int), intent(inout) :: markers(4,counts(1)), sliding(4,counts(1))
  real(c_double), intent(inout) :: metrics(2,counts(1))
  real(c_double), intent(in) :: cache_penetration(4,counts(4))
  real(c_double), intent(in) :: cache_lb(4,counts(4)), cache_lc(4,counts(4))
  integer(c_int) :: unused_node(1)
  unused_node=0
  ! ITAB and NSV are unused in this complete routine; no node identity is read.
  call I25PREP_SLID_1(counts(4), candidates_n, candidates_e, 1, &
      counts(1), 0, 5, main_global, markers, metrics, unused_node, cache_far, &
      cache_penetration, main_nodes, counts(3), normal_refs, cache_lb, cache_lc, &
      sliding, unused_node)
end subroutine

subroutine rd_lifecycle_expand(counts, main_nodes, main_global, main_role, &
    normal_refs, secondary_nodes, sliding, markers, main_stiffness, &
    normal_offsets, normal_mains, removal_offsets, removed_mains, removal_mode, &
    candidates_n, candidates_e, used, found) bind(C)
  use iso_c_binding
  implicit none
  ! Counts: secondaries, mains, normal refs, candidate capacity, normal
  ! adjacency entries, removal entries. Offsets retain native zero bases;
  ! their entries and every native row/reference identity remain one based.
  integer(c_int), intent(in) :: counts(6), removal_mode
  integer(c_int), intent(in) :: main_nodes(4,counts(2)), main_global(counts(2))
  integer(c_int), intent(in) :: main_role(counts(2)), normal_refs(4,counts(2))
  integer(c_int), intent(in) :: secondary_nodes(counts(1)), sliding(4,counts(1))
  integer(c_int), intent(in) :: markers(4,counts(1)), normal_offsets(counts(3)+1)
  integer(c_int), intent(in) :: normal_mains(counts(5)), removal_offsets(counts(1)+1)
  integer(c_int), intent(in) :: removed_mains(counts(6))
  real(c_double), intent(in) :: main_stiffness(counts(2))
  integer(c_int), intent(inout) :: candidates_n(counts(4)), candidates_e(counts(4)), used
  integer(c_int), intent(out) :: found
  integer(c_int) :: unused_node(1)
  unused_node=0
  ! ITAB, ADMSR and NI25 are not read by the admitted local source arm.
  call I25PREP_SLID_2(candidates_n, candidates_e, 1, 1, counts(1), &
      0, counts(2), counts(4), found, main_global, main_role, used, unused_node, &
      main_nodes, counts(3), normal_refs, sliding, secondary_nodes, &
      normal_offsets, normal_mains, markers, main_stiffness, removal_mode, &
      removal_offsets, removed_mains)
end subroutine

subroutine rd_lifecycle_keep(counts, candidates_n, candidates_e, indices, &
    main_global, markers, cache_penetration, penetration_history, node_ids, &
    secondary_nodes, friction_history, metrics, stiffness_history) bind(C)
  use iso_c_binding
  implicit none
  ! Counts: secondaries, mains, occurrences, positive-candidate index count,
  ! global node-map length. Native diagnostic printing uses this actual map.
  integer(c_int), intent(in) :: counts(5), candidates_e(counts(3)), indices(counts(4))
  integer(c_int), intent(in) :: main_global(counts(2)), node_ids(counts(5))
  integer(c_int), intent(in) :: secondary_nodes(counts(1))
  integer(c_int), intent(inout) :: candidates_n(counts(3)), markers(4,counts(1))
  real(c_double), intent(in) :: cache_penetration(4,counts(3))
  real(c_double), intent(inout) :: penetration_history(5,counts(1))
  real(c_double), intent(inout) :: friction_history(6,counts(1)), metrics(2,counts(1))
  real(c_double), intent(inout) :: stiffness_history(2,counts(1))
  integer :: ispmd
  common /LIFECYCLE_PROCESS/ ispmd
  ispmd=0
  call I25KEEPF(counts(4), indices, candidates_n, candidates_e, 1, counts(1), &
      0, 5, main_global, markers, cache_penetration, penetration_history, 1, &
      node_ids, secondary_nodes, friction_history, metrics, stiffness_history)
end subroutine
