! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete original OPTCD under a bounded serial local qualification schedule.
! Input is the retained RAW inventory, after BeginHistory, before MAIN_SLID.
subroutine rd_lifecycle_optcd(counts, precision_mode, previous_dt, kinematics, &
    main_nodes, main_global, main_role, main_stiffness, main_gap, secondary_nodes, &
    secondary_stiffness, secondary_gap, markers, metrics, friction, penetration, &
    stiffness, raw_n, raw_e, output_n, output_e, used, initial_contact, ordinals, required) bind(C)
  use iso_c_binding
  use lifecycle_observations
  implicit none
  ! Counts: nodes, main segments, secondary rows, raw occurrences, output capacity.
  integer(c_int), intent(in) :: counts(5), precision_mode
  real(c_double), intent(in) :: previous_dt, kinematics(3,counts(1),2)
  integer(c_int), intent(in) :: main_nodes(4,counts(2)), main_global(counts(2)), main_role(counts(2))
  real(c_double), intent(in) :: main_stiffness(counts(2)), main_gap(counts(2))
  integer(c_int), intent(in) :: secondary_nodes(counts(3)), raw_e(counts(4))
  real(c_double), intent(in) :: secondary_stiffness(counts(3)), secondary_gap(counts(3))
  integer(c_int), intent(inout) :: markers(4,counts(3)), raw_n(counts(4))
  real(c_double), intent(inout) :: metrics(2,counts(3)), friction(6,counts(3))
  real(c_double), intent(inout) :: penetration(5,counts(3)), stiffness(2,counts(3))
  integer(c_int), intent(inout) :: output_n(counts(5)), output_e(counts(5)), used
  integer(c_int), intent(inout) :: initial_contact(counts(3))
  integer(c_int), intent(out) :: ordinals(counts(5)), required
  integer :: iresp, nspmd, nthread, nvsiz, lskyi_count, ispmd
  common /LIFECYCLE_PRECISION/ iresp
  common /LIFECYCLE_SCHEDULE/ nspmd,nthread,nvsiz
  common /LIFECYCLE_COUNTER/ lskyi_count
  common /LIFECYCLE_PROCESS/ ispmd
  real(c_double) :: dt1
  common /LIFECYCLE_CLOCK/ dt1
  integer :: unused_node(1), counter(1)
  real(c_double) :: unused_gap(1), pmax_gap
  iresp=precision_mode; nspmd=1; nthread=1; nvsiz=128; ispmd=0
  dt1=previous_dt; lskyi_count=0; counter=0; unused_node=0; unused_gap=0; pmax_gap=0
  allocate(optcd_ordinal(counts(5)))
  optcd_ordinal=0; optcd_required_count=0
  ! ICURV/TIME_S/MSEGLO/ITAB/history/MSEGTYP/NRTM are retained in the exact
  ! routine ABI. Selected IGAP1 consumes no GAP_SL/ML; DRAD and DGAPLOAD=0.
  call I25OPTCD(secondary_nodes,raw_e,raw_n,kinematics(:,:,1),counts(4),main_nodes, &
      secondary_gap,main_gap,kinematics(:,:,2),0,secondary_stiffness,0,main_stiffness, &
      1,counts(3),markers,metrics,main_global,counter,unused_node,friction,0, &
      penetration,stiffness,main_role,counts(2),pmax_gap,used,output_e,output_n, &
      counts(5),1,unused_gap,unused_gap,0.0_c_double,0.0_c_double,initial_contact)
  ordinals=optcd_ordinal; required=optcd_required_count
  deallocate(optcd_ordinal)
end subroutine
