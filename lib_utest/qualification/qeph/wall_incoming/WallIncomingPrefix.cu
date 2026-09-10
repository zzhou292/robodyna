#include "WallIncomingFixture.h"

namespace qeph_wall_incoming_test {
void AdvanceIncoming(IncomingRig& w,NativeSequence& native,IncomingEvidence& evidence,
                     sc::NodalWallDeviceResults& published) {
  auto& r=w.coupled.shell;
  Snapshot base; ASSERT_TRUE(Read(r.owner,base)); OwnerAgreement(r,base,native.state);
  ASSERT_EQ(base.stamp.epoch,native.epoch); ASSERT_EQ(base.stamp.time,native.time);
  PortResults cache{}; q::BatchDiagnostics accepted; ASSERT_TRUE(ReadAccepted(r,cache,accepted));
  // Native contact is evaluated on the independent accepted native state
  // BEFORE Propose. In particular, the crossing candidate never supplies
  // the force for the kick that first enters contact.
  const auto native_contact=w.coupled.Host(native.state,native.epoch,1);
  ASSERT_TRUE(native_contact.valid);
  NativeProposal expected; ASSERT_TRUE(native.Propose(r,ContactLoads(native_contact),expected));
  Trial trial; ASSERT_TRUE(Begin(w.coupled,trial));
  Staged components; ASSERT_TRUE(Evaluate(w.coupled,trial,components,native.epoch%2));
  IncomingStage stage;
  ASSERT_TRUE(CheckIncoming(w,base,trial,components,cache,expected,evidence,stage));
  ASSERT_FALSE(::testing::Test::HasFailure())<<"No CW2 receipt after a failed native/contact/ledger/event check";
  const auto element_bytes=Bytes(components.elements);
  const auto contact_bytes=Bytes(components.contact_result);
  ASSERT_TRUE(PublishIncoming(w,trial,stage,published,evidence));
  native.Accept(r,expected);
  EXPECT_EQ(Bytes(published),contact_bytes);
  Snapshot after; ASSERT_TRUE(Read(r.owner,after)); OwnerAgreement(r,after,native.state);
  EXPECT_EQ(after.x,trial.nodal.state.x); EXPECT_EQ(after.v,trial.nodal.state.v);
  EXPECT_EQ(after.omega,trial.nodal.state.omega); EXPECT_EQ(after.q,trial.nodal.state.q);
  ASSERT_TRUE(ReadAccepted(r,cache,accepted)); EXPECT_EQ(Bytes(cache),element_bytes);
  EXPECT_EQ(accepted.epoch,native.epoch); EXPECT_EQ(accepted.attempt,components.shell.attempt);
  EXPECT_EQ(published.diagnostics.base_epoch+1,accepted.epoch);
  EXPECT_EQ(published.diagnostics.time,accepted.time);
  EXPECT_EQ(published.diagnostics.phase,sc::NodalWallDevicePhase::PreparedCandidate);
  EXPECT_EQ(evidence.count,accepted.epoch);
}
void RecordIncoming(const IncomingRig& w,const IncomingEvidence& e,const std::string& suffix) {
  const auto& r=w.coupled.shell;
  Property("h_s"+suffix,r.h); Property("accepted_intervals"+suffix,e.count);
  Property("configured_prefix_horizon_s"+suffix,PrefixHorizon);
  Property("accepted_time_s"+suffix,r.owner.accepted().time);
  Property("initial_kinetic_scale_J"+suffix,static_cast<double>(w.scales.energy));
  Property("initial_momentum_scale_kg_m_s"+suffix,static_cast<double>(w.scales.linear));
  Property("initial_angular_scale_kg_m2_s"+suffix,static_cast<double>(w.scales.angular));
  Property("screened_kappa_N_m3"+suffix,w.model.law().stiffness_per_area);
  Property("crossing_interval_base_epoch"+suffix,e.crossing_base);
  Property("first_applied_contact_base_epoch"+suffix,e.first_applied_base);
  Property("screen_nominal_entry_base_epoch"+suffix,w.schedule.entry_base_epoch);
  Property("omitted_contact_budget_ratio"+suffix,e.omitted_contact);
  Property("omitted_contact_signal_m_s"+suffix,e.omitted_signal);
  Property("omitted_contact_budget_m_s"+suffix,e.omitted_budget);
  Property("premature_endpoint_budget_ratio"+suffix,e.premature_endpoint);
  Property("premature_endpoint_signal_m_s"+suffix,e.premature_signal);
  Property("premature_endpoint_budget_m_s"+suffix,e.premature_budget);
  Property("maximum_regular_rate_ratio"+suffix,e.maximum_regular_rate_ratio);
  Property("maximum_hourglass_rate_ratio"+suffix,e.maximum_hourglass_rate_ratio);
  Property("maximum_contact_work_ratio"+suffix,e.contact.work_ratio);
  Property("maximum_contact_impulse_ratio"+suffix,e.contact.impulse_ratio);
  Property("maximum_contact_defect_ratio"+suffix,e.contact.defect_ratio);
  Property("maximum_source_work_ratio"+suffix,e.source.source_work_ratio);
  Property("maximum_kinetic_work_ratio"+suffix,e.source.kick_ratio);
  Property("maximum_internal_work_ratio"+suffix,e.source.internal_work_ratio);
  Property("maximum_linear_ledger_ratio"+suffix,e.source.momentum_ratio);
  Property("maximum_angular_ledger_ratio"+suffix,e.source.angular_ratio);
  if(e.crossing_base!=UINT64_MAX) {
    const auto& entry=e.accepted[e.crossing_base];
    Property("entry_minimum_base_gap_m"+suffix,entry.minimum_base_gap);
    Property("entry_maximum_base_gap_m"+suffix,entry.maximum_base_gap);
    Property("entry_minimum_endpoint_gap_m"+suffix,entry.minimum_endpoint_gap);
    Property("entry_maximum_endpoint_gap_m"+suffix,entry.maximum_endpoint_gap);
  }
}
void IncomingPrefix(unsigned cells,const ScreenBinding& binding) {
  ::testing::Test::RecordProperty("screen_decision_sha256",binding.decision_sha256);
  ::testing::Test::RecordProperty("scope","incoming-entry-prefix-only");
  unsigned native_intervals=0;
  for(unsigned refinement:{1u,2u}) {
    SCOPED_TRACE(refinement);
    IncomingRig w(cells,binding); ASSERT_TRUE(w.Initialize(refinement)); auto& r=w.coupled.shell;
    const auto owner_allocation=r.owner.allocations();
    const auto shell_allocation=r.batch.allocations();
    const auto wall_allocation=w.coupled.wall.allocations();
    EXPECT_FALSE(w.Initialize(refinement)); // An already bound fixture is immutable.
    NativeSequence native; ASSERT_TRUE(w.InitializeNative(native));
    Snapshot initial; ASSERT_TRUE(Read(r.owner,initial)); OwnerAgreement(r,initial,native.state);
    PortResults cache{}; q::BatchDiagnostics initial_d; ASSERT_TRUE(ReadAccepted(r,cache,initial_d));
    ASSERT_NO_FATAL_FAILURE(CheckInitial(w,initial,cache,initial_d));
    ASSERT_FALSE(::testing::Test::HasFailure());
    IncomingEvidence evidence; sc::NodalWallDeviceResults published;
    for(unsigned step=0;step<w.intervals;++step) {
      SCOPED_TRACE(step);
      ASSERT_NO_FATAL_FAILURE(AdvanceIncoming(w,native,evidence,published));
      ASSERT_FALSE(::testing::Test::HasFailure());
      EXPECT_EQ(r.owner.allocations().device_bytes,owner_allocation.device_bytes);
      EXPECT_EQ(r.owner.allocations().device_allocations,owner_allocation.device_allocations);
      EXPECT_EQ(r.batch.allocations().device_bytes,shell_allocation.device_bytes);
      EXPECT_EQ(r.batch.allocations().device_allocations,shell_allocation.device_allocations);
      EXPECT_EQ(w.coupled.wall.allocations().device_bytes,wall_allocation.device_bytes);
      EXPECT_EQ(w.coupled.wall.allocations().device_allocations,wall_allocation.device_allocations);
    }
    ASSERT_NO_FATAL_FAILURE(CheckFinished(w,evidence));
    EXPECT_EQ(r.owner.accepted().time,PrefixHorizon); EXPECT_EQ(native.time,PrefixHorizon);
    EXPECT_EQ(r.owner.accepted().epoch,w.intervals); EXPECT_EQ(native.epoch,w.intervals);
    const auto suffix="_refinement_"+std::to_string(refinement);
    RecordIncoming(w,evidence,suffix);
    Property("owned_device_bytes"+suffix,owner_allocation.device_bytes+shell_allocation.device_bytes+wall_allocation.device_bytes);
    Property("owned_device_allocations"+suffix,owner_allocation.device_allocations+shell_allocation.device_allocations+wall_allocation.device_allocations);
    native_intervals+=cells*w.intervals;
  }
  ::testing::Test::RecordProperty("native_cell_intervals",native_intervals);
  ::testing::Test::RecordProperty("full_response_qualified","false");
}
} // namespace qeph_wall_incoming_test
