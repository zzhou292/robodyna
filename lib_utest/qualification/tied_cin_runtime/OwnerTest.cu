#include "OwnerFixture.h"

namespace cin_runtime_test {
TEST(CinRuntimeCuda, OneSelectorCarriesZeroDependentInertiaHistoryAndHalfKickAcrossThreeStages) {
  Fixture f;
  f.inertia[f.rows.back().secondary] = 0;
  fea::FENodalState owner;
  ASSERT_EQ(Initialize(owner, f).status, fea::NodalStatus::Ok);
  const auto allocation = owner.allocations();
  Snapshot current(f.mass.size(), f.rows.size());
  fea::NodalStamp stamp;
  Accepted(owner, current, stamp);
  for (unsigned step = 0; step < 3; ++step) {
    fea::NodalTrialToken token;
    fea::NodalAssemblyView view;
    fea::NodalCinAssemblyView cin_view;
    Fill(owner, f, token, view, cin_view);
    ASSERT_EQ(owner.SealAssembly(token).status, fea::NodalStatus::Ok);
    ASSERT_EQ(fea::AdvanceStaggeredCin(owner, token, Admission(view)).status, fea::NodalStatus::Ok);
    fea::NodalPreparedView prepared;
    ASSERT_EQ(owner.CopyPreparedCin(token, current.Cin(), &prepared).status, fea::NodalStatus::Ok);
    ASSERT_EQ(owner.CopyPrepared(token, current.Nodes(), &prepared).status, fea::NodalStatus::Ok);
    EXPECT_EQ(prepared.kick_dt, step == 0 ? .5e-6 : 1e-6);
    for (std::size_t row = 0; row < f.rows.size(); ++row) {
      const auto node = f.rows[row].secondary;
      EXPECT_EQ(current.coefficients[node], 0);
      EXPECT_EQ(current.coefficients[f.mass.size()+node], 0);
      EXPECT_EQ(current.coefficients[2*f.mass.size()+row], f.mass[node]);
      EXPECT_EQ(current.coefficients[2*f.mass.size()+f.rows.size()+row], f.inertia[node]);
    }
    EXPECT_EQ(owner.Commit(token).status, fea::NodalStatus::MissingCandidateValidation);
    // A missing validation is terminal for the attempt. Retry without consuming
    // the first half kick, then publish all histories through the sole swap.
    Fill(owner, f, token, view, cin_view);
    ASSERT_EQ(owner.SealAssembly(token).status, fea::NodalStatus::Ok);
    ASSERT_EQ(fea::AdvanceStaggeredCin(owner, token, Admission(view)).status, fea::NodalStatus::Ok);
    Complete(owner, token, view);
    ASSERT_EQ(owner.Commit(token).status, fea::NodalStatus::Ok);
    Accepted(owner, current, stamp);
    EXPECT_EQ(stamp.epoch, step+1);
    EXPECT_EQ(stamp.reaction_kick_dt, step == 0 ? .5e-6 : 1e-6);
  }
  EXPECT_EQ(owner.allocations().device_bytes, allocation.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations, allocation.device_allocations);
}
TEST(CinRuntimeCuda, LastWitnessAndForceFailurePreserveWholeAcceptedStateAndExactRetry) {
  Fixture f;
  fea::FENodalState owner;
  ASSERT_EQ(Initialize(owner, f).status, fea::NodalStatus::Ok);
  Snapshot initial(f.mass.size(), f.rows.size()), clean(f.mass.size(), f.rows.size()), retry(f.mass.size(), f.rows.size());
  fea::NodalStamp initial_stamp, stamp;
  Accepted(owner, initial, initial_stamp);
  fea::NodalTrialToken token;
  fea::NodalAssemblyView view;
  fea::NodalCinAssemblyView cin_view;
  Fill(owner, f, token, view, cin_view);
  ASSERT_EQ(owner.SealAssembly(token).status, fea::NodalStatus::Ok);
  ASSERT_EQ(fea::AdvanceStaggeredCin(owner, token, Admission(view)).status, fea::NodalStatus::Ok);
  fea::NodalPreparedView prepared;
  ASSERT_EQ(owner.CopyPrepared(token, clean.Nodes(), &prepared).status, fea::NodalStatus::Ok);
  ASSERT_EQ(owner.CopyPreparedCin(token, clean.Cin(), &prepared).status, fea::NodalStatus::Ok);
  owner.Discard();
  f.flags.back() = 2;
  Fill(owner, f, token, view, cin_view);
  ASSERT_EQ(owner.SealAssembly(token).status, fea::NodalStatus::Ok);
  auto bad = fea::AdvanceStaggeredCin(owner, token, Admission(view));
  EXPECT_EQ(bad.status, fea::NodalStatus::MissingStepAdmission);
  EXPECT_EQ(bad.node, f.rows.back().secondary);
  Accepted(owner, retry, stamp);
  Same(initial, retry);
  EXPECT_TRUE(fea::trial_identity::SameStamp(stamp, initial_stamp));
  f.flags.back() = 1;
  const auto node = f.rows.back().secondary;
  const auto old = f.load[5*f.mass.size()+node];
  f.load[5*f.mass.size()+node] = std::numeric_limits<double>::max();
  Fill(owner, f, token, view, cin_view);
  ASSERT_EQ(owner.SealAssembly(token).status, fea::NodalStatus::Ok);
  EXPECT_NE(fea::AdvanceStaggeredCin(owner, token, Admission(view)).status, fea::NodalStatus::Ok);
  Accepted(owner, retry, stamp);
  Same(initial, retry);
  f.load[5*f.mass.size()+node] = old;
  Fill(owner, f, token, view, cin_view);
  ASSERT_EQ(owner.SealAssembly(token).status, fea::NodalStatus::Ok);
  ASSERT_EQ(fea::AdvanceStaggeredCin(owner, token, Admission(view)).status, fea::NodalStatus::Ok);
  ASSERT_EQ(owner.CopyPrepared(token, retry.Nodes(), &prepared).status, fea::NodalStatus::Ok);
  ASSERT_EQ(owner.CopyPreparedCin(token, retry.Cin(), &prepared).status, fea::NodalStatus::Ok);
  Same(clean, retry);
  auto short_output = retry.Cin();
  --short_output.capacity_attachments;
  const auto previous = retry.coefficients;
  fea::NodalPreparedView untouched;
  untouched.owner_id = 992;
  EXPECT_EQ(owner.CopyPreparedCin(token, short_output, &untouched).status, fea::NodalStatus::InvalidInput);
  EXPECT_EQ(retry.coefficients, previous);
  EXPECT_EQ(untouched.owner_id, 992u);
  Complete(owner, token, view);
  ASSERT_EQ(owner.Commit(token).status, fea::NodalStatus::Ok);
  Accepted(owner, retry, stamp);
  Same(clean, retry);
}
TEST(CinRuntimeCuda, LastRecoveredMotionFailureDoesNotPublishPartialNodeOrCoefficientState) {
  Fixture f;
  for (const auto node : f.rows.back().masters) f.velocity[3*node] = std::numeric_limits<double>::max();
  fea::FENodalState owner;
  ASSERT_EQ(Initialize(owner, f).status, fea::NodalStatus::Ok);
  Snapshot initial(f.mass.size(), f.rows.size()), after(f.mass.size(), f.rows.size());
  fea::NodalStamp stamp, after_stamp;
  Accepted(owner, initial, stamp);
  fea::NodalTrialToken token;
  fea::NodalAssemblyView view;
  fea::NodalCinAssemblyView cin_view;
  Fill(owner, f, token, view, cin_view);
  ASSERT_EQ(owner.SealAssembly(token).status, fea::NodalStatus::Ok);
  const auto bad = fea::AdvanceStaggeredCin(owner, token, Admission(view));
  EXPECT_EQ(bad.status, fea::NodalStatus::InvalidOutput);
  EXPECT_EQ(bad.node, f.rows.back().secondary);
  Accepted(owner, after, after_stamp);
  Same(initial, after);
  EXPECT_TRUE(fea::trial_identity::SameStamp(stamp, after_stamp));
}
TEST(CinRuntimeCuda, DefaultInverseRulesAndExplicitByteAndAdvanceAdmissionRemainSeparate) {
  Fixture f;
  fea::FENodalState owner;
  EXPECT_EQ(owner.Initialize(f.Config(), f.Kinematics(), f.inverse.data(), f.Dofs()).status,
      fea::NodalStatus::InvalidInput);
  EXPECT_EQ(owner.allocations().device_bytes, 0u);
  auto startup = f.Startup();
  startup.limits.max_device_bytes = 1;
  startup.mass = reinterpret_cast<const double*>(16);
  EXPECT_EQ(owner.Initialize(f.Config(), f.Kinematics(), f.inverse.data(), f.Dofs(), startup).status,
      fea::NodalStatus::ResourceLimit);
  EXPECT_EQ(owner.allocations().device_bytes, 0u);
  ASSERT_EQ(Initialize(owner, f).status, fea::NodalStatus::Ok);
  Snapshot initial(f.mass.size(), f.rows.size()), after(f.mass.size(), f.rows.size());
  fea::NodalStamp stamp, after_stamp;
  Accepted(owner, initial, stamp);
  fea::NodalTrialToken token;
  fea::NodalAssemblyView view;
  fea::NodalCinAssemblyView cin_view;
  Fill(owner, f, token, view, cin_view);
  ASSERT_EQ(owner.SealAssembly(token).status, fea::NodalStatus::Ok);
  const fea::NodalStaggeredHistoryAdmission legacy{view.owner_id, view.accepted.base_epoch,
      view.attempt, 1e-6, .2, 871};
  EXPECT_EQ(fea::AdvanceStaggeredHistory(owner, token, legacy).status, fea::NodalStatus::MissingStepAdmission);
  Fill(owner, f, token, view, cin_view);
  ASSERT_EQ(owner.SealAssembly(token).status, fea::NodalStatus::Ok);
  auto admission = Admission(view);
  admission.no_explicit_interface_release_event = false;
  EXPECT_EQ(fea::AdvanceStaggeredCin(owner, token, admission).status, fea::NodalStatus::MissingStepAdmission);
  Accepted(owner, after, after_stamp);
  Same(initial, after);
  EXPECT_TRUE(fea::trial_identity::SameStamp(stamp, after_stamp));
}
} // namespace cin_runtime_test
