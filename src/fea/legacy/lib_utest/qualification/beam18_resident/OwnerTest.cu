// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
namespace beam18_resident_test {
TEST(BeamResidentCuda, ActualOrdinaryPartPlainNativeRecurrenceAndOneCommit) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  const auto allocations = rig.batch.allocations();
  Results accepted(rig.fixture.model.parents().size()), after(accepted.size()), candidate(accepted.size());
  b::BatchDiagnostics old, proposed;
  ASSERT_TRUE(rig.Read(accepted, old));
  EXPECT_FALSE(old.has_completed_interval);
  for (unsigned step = 0; step < 8; ++step) {
    SCOPED_TRACE(step);
    fe::NodalTrialToken token; fe::NodalAssemblyView assembly; fe::NodalPreparedView view;
    ASSERT_TRUE(rig.Begin(token, assembly));
    CheckAssembly(rig, token, assembly, accepted);
    ASSERT_TRUE(rig.Prepare(token, assembly, view));
    ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner, token, view, &proposed)));
    ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(proposed, Buffer(candidate))));
    ASSERT_TRUE(rig.Compare(candidate, false, &view));
    b::BatchDiagnostics unchanged;
    ASSERT_TRUE(rig.Read(after, unchanged)); SameResults(accepted, after);
    EXPECT_EQ(unchanged.epoch, old.epoch);
    ASSERT_TRUE(Good(Peer::Commit(rig.batch, rig.owner, token, view, proposed)));
    rig.native_accepted = rig.native_candidate;
    ASSERT_TRUE(rig.Read(after, old)); SameResults(candidate, after);
    accepted = after;
    EXPECT_EQ(old.epoch, step+1);
    EXPECT_TRUE(old.has_completed_interval);
    EXPECT_EQ(old.phase, b::BatchPhase::Accepted);
  }
  EXPECT_EQ(rig.batch.allocations().device_bytes, allocations.device_bytes);
}
TEST(BeamResidentCuda, LateContributorAndCacheFailurePreserveAcceptedAndRetry) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  Results accepted(3), candidate(3), after(3);
  b::BatchDiagnostics old, proposed;
  ASSERT_TRUE(rig.Read(accepted, old));
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    fe::NodalTrialToken token; fe::NodalAssemblyView assembly; fe::NodalPreparedView view;
    ASSERT_TRUE(rig.Begin(token, assembly)); ASSERT_TRUE(rig.Prepare(token, assembly, view));
    ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner, token, view, &proposed)));
    ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(proposed, Buffer(candidate))));
    ASSERT_TRUE(rig.Compare(candidate, false, &view));
    if (attempt == 0) {
      EXPECT_FALSE(Peer::Preflight(rig.batch, rig.owner, token, view, proposed, Peer::Scope(1)));
      EXPECT_FALSE(Peer::Commit(rig.batch, rig.owner, token, view, proposed, false));
    } else if (attempt == 1) {
      const double invalid = std::numeric_limits<double>::infinity();
      ASSERT_EQ(cudaMemcpy(Peer::LastPreparedCouple(rig.batch), &invalid, sizeof(invalid), cudaMemcpyHostToDevice), cudaSuccess);
      after = accepted;
      EXPECT_FALSE(rig.batch.CopyPreparedResults(proposed, Buffer(after)));
      SameResults(accepted, after);
      rig.owner.Discard(); rig.batch.DiscardTrial();
    } else {
      ASSERT_TRUE(Good(Peer::Commit(rig.batch, rig.owner, token, view, proposed)));
      rig.native_accepted = rig.native_candidate;
    }
    b::BatchDiagnostics current;
    ASSERT_TRUE(rig.Read(after, current));
    if (attempt < 2) { SameResults(accepted, after); EXPECT_EQ(current.epoch, 0u); }
    else { SameResults(candidate, after); EXPECT_EQ(current.epoch, 1u); }
    EXPECT_FALSE(rig.batch.CopyPreparedResults(proposed, Buffer(candidate)));
  }
}
TEST(BeamResidentCuda, InitialAuthorityCountsCapsOutputAliasesAndClaimant) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize(false));
  auto wrong = rig.config; ++wrong.configuration_id;
  EXPECT_FALSE(Peer::PreflightAttach(rig.batch, rig.owner, rig.fixture.ledger, rig.fixture.binding,
      rig.fixture.Witnesses(), rig.fixture.model, wrong));
  EXPECT_FALSE(Peer::PreflightAttach(rig.batch, rig.owner, rig.fixture.mechanics.ledger, rig.fixture.binding,
      rig.fixture.Witnesses(), rig.fixture.model, rig.config));
  ASSERT_TRUE(rig.Attach());
  Results output(3); b::BatchDiagnostics diagnostics;
  ASSERT_TRUE(rig.Read(output, diagnostics));
  EXPECT_FALSE(rig.batch.CopyAcceptedResults(rig.owner.accepted(), {output.data(), 2}, &diagnostics));
  auto stamp = rig.owner.accepted(); ++stamp.owner_id;
  EXPECT_FALSE(rig.batch.CopyAcceptedResults(stamp, Buffer(output), &diagnostics));
  auto* model = const_cast<b::Parent*>(rig.fixture.model.parents().data());
  EXPECT_FALSE(rig.batch.CopyAcceptedResults(rig.owner.accepted(), {reinterpret_cast<b::Result*>(model), 3}, &diagnostics));
  EXPECT_FALSE(rig.batch.CopyAcceptedResults(rig.owner.accepted(), Buffer(output), reinterpret_cast<b::BatchDiagnostics*>(output.data())));
  b::BatchForecast forecast;
  ASSERT_TRUE(b::Batch::Forecast(rig.config, rig.fixture.model, forecast));
  b::Batch exact, rejected;
  auto c = rig.config;
  c.limits.max_host_bytes = forecast.startup_host_bytes;
  c.limits.max_device_bytes = forecast.device_bytes;
  ASSERT_TRUE(Good(exact.InitializeJoined(c, rig.fixture.model)));
  --c.limits.max_device_bytes;
  EXPECT_EQ(rejected.InitializeJoined(c, rig.fixture.model).status, b::BatchStatus::ResourceLimit);
}
} // namespace beam18_resident_test
