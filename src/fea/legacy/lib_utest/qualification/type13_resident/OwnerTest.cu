// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace type13_resident_test {
TEST_F(Type13ResidentCuda, LoadedCompleteHistoryRemovalAndLateParticipantRollback) {
  type13_model_test::Fixture input;
  t::ModelPropertyInput properties[2] = {input.declaration, input.declaration};
  properties[1].source_id = 51;
  for (auto& channel : properties[1].input.channels) {
    channel.failure_negative = -1e-5;
    channel.failure_positive = 1e-5;
  }
  input.connections.back().property = 1;
  auto declared = input.Input();
  declared.properties = properties;
  declared.property_count = 2;
  Rig rig;
  ASSERT_TRUE(rig.Initialize(declared));
  const auto allocation = rig.batch.allocations();
  bool removed = false;
  double maximum_force = 0;
  for (unsigned step = 0; step < 8; ++step) {
    SCOPED_TRACE(step);
    std::vector<t::Evaluation> accepted;
    t::BatchDiagnostics accepted_diagnostics;
    ASSERT_TRUE(rig.Read(accepted, accepted_diagnostics));
    fe::NodalTrialToken token;
    fe::NodalAssemblyView view;
    fe::NodalPreparedView prepared;
    ASSERT_TRUE(rig.Begin(token, view, 1e12));
    rig.CompareAssembly(view);
    ASSERT_TRUE(rig.Prepare(token, view, prepared));
    t::BatchDiagnostics diagnostics;
    ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner, token, prepared, &diagnostics)));
    std::vector<t::Evaluation> values(accepted.size());
    ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(diagnostics, values.data(), values.size())));
    std::vector<t::Evaluation> native_next;
    rig.ComparePrepared(token, prepared, values, native_next);
    removed |= values.back().newly_failed;
    maximum_force = std::max(maximum_force, tl::math::fixed3::Norm(values.back().local_force_N));
    if (step == 2) {
      EXPECT_EQ(t::BatchQualificationPeer::Commit(rig.batch, rig.owner, token,
          prepared, diagnostics, false).status, t::BatchStatus::ElementFailure);
      std::vector<t::Evaluation> after;
      t::BatchDiagnostics after_diagnostics;
      ASSERT_TRUE(rig.Read(after, after_diagnostics));
      EXPECT_TRUE(detail::SameDiagnostics(accepted_diagnostics, after_diagnostics));
      for (std::size_t e = 0; e < after.size(); ++e) {
        Exact(accepted[e], after[e]);
      }
      ASSERT_TRUE(rig.Begin(token, view, 1e12));
      ASSERT_TRUE(rig.Prepare(token, view, prepared));
      ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner, token, prepared, &diagnostics)));
      std::vector<t::Evaluation> retry(values.size());
      ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(diagnostics, retry.data(), retry.size())));
      for (std::size_t e = 0; e < retry.size(); ++e) {
        Exact(values[e], retry[e]);
      }
    }
    ASSERT_TRUE(Good(t::BatchQualificationPeer::Commit(rig.batch, rig.owner, token, prepared, diagnostics)));
    rig.native = std::move(native_next);
    EXPECT_EQ(rig.owner.accepted().epoch, step+1);
  }
  EXPECT_TRUE(removed);
  EXPECT_GT(maximum_force, 1e-3);
  EXPECT_FALSE(rig.native.back().native_history.active);
  EXPECT_EQ(rig.native.back().local_force_N.x, 0);
  EXPECT_EQ(rig.batch.allocations().device_bytes, allocation.device_bytes);
  EXPECT_EQ(rig.batch.allocations().device_allocations, 1u);
}

TEST_F(Type13ResidentCuda, LastElementNumericFailureAndMalformedReadbackPreserveAcceptedState) {
  type13_model_test::Fixture input;
  Rig rig;
  ASSERT_TRUE(rig.Initialize(input.Input()));
  std::vector<t::Evaluation> accepted;
  t::BatchDiagnostics initial;
  ASSERT_TRUE(rig.Read(accepted, initial));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  fe::NodalPreparedView prepared;
  ASSERT_TRUE(rig.Begin(token, view, 1e9));
  ASSERT_TRUE(rig.Prepare(token, view, prepared));
  const auto last_node = rig.source.contributions.records()[3].value.global_node;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  // Fault only the owned candidate endpoint used by the final connection.
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(prepared.kinematics.position_xyz)+3*last_node,
      &nan, sizeof(nan), cudaMemcpyHostToDevice, prepared.stream), cudaSuccess);
  t::BatchDiagnostics output;
  output.owner_id = 999;
  const auto rejected = rig.batch.EvaluateCandidate(rig.owner, token, prepared, &output);
  EXPECT_EQ(rejected.status, t::BatchStatus::ElementFailure);
  EXPECT_EQ(rejected.element, 1u);
  EXPECT_EQ(output.owner_id, 999u);
  rig.Discard();
  std::vector<t::Evaluation> after;
  t::BatchDiagnostics after_diagnostics;
  ASSERT_TRUE(rig.Read(after, after_diagnostics));
  EXPECT_TRUE(detail::SameDiagnostics(initial, after_diagnostics));
  for (std::size_t e = 0; e < accepted.size(); ++e) {
    Exact(accepted[e], after[e]);
  }
  ASSERT_TRUE(rig.Begin(token, view, 1e9));
  ASSERT_TRUE(rig.Prepare(token, view, prepared));
  ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner, token, prepared, &output)));
  auto stale = output;
  stale.internal_work_increment_J[5] += 1;
  auto sentinel = accepted;
  EXPECT_EQ(rig.batch.CopyPreparedResults(stale, sentinel.data(), sentinel.size()).status,
            t::BatchStatus::StaleTrial);
  EXPECT_EQ(rig.batch.CopyPreparedResults(output, reinterpret_cast<t::Evaluation*>(1), 0).status,
            t::BatchStatus::ResourceLimit);
  for (std::size_t e = 0; e < accepted.size(); ++e) {
    Exact(accepted[e], sentinel[e]);
  }
  rig.Discard();
  auto* device = t::BatchQualificationPeer::AcceptedDeviceResults(rig.batch);
  auto* field = &device[1].native_history.channels[5].deformation;
  ASSERT_EQ(cudaMemcpy(field, &nan, sizeof(nan), cudaMemcpyHostToDevice), cudaSuccess);
  after_diagnostics.owner_id = 123;
  EXPECT_EQ(rig.batch.CopyAcceptedResults(rig.owner.accepted(), sentinel.data(), sentinel.size(),
                                          &after_diagnostics).status, t::BatchStatus::NonfiniteResult);
  EXPECT_EQ(after_diagnostics.owner_id, 123u);
  for (std::size_t e = 0; e < accepted.size(); ++e) {
    Exact(accepted[e], sentinel[e]);
  }
  ASSERT_EQ(cudaMemcpy(field, &accepted[1].native_history.channels[5].deformation,
                       sizeof(double), cudaMemcpyHostToDevice), cudaSuccess);
  ASSERT_TRUE(rig.Read(after, after_diagnostics));
}

TEST_F(Type13ResidentCuda, DeviceCapBeforeAllocationAndRetryOnSameEmptyBatch) {
  type13_model_test::Fixture input;
  Rig rig;
  ASSERT_TRUE(rig.Initialize(input.Input()));
  t::BatchForecast forecast;
  ASSERT_TRUE(t::Batch::Forecast(rig.config, rig.source.contributions, forecast));
  t::Batch fresh;
  auto limited = rig.config;
  limited.limits.max_device_bytes = forecast.device_bytes - 1;
  EXPECT_EQ(fresh.InitializeJoined(limited, rig.source.contributions).status, t::BatchStatus::ResourceLimit);
  EXPECT_EQ(fresh.allocations().device_allocations, 0u);
  EXPECT_EQ(fresh.startup_host_bytes(), 0u);
  limited.limits.max_device_bytes = forecast.device_bytes;
  limited.limits.max_host_bytes = forecast.startup_host_bytes;
  ASSERT_TRUE(Good(fresh.InitializeJoined(limited, rig.source.contributions)));
  EXPECT_EQ(fresh.allocations().device_bytes, forecast.device_bytes);
}
} // namespace type13_resident_test
