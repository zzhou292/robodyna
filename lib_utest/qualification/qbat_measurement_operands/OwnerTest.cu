// SPDX-License-Identifier: MIT
#include "../qbat_mapped/OwnerFixture.h"

namespace qbat_mapped_test {
TEST(QbatMeasurementOwner,FailedCandidatePreservesOwnerOutputAndAcceptedHistoryThenRetries) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  const auto allocation=rig.batch.allocations();
  const auto stamp=rig.owner.accepted();
  const auto accepted=rig.Accepted();
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView view;
  ASSERT_TRUE(rig.Prepare(token,assembly,view));
  const auto node=rig.fixture.mechanics.domain.Find(13);
  const double nan=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(view.kinematics.position_xyz)+3*node,
      &nan,sizeof(nan),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  qb::BatchDiagnostics diagnostics;
  diagnostics.internal_kick_work=19;
  const auto before=Bytes(diagnostics);
  EXPECT_NE(rig.batch.EvaluateCandidate(rig.owner,token,view,&diagnostics).status,qb::BatchStatus::Success);
  EXPECT_EQ(Bytes(diagnostics),before);
  EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,rig.owner.accepted()));
  EXPECT_EQ(qbat_resident_test::ResultValues(rig.Accepted()),qbat_resident_test::ResultValues(accepted));
  rig.owner.Discard();
  rig.batch.DiscardTrial();
  ASSERT_TRUE(rig.Prepare(token,assembly,view));
  ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,view,&diagnostics).status,qb::BatchStatus::Success);
  EXPECT_GT(diagnostics.attempt,0u);
  EXPECT_EQ(diagnostics.epoch,1u);
  qb::BatchResult trial;
  ASSERT_EQ(rig.batch.CopyPreparedResults(diagnostics,&trial,1).status,qb::BatchStatus::Success);
  EXPECT_NE(qbat_resident_test::ResultValues(trial),qbat_resident_test::ResultValues(accepted));
  const auto completed=diagnostics;
  rig.owner.Discard();
  rig.batch.DiscardTrial();
  const auto old_trial=Bytes(trial);
  EXPECT_NE(rig.batch.CopyPreparedResults(completed,&trial,1).status,qb::BatchStatus::Success);
  EXPECT_EQ(Bytes(trial),old_trial);
  EXPECT_EQ(rig.batch.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(rig.batch.allocations().device_allocations,allocation.device_allocations);
  EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,rig.owner.accepted()));
}
TEST(QbatMeasurementOwner,CompleteForecastRejectsOneByteShortAndAdmitsExactCaps) {
  Rig rig;
  ASSERT_EQ(InitializeOwner(rig.fixture,rig.owner).status,fe::NodalStatus::Ok);
  rig.config=rig.fixture.Config();
  rig.config.owner=rig.owner.accepted();
  const auto forecast=qb::Batch::ForecastMapped(rig.config,rig.fixture.physical,rig.fixture.Witnesses());
  ASSERT_EQ(forecast.report.status,qb::BatchStatus::Success);
  const auto device=forecast.footprint.device_bytes;
  const auto host=forecast.footprint.startup_host_bytes;
  auto config=rig.config;
  config.max_device_bytes=device-1;
  EXPECT_EQ(rig.batch.InitializeMapped(config,rig.fixture.physical,rig.owner,
      rig.fixture.Witnesses()).status,qb::BatchStatus::ResourceLimit);
  EXPECT_EQ(rig.batch.allocations().device_allocations,0u);
  config.max_device_bytes=device;
  config.storage_limits.max_host_bytes=host-1;
  EXPECT_EQ(rig.batch.InitializeMapped(config,rig.fixture.physical,rig.owner,
      rig.fixture.Witnesses()).status,qb::BatchStatus::ResourceLimit);
  EXPECT_EQ(rig.batch.allocations().device_allocations,0u);
  config.storage_limits.max_host_bytes=host;
  ASSERT_EQ(rig.batch.InitializeMapped(config,rig.fixture.physical,rig.owner,
      rig.fixture.Witnesses()).status,qb::BatchStatus::Success);
  EXPECT_EQ(rig.batch.allocations().device_bytes,device);
  EXPECT_EQ(rig.batch.host_bytes(),host);
}
} // namespace qbat_mapped_test
