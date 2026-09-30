// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include "../connector_mapped_assembly/OwnerRanges.h"

namespace physical_publication_test {
namespace {
fe::type13::BatchConfig BeamConfig(const fe::FENodalState& owner) {
  fe::type13::BatchConfig config;
  config.owner = owner.accepted();
  config.configuration_id = Configuration;
  config.qualification_id = Qualification;
  config.assembly = fe::type13::BatchAssembly::CinNativeStiffness;
  return config;
}
bool InitializeOwner(Fixture& fixture, fe::FENodalState& owner) {
  const auto cin = fixture.CinStartup();
  return Good(owner.Initialize(fixture.OwnerConfig(),
      {fixture.x.data(),fixture.v.data(),fixture.w.data(),fixture.domain.node_count(),fixture.q.data()},
      fixture.im.data(), {fixture.fixed.data(),fixture.rotation_fixed.data(),
                         fixture.ij.data(),fixture.present.data()}, fixture.rigid, &cin));
}
}
TEST(Type13MappedCuda, ExactOwnerRosterAndBudgetBeforeAllocationThenRetry) {
  Fixture fixture;
  fe::FENodalState owner;
  ASSERT_TRUE(InitializeOwner(fixture,owner));
  const auto config = BeamConfig(owner);
  fe::type13::Batch batch;
  EXPECT_EQ(batch.InitializeJoined(config,fixture.beam_coefficients).status,
            fe::type13::BatchStatus::InvalidInput);
  EXPECT_EQ(batch.allocations().device_bytes,0u);
  fe::type13::BatchForecast forecast;
  ASSERT_TRUE(fe::type13::Batch::ForecastMapped(config,fixture.physical,fixture.rigid,
                                              fixture.WitnessSource(),forecast));
  fe::type13::BatchMappedLimits cap{forecast.startup_host_bytes-1};
  EXPECT_EQ(batch.InitializeMapped(config,fixture.physical,fixture.rigid,owner,
      fixture.WitnessSource(),cap).status,fe::type13::BatchStatus::ResourceLimit);
  EXPECT_EQ(batch.allocations().device_bytes,0u);
  auto witnesses = fixture.witnesses;
  ++witnesses.back().source_element_id;
  auto wrong = fixture.WitnessSource();
  wrong.witnesses = witnesses.data();
  EXPECT_NE(batch.InitializeMapped(config,fixture.physical,fixture.rigid,owner,wrong).status,
            fe::type13::BatchStatus::Success);
  EXPECT_EQ(batch.allocations().device_bytes,0u);
  auto stale = config;
  ++stale.owner.owner_id;
  EXPECT_NE(batch.InitializeMapped(stale,fixture.physical,fixture.rigid,owner,
      fixture.WitnessSource()).status,fe::type13::BatchStatus::Success);
  EXPECT_EQ(batch.allocations().device_bytes,0u);
  ++cap.max_host_bytes;
  ASSERT_TRUE(Good(batch.InitializeMapped(config,fixture.physical,fixture.rigid,owner,
                                         fixture.WitnessSource(),cap)));
  EXPECT_EQ(batch.allocations().device_bytes,forecast.device_bytes);
  EXPECT_EQ(batch.startup_host_bytes(),forecast.startup_host_bytes);
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  ASSERT_TRUE(Good(owner.BeginTrial(&token,&view)));
  EXPECT_EQ(batch.AssembleAccepted(owner,token,view).status,fe::type13::BatchStatus::InvalidInput);
  // Actual dependent zeros, and a solid-only absent rotation elsewhere, survive
  // the complete proof without fabricated positive inverse M/J.
  double inverse_mass=1,inverse_inertia=1;
  const auto dependent = fixture.domain.Find(901);
  ASSERT_EQ(cudaMemcpy(&inverse_mass,view.mass.inverse_mass+dependent,sizeof(double),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&inverse_inertia,view.inverse_inertia+dependent,sizeof(double),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(inverse_mass,0);
  EXPECT_EQ(inverse_inertia,0);
  ASSERT_TRUE(Good(batch.AssembleMappedAccepted(owner,token,view)));
  fe::type13::BatchDiagnostics diagnostics;
  ASSERT_TRUE(Good(batch.CopyAcceptedDiagnostics(owner.accepted(),&diagnostics)));
  auto* overlap = reinterpret_cast<fe::type13::BatchDiagnostics*>(
      const_cast<fe::RigidBindingMember*>(fixture.rigid.members().data()));
  EXPECT_EQ(batch.CopyAcceptedDiagnostics(owner.accepted(),overlap).status,
            fe::type13::BatchStatus::InvalidInput);
  owner.Discard();
  batch.DiscardTrial();
}
TEST(Type13MappedCuda, LateEndpointMaskRejectsThenEpochZeroRetryUsesSameCache) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  Snapshot before,after;
  ASSERT_TRUE(rig.Read(before));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  ASSERT_TRUE(Good(rig.owner.BeginTrial(&token,&view)));
  const auto endpoint = rig.fixture.domain.Find(13);
  const std::uint8_t invalid = 1, valid = 0;
  auto* mask = const_cast<std::uint8_t*>(view.rotation_fixed)+endpoint;
  ASSERT_EQ(cudaMemcpyAsync(mask,&invalid,sizeof(invalid),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  EXPECT_NE(rig.beams.AssembleMappedAccepted(rig.owner,token,view).status,
            fe::type13::BatchStatus::Success);
  ASSERT_EQ(cudaMemcpyAsync(mask,&valid,sizeof(valid),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  rig.owner.Discard();
  rig.beams.DiscardTrial();
  ASSERT_TRUE(rig.Read(after));
  Exact(before,after);
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics candidate;
  ASSERT_TRUE(rig.Prepare(token,prepared,candidate));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,token,candidate,
      {prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,Qualification,true})));
  EXPECT_EQ(rig.owner.accepted().epoch,1u);
}
TEST(Type13MappedCuda, AuthenticatedOwnerRejectsCrossAliasedAndShiftedAssemblyRangesBeforeRetry) {
  Rig rig;ASSERT_TRUE(rig.Initialize());
  Snapshot before;ASSERT_TRUE(rig.Read(before));
  for(unsigned fault=0;fault<4;++fault) {
    fe::NodalTrialToken token;fe::NodalAssemblyView view;fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(Good(rig.owner.BeginTrial(&token,&view)));
    ASSERT_TRUE(Good(rig.owner.BorrowCinAssembly(token,&cin)));
    connector_owner_test::Distinct(view,cin);
    const auto forged=connector_owner_test::Forge(view,cin,fault);
    ASSERT_EQ(rig.owner.AuthenticateAssemblyView(token,forged).status,fe::NodalStatus::StaleTrial);
    EXPECT_EQ(rig.beams.AssembleMappedAccepted(rig.owner,token,forged).status,fe::type13::BatchStatus::StaleTrial);
    Snapshot after;ASSERT_TRUE(rig.Read(after));Exact(before,after);
    // The rejected token is expired. Retry uses a fresh actual owner view.
    ASSERT_TRUE(Good(rig.owner.BeginTrial(&token,&view)));
    ASSERT_TRUE(Good(rig.beams.AssembleMappedAccepted(rig.owner,token,view)));
    rig.owner.Discard();rig.beams.DiscardTrial();
    ASSERT_TRUE(rig.Read(after));Exact(before,after);
  }
}

} // namespace physical_publication_test
