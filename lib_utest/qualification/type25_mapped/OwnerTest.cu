// SPDX-License-Identifier: MIT
#include "OwnerFixture.h"
#include <limits>
namespace type25_mapped_test {
TEST(Type25MappedCuda, PhysicalCapsSourceRolesRosterAndStrictLegacyRoute) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  EXPECT_EQ(rig.batch.allocations().device_allocations,1u);
  for (unsigned fault=0;fault<5;++fault) {
    spring::Batch candidate;
    auto config=rig.config;
    auto witnesses=rig.fixture.Witnesses();
    if (fault==0) config.max_device_bytes=rig.batch.allocations().device_bytes-1;
    if (fault==1) config.max_host_bytes=rig.batch.host_bytes()-1;
    if (fault==2) config.owner.owner_id++;
    if (fault==3) witnesses.witness_count++;
    if (fault==4) config.max_nodes=spring::LegacyCapacity.nodes+1;
    const auto rejected=candidate.InitializeMapped(config,rig.fixture.physical,rig.owner,witnesses,spring::CapacityProfile::Legacy);
    EXPECT_NE(rejected.status,spring::BatchStatus::Success)<<fault;
    EXPECT_EQ(candidate.allocations().device_allocations,0u);
    auto exact=rig.config;
    exact.max_device_bytes=rig.batch.allocations().device_bytes;
    exact.max_host_bytes=rig.batch.host_bytes();
    EXPECT_EQ(candidate.InitializeMapped(exact,rig.fixture.physical,rig.owner,
        rig.fixture.Witnesses(),spring::CapacityProfile::Legacy).status,spring::BatchStatus::Success);
  }
  Fixture restricted(false,false,true);
  fe::FENodalState owner;
  ASSERT_EQ(InitializeOwner(restricted,owner).status,fe::NodalStatus::Ok);
  auto config=restricted.Config();
  config.owner=owner.accepted();
  spring::Batch candidate;
  const auto rejected=candidate.InitializeMapped(config,restricted.physical,owner,
      restricted.Witnesses(),spring::CapacityProfile::Legacy);
  EXPECT_EQ(rejected.status,spring::BatchStatus::InvalidInput);
  EXPECT_EQ(rejected.node,restricted.base.mechanics.domain.Find(10));
  EXPECT_EQ(rejected.element,restricted.model.connection_count()-1);
  EXPECT_EQ(candidate.allocations().device_bytes,0u);
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  ASSERT_EQ(rig.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  EXPECT_EQ(rig.batch.AssembleAccepted(rig.owner,view).status,spring::BatchStatus::InvalidInput);
  ASSERT_EQ(rig.batch.AssembleMappedAccepted(rig.owner,token,view).status,spring::BatchStatus::Success);
  rig.CheckAssembly(token,view);
  auto* alias=reinterpret_cast<spring::BatchDiagnostics*>(const_cast<spring::ConnectionInput*>(
      rig.fixture.physical.coefficients()->type25()->connections()));
  EXPECT_EQ(rig.batch.CopyAcceptedDiagnostics(rig.owner.accepted(),alias).status,spring::BatchStatus::InvalidInput);
  rig.owner.Discard();
}
TEST(Type25MappedCuda, UniquePublicationClaimAndStaleTokenLeaveAcceptedHistoryUntouched) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  using Peer=spring::BatchQualificationPeer;
  EXPECT_NE(Peer::Claim(rig.batch,rig.owner,rig.fixture.physical,rig.config).status,spring::BatchStatus::Success);
  Peer::Release(rig.batch,Peer::Scope(1));
  EXPECT_NE(Peer::Claim(rig.batch,rig.owner,rig.fixture.physical,rig.config,Peer::Scope(1)).status,spring::BatchStatus::Success);
  const auto before=rig.Accepted();
  fe::NodalTrialToken stale;
  fe::NodalAssemblyView stale_view;
  ASSERT_TRUE(rig.Begin(stale,stale_view));
  rig.owner.Discard();
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  ASSERT_TRUE(rig.Begin(token,view));
  EXPECT_EQ(rig.batch.AssembleMappedAccepted(rig.owner,stale,view).status,spring::BatchStatus::StaleTrial);
  const auto after=rig.Accepted();
  for (std::size_t e=0;e<before.size();++e) Exact(before[e],after[e]);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
}
TEST(Type25MappedCuda, ActualCinOwnerNativeRecurrenceLateDiscardRetryAndPostFailureStiffness) {
  Rig rig(true);
  ASSERT_TRUE(rig.Initialize());
  auto native=rig.Accepted();
  bool saw_failed_force=false;
  const auto bytes=rig.batch.allocations().device_bytes;
  for (unsigned step=0;step<4;++step) {
    SCOPED_TRACE(step);
    const auto before=rig.Accepted();
    const auto owner_before=rig.Snapshot();
    const auto stamp=rig.owner.accepted();
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    fe::NodalPreparedView prepared;
    ASSERT_TRUE(rig.Prepare(token,assembly,prepared));
    spring::BatchDiagnostics diagnostics;
    ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&diagnostics).status,spring::BatchStatus::Success);
    std::vector<spring::Evaluation> candidate(rig.config.element_count);
    ASSERT_EQ(rig.batch.CopyPreparedResults(diagnostics,candidate.data(),candidate.size()).status,spring::BatchStatus::Success);
    const auto nodes=rig.Nodes(prepared);
    auto next_native=native;
    for (std::size_t e=0;e<candidate.size();++e) {
      const auto& c=rig.fixture.model.connections()[e];
      const auto& p=rig.fixture.model.properties()[c.property_index].property;
      const spring::EndpointKinematics local[2]{nodes[c.global_node[0]],nodes[c.global_node[1]]};
      next_native[e]=type25_test::NativeEvaluate(rig.fixture.model.source_units(),p,
          rig.fixture.model.references()[e],native[e].history,local,rig.config.owner.fixed_dt);
      CompareNative(candidate[e],next_native[e],rig.fixture.model.source_units(),p,local,rig.config.owner.fixed_dt);
    }
    if (!candidate.back().history.active && before.back().history.active) {
      EXPECT_GT(tl::math::fixed3::Norm(candidate.back().endpoints[0].force_N),0);
      saw_failed_force=true;
    }
    // A later contributor rejects after every TYPE25 parent has prepared.
    EXPECT_EQ(spring::BatchQualificationPeer::Commit(rig.batch,rig.owner,token,prepared,diagnostics,false).status,
        spring::BatchStatus::ElementFailure);
    EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,rig.owner.accepted()));
    const auto after=rig.Accepted();
    for (std::size_t e=0;e<before.size();++e) Exact(before[e],after[e]);
    const auto owner_after=rig.Snapshot();
    for (std::size_t k=0;k<owner_before.size();++k) EXPECT_EQ(Bits(owner_before[k]),Bits(owner_after[k]))<<k;
    ASSERT_TRUE(rig.Prepare(token,assembly,prepared));
    spring::BatchDiagnostics retry_diagnostics;
    ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&retry_diagnostics).status,spring::BatchStatus::Success);
    std::vector<spring::Evaluation> retry(candidate.size());
    ASSERT_EQ(rig.batch.CopyPreparedResults(retry_diagnostics,retry.data(),retry.size()).status,spring::BatchStatus::Success);
    for (std::size_t e=0;e<candidate.size();++e) Exact(candidate[e],retry[e]);
    ASSERT_EQ(spring::BatchQualificationPeer::Commit(rig.batch,rig.owner,token,prepared,retry_diagnostics).status,
        spring::BatchStatus::Success);
    const auto published=rig.Accepted();
    for (std::size_t e=0;e<candidate.size();++e) Exact(candidate[e],published[e]);
    native=std::move(next_native);
    EXPECT_EQ(rig.batch.allocations().device_bytes,bytes);
  }
  EXPECT_TRUE(saw_failed_force);
  EXPECT_FALSE(rig.Accepted().back().history.active);
}
TEST(Type25MappedCuda, LastParentGeometryFailurePreservesAllAcceptedFieldsAndAllowsRetry) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  const auto before=rig.Accepted();
  const auto snapshot=rig.Snapshot();
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  fe::NodalPreparedView prepared;
  ASSERT_TRUE(rig.Prepare(token,view,prepared));
  const auto& last=rig.fixture.model.connections()[rig.config.element_count-1];
  const auto values=rig.Nodes(prepared);
  const auto position=values[last.global_node[0]].position;
  const double xyz[3]{position.x,position.y,position.z};
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(prepared.kinematics.position_xyz)+3*last.global_node[1],
      xyz,sizeof(xyz),cudaMemcpyHostToDevice,prepared.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(prepared.stream),cudaSuccess);
  spring::BatchDiagnostics output;
  output.attempt=12345;
  const auto rejected=rig.batch.EvaluateCandidate(rig.owner,token,prepared,&output);
  EXPECT_EQ(rejected.status,spring::BatchStatus::ElementFailure);
  EXPECT_EQ(rejected.element,rig.config.element_count-1);
  EXPECT_EQ(output.attempt,12345u);
  rig.owner.Discard();
  rig.batch.DiscardTrial();
  const auto after=rig.Accepted();
  for (std::size_t e=0;e<before.size();++e) Exact(before[e],after[e]);
  const auto owner_after=rig.Snapshot();
  for (std::size_t k=0;k<snapshot.size();++k) EXPECT_EQ(Bits(snapshot[k]),Bits(owner_after[k]));
  ASSERT_TRUE(rig.Prepare(token,view,prepared));
  ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&output).status,spring::BatchStatus::Success);
  ASSERT_EQ(spring::BatchQualificationPeer::Commit(rig.batch,rig.owner,token,prepared,output).status,spring::BatchStatus::Success);
}
}
