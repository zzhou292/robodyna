// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include "../rigid_assembly_owner/OwnerFixture.h"

namespace solid_resident_test {
TEST_F(SolidResidentCuda, LastFamilyMaterialFailureAndLaterParticipantRejectPreserveAllAcceptedState) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  Results initial;
  s::BatchDiagnostics initial_diagnostics;
  ASSERT_TRUE(rig.Read(initial,initial_diagnostics));
  rig.native.Compare(initial,0,0);
  rig.native.Accept();
  rigid_assembly_owner_test::Snapshot owner_initial(rig.config.owner.node_count);
  owner_initial.Read(rig.owner);
  for (unsigned attempt=0;attempt<3;++attempt) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    fe::NodalPreparedView prepared;
    ASSERT_TRUE(rig.Begin(token,assembly));
    ASSERT_TRUE(rig.Prepare(token,assembly,prepared));
    s::BatchDiagnostics candidate;
    candidate.owner_id=0xabcdef;
    if (attempt==0) {
      auto* address=Peer::LastMaterialDensity(rig.batch);
      double prior=0;
      ASSERT_EQ(cudaMemcpy(&prior,address,sizeof(prior),cudaMemcpyDeviceToHost),cudaSuccess);
      const double invalid=std::numeric_limits<double>::quiet_NaN();
      ASSERT_EQ(cudaMemcpy(address,&invalid,sizeof(invalid),cudaMemcpyHostToDevice),cudaSuccess);
      const auto rejected=rig.batch.EvaluateCandidate(rig.owner,token,prepared,&candidate);
      EXPECT_EQ(rejected.status,s::BatchStatus::ElementFailure);
      EXPECT_EQ(rejected.family,s::Family::Solid6z);
      EXPECT_EQ(candidate.owner_id,0xabcdefu);
      ASSERT_EQ(cudaMemcpy(address,&prior,sizeof(prior),cudaMemcpyHostToDevice),cudaSuccess);
      rig.owner.Discard();rig.batch.DiscardTrial();
    } else {
      ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&candidate)));
      Results next;
      ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(candidate,next.Buffers())));
      ASSERT_TRUE(rig.ComparePrepared(prepared,next));
      const auto committed=Peer::Commit(rig.batch,rig.owner,token,prepared,candidate,attempt==2);
      if (attempt==1) EXPECT_EQ(committed.status,s::BatchStatus::ElementFailure);
      else ASSERT_TRUE(Good(committed));
    }
    Results accepted;
    s::BatchDiagnostics diagnostics;
    ASSERT_TRUE(rig.Read(accepted,diagnostics));
    if (attempt<2) {
      Exact(initial,accepted);
      EXPECT_TRUE(detail::SameDiagnostics(initial_diagnostics,diagnostics));
      EXPECT_EQ(rig.owner.accepted().epoch,0u);
      rigid_assembly_owner_test::Snapshot owner_unchanged(rig.config.owner.node_count);
      owner_unchanged.Read(rig.owner);
      rigid_assembly_owner_test::Same(owner_initial,owner_unchanged);
    } else EXPECT_EQ(rig.owner.accepted().epoch,1u);
  }
}
TEST_F(SolidResidentCuda, LateReadbackNaNAndAliasingPreserveEveryCallerResult) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView prepared;
  ASSERT_TRUE(rig.Begin(token,assembly));
  ASSERT_TRUE(rig.Prepare(token,assembly,prepared));
  s::BatchDiagnostics candidate;
  ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&candidate)));
  Results output;
  auto buffers=output.Buffers();
  const auto saved=output;
  auto* address=Peer::PreparedLastCacheField(rig.batch);
  double prior=0;
  ASSERT_EQ(cudaMemcpy(&prior,address,sizeof(prior),cudaMemcpyDeviceToHost),cudaSuccess);
  const double nan=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpy(address,&nan,sizeof(nan),cudaMemcpyHostToDevice),cudaSuccess);
  EXPECT_EQ(rig.batch.CopyPreparedResults(candidate,buffers).status,s::BatchStatus::NonfiniteResult);
  EXPECT_EQ(std::memcmp(&output,&saved,sizeof(output)),0); // Failure atomicity, including padding.
  ASSERT_EQ(cudaMemcpy(address,&prior,sizeof(prior),cudaMemcpyHostToDevice),cudaSuccess);
  buffers.solid6z=reinterpret_cast<s::Result6z*>(buffers.solid18);
  EXPECT_EQ(rig.batch.CopyPreparedResults(candidate,buffers).status,s::BatchStatus::InvalidInput);
  EXPECT_EQ(std::memcmp(&output,&saved,sizeof(output)),0);
  buffers=output.Buffers();
  --buffers.count6z;
  EXPECT_EQ(rig.batch.CopyPreparedResults(candidate,buffers).status,s::BatchStatus::InvalidInput);
  ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(candidate,output.Buffers())));
  EXPECT_EQ(output.c.stamp.sample_index,1u);
  rig.owner.Discard();rig.batch.DiscardTrial();
}
} // namespace solid_resident_test
