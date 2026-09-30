// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace solid_resident_test {
TEST_F(SolidResidentCuda, ThreeNativeHistoriesShareActualOwnerStiffnessAndOneCommit) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  Results accepted;
  s::BatchDiagnostics diagnostics;
  ASSERT_TRUE(rig.Read(accepted,diagnostics));
  EXPECT_FALSE(diagnostics.has_completed_interval);
  EXPECT_EQ(diagnostics.phase,s::BatchPhase::Accepted);
  rig.native.Compare(accepted,0,0);
  ASSERT_FALSE(HasFailure());
  rig.native.Accept();
  const auto allocations=rig.batch.allocations();
  EXPECT_EQ(allocations.device_allocations,1u);
  for (unsigned step=0;step<8;++step) {
    SCOPED_TRACE(step);
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    fe::NodalPreparedView prepared;
    ASSERT_TRUE(rig.Begin(token,assembly));
    rig.CompareAssembly(token,assembly,accepted);
    ASSERT_FALSE(HasFailure());
    ASSERT_TRUE(rig.Prepare(token,assembly,prepared));
    s::BatchDiagnostics candidate;
    ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&candidate)));
    Results next;
    ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(candidate,next.Buffers())));
    ASSERT_TRUE(rig.ComparePrepared(prepared,next));
    Results unchanged;
    s::BatchDiagnostics old;
    ASSERT_TRUE(rig.Read(unchanged,old));
    Exact(accepted,unchanged);
    EXPECT_EQ(rig.owner.accepted().epoch,step);
    EXPECT_EQ(Peer::Preflight(rig.batch,rig.owner,token,prepared,candidate,Peer::Scope(1)).status,
        s::BatchStatus::StaleTrial);
    ASSERT_TRUE(Good(Peer::Commit(rig.batch,rig.owner,token,prepared,candidate)));
    rig.native.Accept();
    ASSERT_TRUE(rig.Read(accepted,diagnostics));
    Exact(next,accepted);
    EXPECT_EQ(rig.owner.accepted().epoch,step+1);
    EXPECT_EQ(diagnostics.epoch,step+1);
    EXPECT_TRUE(diagnostics.has_completed_interval);
    EXPECT_TRUE(diagnostics.accepted_force_assembled);
    EXPECT_EQ(diagnostics.parent_count[0],1u);
    EXPECT_EQ(diagnostics.parent_count[1],1u);
    EXPECT_EQ(diagnostics.parent_count[2],1u);
    EXPECT_EQ(rig.batch.allocations().device_bytes,allocations.device_bytes);
  }
}
} // namespace solid_resident_test
