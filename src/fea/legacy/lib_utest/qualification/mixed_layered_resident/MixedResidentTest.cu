#include "MixedResidentFixture.h"

namespace mixed_layered_test {
TEST_F(MixedShellCuda, MixedLayeredLawsYieldAndUnloadWithOnePublicationAndStableStorage) {
  Rig r;fe::ShellBatchSectionBinding catalog;ASSERT_TRUE(Initialize(r,catalog));ASSERT_TRUE(r.Bind());
  const auto qa=r.qeph.allocations(),ta=r.t3.allocations();
  EXPECT_EQ(qa.device_allocations,2u);EXPECT_EQ(ta.device_allocations,2u);
  Frame old;ASSERT_TRUE(ReadFrame(r,old));
  for(unsigned interval=0;interval<4;++interval) {
    Prepared p;ASSERT_TRUE(Prepare(r,PlasticSchedule(r,interval),p));
    Frame next;ASSERT_TRUE(Evaluate(r,p,next));CheckAdapters(r,catalog,p,old,next);
    Frame held;ASSERT_TRUE(ReadFrame(r,held));SameFrame(held,old);
    if(!interval) {
      ASSERT_NE(next.qsection[1].plastic(),nullptr);ASSERT_NE(next.tsection[0].plastic(),nullptr);
      EXPECT_GT(next.qsection[1].plastic()->diagnostics.maximum_plastic_strain,0);
      EXPECT_GT(next.tsection[0].plastic()->diagnostics.maximum_plastic_strain,0);
      EXPECT_GT(next.qsection[1].plastic()->history.point[0].filtered_rate_per_s,0);
      EXPECT_GT(next.qsection[1].plastic()->cumulative_plastic_work_J,0);
      EXPECT_NE(next.qforce[0].proposed_history.data().thickness,r.binding.qeph_reference(0).input.thickness);
      EXPECT_NE(next.qsection[0].elastic()->point[0].stress[0],next.qsection[1].plastic()->history.point[0].stress[0]);
    }
    ASSERT_FALSE(::testing::Test::HasFailure());ASSERT_TRUE(Commit(r,p,next));
    Frame accepted;ASSERT_TRUE(ReadFrame(r,accepted));SameFrame(accepted,next);old=accepted;
    EXPECT_EQ(r.owner.accepted().epoch,interval+1);
    EXPECT_EQ(r.qeph.allocations().device_bytes,qa.device_bytes);EXPECT_EQ(r.t3.allocations().device_bytes,ta.device_bytes);
    EXPECT_EQ(r.qeph.allocations().device_allocations,2u);EXPECT_EQ(r.t3.allocations().device_allocations,2u);
  }
}
namespace {
__global__ void CollapseLastTriangle(fe::NodalPreparedView v) {
  for(unsigned axis=0;axis<3;++axis)
    const_cast<double*>(v.kinematics.position_xyz)[3*4+axis]=v.kinematics.position_xyz[3+axis];
}
}
TEST_F(MixedShellCuda, MixedLayeredLateFamilyFailurePreservesYieldedStateAndRetriesExactly) {
  Rig r;fe::ShellBatchSectionBinding catalog;ASSERT_TRUE(Initialize(r,catalog));ASSERT_TRUE(r.Bind());
  Prepared first;ASSERT_TRUE(Prepare(r,PlasticSchedule(r,0),first));Frame initial;
  ASSERT_TRUE(Evaluate(r,first,initial));ASSERT_TRUE(Commit(r,first,initial));
  Snapshot state;ASSERT_TRUE(Read(r.owner,state));Frame old;ASSERT_TRUE(ReadFrame(r,old));
  ASSERT_GT(old.qsection[1].plastic()->diagnostics.maximum_plastic_strain,0);
  Prepared clean_p;ASSERT_TRUE(Prepare(r,PlasticSchedule(r,1),clean_p));Frame clean;
  ASSERT_TRUE(Evaluate(r,clean_p,clean));r.Discard();
  Prepared failed;ASSERT_TRUE(Prepare(r,PlasticSchedule(r,1),failed));q::BatchDiagnostics qd;
  ASSERT_EQ(r.qeph.EvaluateCandidate(failed.view,&qd).status,q::BatchStatus::Success);
  CollapseLastTriangle<<<1,1,0,failed.view.stream>>>(failed.view);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  t::BatchDiagnostics td;EXPECT_EQ(r.t3.EvaluateCandidate(failed.view,&td).status,t::BatchStatus::ElementFailure);
  r.Discard();Frame held;ASSERT_TRUE(ReadFrame(r,held));SameFrame(held,old);
  Snapshot after;ASSERT_TRUE(Read(r.owner,after));SameState(after,state);
  auto stale=held.qsection;
  EXPECT_EQ(r.qeph.CopyPreparedLayeredSectionHistory(qd,stale.data(),Parents).status,q::BatchStatus::StaleTrial);
  Prepared retry;ASSERT_TRUE(Prepare(r,PlasticSchedule(r,1),retry));Frame actual;
  ASSERT_TRUE(Evaluate(r,retry,actual));SameFrame(actual,clean);ASSERT_TRUE(Commit(r,retry,actual));
}
} // namespace mixed_layered_test
