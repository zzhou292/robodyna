#include "../resident_shell_tab1/CudaFixture.h"

namespace resident_tab1_test {
namespace {
__global__ void CollapseLastQuad(fe::NodalPreparedView view) {
  constexpr unsigned first=7*(Parents-1);
  for(unsigned node=1;node<4;++node) for(unsigned axis=0;axis<3;++axis)
    const_cast<double*>(view.kinematics.position_xyz)[3*(first+node)+axis]=
        view.kinematics.position_xyz[3*first+axis];
}
}
TEST_F(Cuda,QephPrivatePartialTrialFailurePreservesCommonOwnerAndRetriesExactly) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize(Placement::TopReferencePlane));
  ASSERT_TRUE(rig.Bind());
  ASSERT_EQ(rig.publication.Initialize(rig.owner,rig.qeph,rig.t3).status,fe::ShellPublicationStatus::Success);
  // A genuine accepted interval supplies noninitial material/force histories.
  Prepared initial;
  Frame initial_candidate;
  ASSERT_TRUE(Prepare(rig,initial));
  ASSERT_TRUE(Evaluate(rig,initial,initial_candidate));
  ASSERT_TRUE(Commit(rig,initial,initial_candidate));
  Frame accepted;
  ASSERT_TRUE(Read(rig,accepted));
  temporal::Snapshot before;
  ASSERT_TRUE(temporal::Read(rig.owner,before));
  Prepared clean;
  Frame expected;
  ASSERT_TRUE(Prepare(rig,clean));
  ASSERT_TRUE(Evaluate(rig,clean,expected));
  rig.Discard();
  Prepared bad;
  ASSERT_TRUE(Prepare(rig,bad));
  CollapseLastQuad<<<1,1,0,bad.view.stream>>>(bad.view);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  q::BatchDiagnostics diagnostics;
  const auto rejected=rig.qeph.EvaluateCandidate(bad.view,&diagnostics);
  EXPECT_EQ(rejected.status,q::BatchStatus::ElementFailure);
  EXPECT_EQ(rejected.element,Parents-1);
  rig.Discard();
  Frame held;
  ASSERT_TRUE(Read(rig,held));
  Same(held,accepted);
  temporal::Snapshot after;
  ASSERT_TRUE(temporal::Read(rig.owner,after));
  temporal::SameState(before,after);
  Prepared retry;
  Frame actual;
  ASSERT_TRUE(Prepare(rig,retry));
  ASSERT_TRUE(Evaluate(rig,retry,actual));
  Same(actual,expected);
  ASSERT_TRUE(Commit(rig,retry,actual));
}
} // namespace resident_tab1_test
