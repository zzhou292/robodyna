#include "FailureResidentFixture.h"
#include "lib_src/elements/failure/ShellFailureArenaLayout.h"
namespace resident_failure_test {
TEST_F(Cuda,CompleteMixedParentsRemoveThenRetainPostRemovalHistoryAndOneClock) {
  Rig rig;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  ASSERT_TRUE(Initialize(rig,catalog,failure));
  ASSERT_TRUE(rig.Bind());
  const auto qalloc=rig.qeph.allocations();
  const auto talloc=rig.t3.allocations();
  EXPECT_EQ(qalloc.device_allocations,3u);
  EXPECT_EQ(talloc.device_allocations,3u);
  Frame old;
  ASSERT_TRUE(Read(rig,old));
  for(unsigned e=0;e<Parents;++e) {
    EXPECT_TRUE(old.qfailure[e].active);
    EXPECT_TRUE(old.tfailure[e].active);
  }
  for(unsigned step=0;step<3;++step) {
    Prepared prepared;
    ASSERT_TRUE(mixed::Prepare(rig,mixed::PlasticSchedule(rig,0),prepared));
    Frame next;
    ASSERT_TRUE(Evaluate(rig,prepared,next));
    CheckForceAdapters(rig,catalog,prepared,old,next);
    EXPECT_TRUE(next.qfailure[0].active);
    EXPECT_TRUE(next.tfailure[0].active);
    EXPECT_FALSE(next.qfailure[1].active);
    EXPECT_FALSE(next.tfailure[1].active);
    EXPECT_EQ(next.qfailure[0].policy,fe::ShellFailurePolicy::None);
    EXPECT_EQ(next.qfailure[1].policy,fe::ShellFailurePolicy::ConstantAllPoints);
    if(step) {
      EXPECT_NE(next.material.qforce[1].proposed_history.data().strain_curvature[0],old.material.qforce[1].proposed_history.data().strain_curvature[0]);
      EXPECT_NE(next.material.tforce[1].proposed_history.data().strain_curvature[0],old.material.tforce[1].proposed_history.data().strain_curvature[0]);
    }
    Frame held;
    ASSERT_TRUE(Read(rig,held));
    Same(held,old);
    ASSERT_TRUE(mixed::Commit(rig,prepared,next.material));
    Frame accepted;
    ASSERT_TRUE(Read(rig,accepted));
    Same(accepted,next);
    EXPECT_EQ(rig.owner.accepted().epoch,step+1);
    EXPECT_EQ(rig.qeph.allocations().device_bytes,qalloc.device_bytes);
    EXPECT_EQ(rig.t3.allocations().device_bytes,talloc.device_bytes);
    old=accepted;
  }
}
TEST_F(Cuda,DefaultOffKeepsMixedLayoutAndWholeOtherFamilyFailureScopeCannotChange) {
  Rig disabled,enabled;
  fe::ShellBatchPlasticityBinding dc,ec;
  fe::ShellBatchFailureBinding df,ef;
  ASSERT_TRUE(Initialize(disabled,dc,df,false));
  ASSERT_TRUE(Initialize(enabled,ec,ef,true,false,1.));
  ASSERT_TRUE(disabled.Bind());
  ASSERT_TRUE(enabled.Bind());
  storage::FailureLayout layout;
  ASSERT_TRUE(layout.Initialize(Parents,fe::ShellBatchFailureLimits{}.max_device_bytes));
  EXPECT_EQ(disabled.qeph.allocations().device_allocations,2u);
  EXPECT_EQ(disabled.t3.allocations().device_allocations,2u);
  EXPECT_EQ(enabled.qeph.allocations().device_bytes-disabled.qeph.allocations().device_bytes,layout.bytes);
  EXPECT_EQ(enabled.t3.allocations().device_bytes-disabled.t3.allocations().device_bytes,layout.bytes);
  Frame off,on;
  ASSERT_TRUE(mixed::ReadFrame(disabled,off.material));
  ASSERT_TRUE(Read(enabled,on));
  off.qfailure=on.qfailure;
  off.tfailure=on.tfailure;
  Same(off,on);
  for (unsigned step = 0; step < 3; ++step) {
    Prepared off_prepared, on_prepared;
    ASSERT_TRUE(mixed::Prepare(disabled,mixed::PlasticSchedule(disabled,0),off_prepared));
    ASSERT_TRUE(mixed::Prepare(enabled,mixed::PlasticSchedule(enabled,0),on_prepared));
    ASSERT_TRUE(mixed::Evaluate(disabled,off_prepared,off.material));
    ASSERT_TRUE(Evaluate(enabled,on_prepared,on));
    EXPECT_TRUE(on.qfailure[1].active);
    EXPECT_TRUE(on.tfailure[1].active);
    off.qfailure = on.qfailure;
    off.tfailure = on.tfailure;
    Same(off,on);
    ASSERT_TRUE(mixed::Commit(disabled,off_prepared,off.material));
    ASSERT_TRUE(mixed::Commit(enabled,on_prepared,on.material));
  }
  qe::BatchDiagnostics diagnostic;
  EXPECT_EQ(disabled.qeph.CopyAcceptedFailureHistory(disabled.owner.accepted(),off.qfailure.data(),Parents,&diagnostic).status,qe::BatchStatus::InvalidInput);
  Rig mismatch;
  fe::ShellBatchPlasticityBinding mc;
  fe::ShellBatchFailureBinding mf;
  ASSERT_TRUE(Initialize(mismatch,mc,mf,true,true));
  ASSERT_TRUE(mixed::AssembleCollectionForBinding(mismatch));
  EXPECT_EQ(mismatch.publication.Initialize(mismatch.owner,mismatch.qeph,mismatch.t3).status,fe::ShellPublicationStatus::InvalidInput);
}
namespace {
__global__ void CollapseLastT3(fe::NodalPreparedView view) {
  for(unsigned axis=0;axis<3;++axis)
    const_cast<double*>(view.kinematics.position_xyz)[3*4+axis]=(view.kinematics.position_xyz[axis]+view.kinematics.position_xyz[3*2+axis])*0.5;
}
}
TEST_F(Cuda,LateInactiveParentGeometryRejectionRetainsEveryAcceptedFieldAndRetriesExactly) {
  Rig rig;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  ASSERT_TRUE(Initialize(rig,catalog,failure));
  ASSERT_TRUE(rig.Bind());
  Prepared first;
  ASSERT_TRUE(mixed::Prepare(rig,mixed::PlasticSchedule(rig,0),first));
  Frame removed;
  ASSERT_TRUE(Evaluate(rig,first,removed));
  ASSERT_TRUE(mixed::Commit(rig,first,removed.material));
  Frame old;
  ASSERT_TRUE(Read(rig,old));
  ASSERT_FALSE(old.tfailure[1].active);
  mixed::Snapshot state;
  ASSERT_TRUE(mixed::Read(rig.owner,state));
  Prepared clean;
  ASSERT_TRUE(mixed::Prepare(rig,mixed::PlasticSchedule(rig,0),clean));
  Frame expected;
  ASSERT_TRUE(Evaluate(rig,clean,expected));
  rig.Discard();
  Prepared rejected;
  ASSERT_TRUE(mixed::Prepare(rig,mixed::PlasticSchedule(rig,0),rejected));
  qe::BatchDiagnostics qd;
  ASSERT_EQ(rig.qeph.EvaluateCandidate(rejected.view,&qd).status,qe::BatchStatus::Success);
  CollapseLastT3<<<1,1,0,rejected.view.stream>>>(rejected.view);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  tr::BatchDiagnostics td;
  const auto failed=rig.t3.EvaluateCandidate(rejected.view,&td);
  EXPECT_EQ(failed.status,tr::BatchStatus::ElementFailure);
  EXPECT_EQ(failed.element,Parents-1);
  rig.Discard();
  Frame held;
  ASSERT_TRUE(Read(rig,held));
  Same(held,old);
  mixed::Snapshot after;
  ASSERT_TRUE(mixed::Read(rig.owner,after));
  mixed::SameState(state,after);
  Prepared retry;
  ASSERT_TRUE(mixed::Prepare(rig,mixed::PlasticSchedule(rig,0),retry));
  Frame actual;
  ASSERT_TRUE(Evaluate(rig,retry,actual));
  Same(actual,expected);
  ASSERT_TRUE(mixed::Commit(rig,retry,actual.material));
}
} // namespace resident_failure_test
