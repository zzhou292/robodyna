#include "CudaFixture.h"

namespace resident_tab1_test {
namespace {
bool Ready(Rig& rig, Frame& accepted) {
  if (!rig.Initialize(Placement::TopReferencePlane) || !rig.Bind()) return false;
  if (rig.publication.Initialize(rig.owner, rig.qeph, rig.t3).status != fe::ShellPublicationStatus::Success) return false;
  if (!Read(rig, accepted)) return false;
  for (unsigned step = 0; step < 32 && accepted.tfailure.back().active; ++step) {
    Prepared prepared;
    Frame candidate;
    if (!Prepare(rig, prepared) || !Evaluate(rig, prepared, candidate) ||
        !Commit(rig, prepared, candidate) || !Read(rig, accepted)) return false;
  }
  return !accepted.tfailure.back().active;
}
__global__ void CollapseLastTriangle(fe::NodalPreparedView view) {
  constexpr unsigned first = 7 * (Parents - 1) + 4;
  for (unsigned axis = 0; axis < 3; ++axis) {
    const_cast<double*>(view.kinematics.position_xyz)[3 * (first + 2) + axis] =
        .5 * (view.kinematics.position_xyz[3 * first + axis] +
              view.kinematics.position_xyz[3 * (first + 1) + axis]);
  }
}
} // namespace
TEST_F(Cuda,LastRemovedParentGeometryFailurePreservesCompleteOwnerAndRetriesExactly) {
  Rig rig;
  Frame accepted;
  ASSERT_TRUE(Ready(rig, accepted));
  temporal::Snapshot state;
  ASSERT_TRUE(temporal::Read(rig.owner, state));
  Prepared clean;
  Frame expected;
  ASSERT_TRUE(Prepare(rig, clean));
  ASSERT_TRUE(Evaluate(rig, clean, expected));
  rig.Discard();
  Prepared bad;
  ASSERT_TRUE(Prepare(rig, bad));
  q::BatchDiagnostics qd;
  ASSERT_EQ(rig.qeph.EvaluateCandidate(bad.view, &qd).status, q::BatchStatus::Success);
  CollapseLastTriangle<<<1, 1, 0, bad.view.stream>>>(bad.view);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  t::BatchDiagnostics td;
  const auto rejected = rig.t3.EvaluateCandidate(bad.view, &td);
  EXPECT_EQ(rejected.status, t::BatchStatus::ElementFailure);
  EXPECT_EQ(rejected.element, Parents - 1);
  rig.Discard();
  Frame held;
  ASSERT_TRUE(Read(rig, held));
  Same(held, accepted);
  temporal::Snapshot after;
  ASSERT_TRUE(temporal::Read(rig.owner, after));
  temporal::SameState(state, after);
  Prepared retry;
  Frame actual;
  ASSERT_TRUE(Prepare(rig, retry));
  ASSERT_TRUE(Evaluate(rig, retry, actual));
  Same(actual, expected);
  ASSERT_TRUE(Commit(rig, retry, actual));
}
TEST_F(Cuda,ExactCapacityAndLateTypedReadbackFaultsKeepOutputsAndAcceptedHistoryAtomic) {
  Rig rig;
  Frame accepted;
  ASSERT_TRUE(Ready(rig, accepted));
  t::BatchDiagnostics diagnostics;
  Arm(ReadFault::NonfiniteDamage);
  EXPECT_EQ(rig.t3.CopyAcceptedFailureHistory(rig.owner.accepted(),
      reinterpret_cast<fe::ShellBatchFailureState*>(1), Parents - 1, &diagnostics).status,
      t::BatchStatus::ResourceLimit);
  EXPECT_EQ(Copies(), 0u);
  Arm(ReadFault::None);
  for (auto mode : {ReadFault::NonfiniteDamage, ReadFault::InvalidDisplayCap,
                    ReadFault::InvalidPolicy, ReadFault::InvalidFlag}) {
    Prepared clean;
    Frame expected;
    ASSERT_TRUE(Prepare(rig, clean));
    ASSERT_TRUE(Evaluate(rig, clean, expected));
    auto output = accepted.tfailure;
    Arm(mode);
    const auto report = rig.t3.CopyPreparedFailureHistory(expected.diagnostics.t3, output.data(), Parents);
    EXPECT_NE(report.status, t::BatchStatus::Success);
    EXPECT_EQ(Copies(), 1u);
    for (unsigned e = 0; e < Parents; ++e) placed::Exact(Values(output[e]), Values(accepted.tfailure[e]));
    rig.Discard();
    Frame held;
    ASSERT_TRUE(Read(rig, held));
    Same(held, accepted);
    Prepared retry;
    Frame actual;
    ASSERT_TRUE(Prepare(rig, retry));
    ASSERT_TRUE(Evaluate(rig, retry, actual));
    Same(actual, expected);
    ASSERT_TRUE(Commit(rig, retry, actual));
    ASSERT_TRUE(Read(rig, accepted));
  }
}
} // namespace resident_tab1_test
