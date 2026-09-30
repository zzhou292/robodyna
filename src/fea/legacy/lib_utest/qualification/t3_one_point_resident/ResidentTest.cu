#include "ResidentFixture.h"
#include "lib_utest/qualification/native/t3/T3Reference.h"

namespace t3_one_point_resident_test {
TEST_F(Cuda, OriginalTriangleMixedResidentRetainsIndependentNativePointHistoryAndOneSelector) {
  Rig rig;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  ASSERT_TRUE(Initialize(rig, catalog, failure));
  ASSERT_TRUE(rig.Bind());
  const auto allocations = rig.t3.allocations();
  EXPECT_EQ(allocations.device_allocations, 4u);
  EXPECT_EQ(rig.qeph.allocations().device_allocations, 3u);
  EXPECT_EQ(rig.binding.t3_source_id(1), 2357656u);
  Frame old;
  ASSERT_TRUE(mixed::ReadFrame(rig, old));
  ASSERT_NE(old.tsection[0].elastic(), nullptr);
  ASSERT_NE(old.tsection[1].one_point(), nullptr);
  EXPECT_EQ(old.tsection[1].plastic(), nullptr);
  EXPECT_EQ(old.tsection[1].one_point()->point.saved.plastic_strain, 0);
  EXPECT_EQ(old.tsection[1].one_point()->point.reported_thickness_m, .0005);
  // Both participants authenticate the actual owner's immutable combined M/J.
  // Check every copied device coefficient against once-only ordered reference sums.
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  ASSERT_EQ(rig.owner.BeginTrial(&token, &view).status, fe::NodalStatus::Ok);
  double inverse[Nodes]{}, inertia[Nodes]{}, mass[Nodes]{}, total_j[Nodes]{};
  ASSERT_EQ(cudaMemcpy(inverse, view.mass.inverse_mass, sizeof(inverse), cudaMemcpyDeviceToHost), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(inertia, view.inverse_inertia, sizeof(inertia), cudaMemcpyDeviceToHost), cudaSuccess);
  for (unsigned e = 0; e < Parents; ++e) {
    const auto& reference = rig.binding.qeph_reference(e);
    for (unsigned n = 0; n < 4; ++n) {
      const auto global = rig.binding.qeph_nodes(e)[n];
      mass[global] += reference.nodal_mass[n];
      total_j[global] += reference.isotropic_inertia[n];
    }
  }
  for (unsigned e = 0; e < Parents; ++e) {
    const auto& reference = rig.binding.t3_reference(e);
    for (unsigned n = 0; n < 3; ++n) {
      const auto global = rig.binding.t3_nodes(e)[n];
      mass[global] += reference.nodal_mass[n];
      total_j[global] += reference.isotropic_inertia[n];
    }
  }
  for (unsigned n = 0; n < Nodes; ++n) {
    EXPECT_DOUBLE_EQ(inverse[n], 1 / mass[n]);
    EXPECT_DOUBLE_EQ(inertia[n], 1 / total_j[n]);
  }
  rig.Discard();
  pure::Fixture oracle;
  pure::NativeState native(oracle.Virgin());
  bool yielded = false, unloaded = false;
  for (unsigned step = 0; step < 64; ++step) {
    Prepared prepared;
    ASSERT_TRUE(Prepare(rig, step, prepared));
    auto next_native = native;
    ASSERT_EQ(next_native.Step(oracle, Interval(rig, prepared)), 0);
    Frame next;
    ASSERT_TRUE(mixed::Evaluate(rig, prepared, next));
    NativeAgreement(old, next, oracle, next_native);
    ASSERT_FALSE(::testing::Test::HasFailure());
    const auto& point = next.tsection[1].one_point()->point;
    yielded = yielded || point.saved.plastic_strain > 0;
    unloaded = unloaded || (yielded && point.current.plastic_increment == 0);
    Frame held;
    ASSERT_TRUE(mixed::ReadFrame(rig, held));
    Same(old, held);
    ASSERT_TRUE(mixed::Commit(rig, prepared, next));
    Frame accepted;
    ASSERT_TRUE(mixed::ReadFrame(rig, accepted));
    Same(accepted, next);
    EXPECT_EQ(rig.owner.accepted().epoch, step + 1);
    EXPECT_EQ(rig.t3.allocations().device_bytes, allocations.device_bytes);
    EXPECT_EQ(rig.t3.allocations().device_allocations, allocations.device_allocations);
    native = next_native;
    old = accepted;
  }
  EXPECT_TRUE(yielded);
  EXPECT_TRUE(unloaded);
  EXPECT_GT(old.tsection[1].one_point()->cumulative_plastic_work_J, 0);
}

namespace {
__global__ void CollapseOriginalTriangle(fe::NodalPreparedView view) {
  for (unsigned axis = 0; axis < 3; ++axis)
    const_cast<double*>(view.kinematics.position_xyz)[3*4+axis] =
        .5 * (view.kinematics.position_xyz[axis] + view.kinematics.position_xyz[3*2+axis]);
}
}
TEST_F(Cuda, OnePointRemovalAndLateGeometryFailureRetainNativeWorkAndExactRetry) {
  Rig rig;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  constexpr double d1 = 1e-6; // Explicit failure-control fixture, not original source D1.
  ASSERT_TRUE(Initialize(rig, catalog, failure, true, d1));
  ASSERT_TRUE(rig.Bind());
  pure::Fixture oracle;
  oracle.failure.failure_strain = d1;
  pure::NativeState native(oracle.Virgin());
  Frame old;
  ASSERT_TRUE(mixed::ReadFrame(rig, old));
  bool removed = false;
  unsigned step = 0;
  for (; step < 16; ++step) {
    Prepared prepared;
    ASSERT_TRUE(Prepare(rig, step, prepared));
    ASSERT_EQ(native.Step(oracle, Interval(rig, prepared)), 0);
    Frame next;
    ASSERT_TRUE(mixed::Evaluate(rig, prepared, next));
    NativeAgreement(old, next, oracle, native);
    ASSERT_FALSE(::testing::Test::HasFailure());
    removed = next.tsection[1].one_point()->point.failure.failed_now;
    ASSERT_TRUE(mixed::Commit(rig, prepared, next));
    old = next;
    if (removed) { ++step; break; }
  }
  ASSERT_TRUE(removed);
  const auto& point = old.tsection[1].one_point()->point;
  EXPECT_GT(point.current.equivalent_stress_pa, 0);
  EXPECT_GT(old.tsection[1].one_point()->cumulative_plastic_work_J, 0);
  for (double stress : point.saved.stress) EXPECT_EQ(stress, 0);
  for (const auto force : old.tforce[1].internal_force) {
    EXPECT_EQ(force.x, 0);
    EXPECT_EQ(force.y, 0);
    EXPECT_EQ(force.z, 0);
  }
  Prepared clean;
  ASSERT_TRUE(Prepare(rig, step, clean));
  Frame expected;
  ASSERT_TRUE(mixed::Evaluate(rig, clean, expected));
  auto next_native = native;
  ASSERT_EQ(next_native.Step(oracle, Interval(rig, clean)), 0);
  NativeAgreement(old, expected, oracle, next_native);
  EXPECT_FALSE(expected.tsection[1].one_point()->point.failure.failed_now);
  rig.Discard();
  temporal::Snapshot before;
  ASSERT_TRUE(temporal::Read(rig.owner, before));
  Prepared invalid;
  ASSERT_TRUE(Prepare(rig, step, invalid));
  CollapseOriginalTriangle<<<1,1,0,invalid.view.stream>>>(invalid.view);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  t3::BatchDiagnostics rejected;
  const auto report = rig.t3.EvaluateCandidate(invalid.view, &rejected);
  EXPECT_EQ(report.status, t3::BatchStatus::ElementFailure);
  EXPECT_EQ(report.element, 1u);
  rig.Discard();
  temporal::Snapshot after;
  ASSERT_TRUE(temporal::Read(rig.owner, after));
  temporal::SameState(before, after);
  Frame held;
  ASSERT_TRUE(mixed::ReadFrame(rig, held));
  Same(old, held);
  Prepared retry;
  ASSERT_TRUE(Prepare(rig, step, retry));
  Frame actual;
  ASSERT_TRUE(mixed::Evaluate(rig, retry, actual));
  Same(expected, actual);
  ASSERT_TRUE(mixed::Commit(rig, retry, actual));
}
} // namespace t3_one_point_resident_test
