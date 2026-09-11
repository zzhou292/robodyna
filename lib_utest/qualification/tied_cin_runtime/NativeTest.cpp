#include "NativeFixture.h"

namespace cin_runtime_test {
namespace {
void ThreeCurrentGeometryIntervals(bool shared_masters) {
  SCOPED_TRACE(shared_masters);
  Fixture f;
  f.inertia[f.rows.back().secondary] = 0;
  if (shared_masters) {
    // Synthetic repeated association exercises interactions between NSV rows.
    // The other case retains the distinct Q4 and repeated-slot T3 patches.
    std::copy(f.rows.front().masters, f.rows.front().masters+4, f.rows.back().masters);
  }
  NativeState native(f);
  f.saved_mass = native.saved_mass;
  f.saved_inertia = native.saved_inertia;
  const auto initial_load = f.load;
  const auto initial_stif = f.stif;
  const auto initial_stifr = f.stifr;
  const auto initial_mass = f.mass;
  const auto initial_inertia = f.inertia;
  constexpr double dt = 1e-4;
  for (unsigned step = 0; step < 3; ++step) {
    SCOPED_TRACE(step);
    for (std::size_t i = 0; i < f.load.size(); ++i) f.load[i] = initial_load[i]*(step+1);
    f.stif = initial_stif;
    f.stifr = initial_stifr;
    const double kick = step ? dt : dt/2;
    ASSERT_EQ(native.Step(f, step*dt, dt, kick), 0);
    ASSERT_TRUE(cin::PrepareForceTrial(f.View(), f.Force()));
    for (std::size_t node = 0; node < f.mass.size(); ++node) {
      if (f.dependent[node]) continue;
      for (unsigned axis = 0; axis < 3; ++axis) {
        const auto i = 3*node+axis;
        f.a[i] = (1/f.mass[node])*f.load[axis*f.mass.size()+node];
        f.ar[i] = (1/f.inertia[node])*f.load[(axis+3)*f.mass.size()+node];
        f.velocity[i] = f.velocity[i]+kick*f.a[i];
        f.omega[i] = f.omega[i]+kick*f.ar[i];
      }
    }
    ASSERT_TRUE(cin::RecoverMotionTrial(f.View(), f.Motion()));
    CompareValues(f, native);
    for (std::size_t row = 0; row < f.rows.size(); ++row) {
      EXPECT_EQ(f.saved_mass[row], initial_mass[f.rows[row].secondary]);
      EXPECT_EQ(f.saved_inertia[row], initial_inertia[f.rows[row].secondary]);
    }
    for (std::size_t i = 0; i < f.x.size(); ++i) f.x[i] = f.x[i]+dt*f.velocity[i];
    native.Drift(dt);
  }
  EXPECT_NE(f.x, Fixture{}.x);
  EXPECT_EQ(native.saved_inertia.back(), 0);
  // Full I2FORCES is executed, including its unconditional diagnostic history.
  // Its normal+tangent integral must retain the applied secondary impulse.
  for (unsigned axis = 0; axis < 3; ++axis) {
    double impulse = 0;
    for (const auto& row : f.rows) impulse += 6*dt*initial_load[axis*f.mass.size()+row.secondary];
    tied_patch_test::Near(native.force_integral[axis]+native.force_integral[axis+3], impulse);
  }
}

} // namespace
TEST(CinRuntimeNative, CompleteForceMotionAndLiteralSeedCarryThreeCurrentGeometryIntervals) {
  ThreeCurrentGeometryIntervals(false);
  ThreeCurrentGeometryIntervals(true);
}

TEST(CinRuntimeNative, NativeLateOverflowAndMalformedRankPreserveEveryExposedDestination) {
  Fixture f;
  NativeState native(f);
  ASSERT_EQ(native.Step(f, 0, 1e-6, .5e-6), 0);
  native.couple[3*f.rows.back().secondary+2] = std::numeric_limits<double>::max();
  const auto before = native;
  EXPECT_EQ(native.Call(1e-6, 1e-6, 1e-6), 2);
  EXPECT_EQ(native.force, before.force);
  EXPECT_EQ(native.couple, before.couple);
  EXPECT_EQ(native.mass, before.mass);
  EXPECT_EQ(native.inertia, before.inertia);
  EXPECT_EQ(native.stif, before.stif);
  EXPECT_EQ(native.stifr, before.stifr);
  EXPECT_EQ(native.saved_mass, before.saved_mass);
  EXPECT_EQ(native.saved_inertia, before.saved_inertia);
  EXPECT_EQ(native.numerical_mass, before.numerical_mass);
  EXPECT_EQ(native.force_integral, before.force_integral);
  EXPECT_EQ(native.dpara, before.dpara);
  EXPECT_EQ(native.velocity, before.velocity);
  EXPECT_EQ(native.omega, before.omega);
  EXPECT_EQ(native.acceleration, before.acceleration);
  EXPECT_EQ(native.angular_acceleration, before.angular_acceleration);
  native.masters.back() = native.n+1;
  EXPECT_EQ(native.Call(1e-6, 1e-6, 1e-6), 1);
  EXPECT_EQ(native.mass, before.mass);
  native = before;
  native.couple[3*f.rows.back().secondary+2] = 0;
  EXPECT_EQ(native.Call(1e-6, 1e-6, 1e-6), 0);
}

TEST(CinRuntimeNative, CompleteNoReleaseCheckAcceptsCoincidentActiveAndRepeatedTriangleWitnesses) {
  Fixture f;
  NativeState native(f);
  std::vector<int> quads(native.masters.begin(), native.masters.begin()+4);
  quads.insert(quads.end(), native.masters.begin(), native.masters.begin()+4);
  const std::vector<int> triangles(native.masters.begin()+4, native.masters.begin()+7);
  std::vector<int> active{0, 1, 1}, node_active(native.n, 1), signed_nodes(native.r, -88);
  std::vector<double> mass(native.n, 0), inertia(native.n, 0);
  int status = -1;
  auto call = [&] {
    tl_cin_native_witness(native.n, native.r, 2, 1, native.masters.data(), native.secondary.data(),
        quads.data(), triangles.data(), active.data(), node_active.data(), native.saved_mass.data(),
        native.saved_inertia.data(), signed_nodes.data(), mass.data(), inertia.data(), &status);
    ASSERT_EQ(status, 0);
  };
  call();
  EXPECT_EQ(signed_nodes, native.secondary);
  EXPECT_EQ(mass, std::vector<double>(native.n, 0));
  // The declared first shell is inactive; its exact active coincident layer
  // remains a sufficient native witness. All ITAG2 values in the oracle are 1.
  active[1] = 0;
  call();
  EXPECT_EQ(signed_nodes[0], -native.secondary[0]);
  EXPECT_EQ(signed_nodes[1], native.secondary[1]);
  EXPECT_EQ(mass[f.rows[0].secondary], native.saved_mass[0]);
  active[1] = 1;
  const auto original_fourth = quads[7];
  quads[7] = native.secondary[1];
  call();
  EXPECT_EQ(signed_nodes[0], -native.secondary[0]);
  quads[7] = original_fourth;
  node_active[f.rows.back().masters[2]] = 0;
  call();
  EXPECT_EQ(signed_nodes[1], -native.secondary[1]);
  EXPECT_EQ(inertia[f.rows.back().secondary], native.saved_inertia.back());
  // Production intentionally rejects this pending/released domain; it does
  // not copy the native release branch into its admitted no-release stage.
  f.flags.back() = 2;
  EXPECT_EQ(cin::PrepareForceTrial(f.View(), f.Force()).status, cin::StageStatus::PendingReleaseEligibility);
}
} // namespace cin_runtime_test
