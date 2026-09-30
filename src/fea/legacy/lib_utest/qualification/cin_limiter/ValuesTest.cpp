// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../cin_physical_timestep/Fixture.h"
#include "lib_src/solvers/cin_limiter/Capture.h"
#include "lib_src/solvers/FENodalStateStorage.h"
#include "lib_src/solvers/nodal_seal/RowLayout.h"
#include <gtest/gtest.h>

namespace cin_limiter_test {
namespace fe = tl::fea;
namespace dt = fe::cin_timestep;
using Kind = fe::NodalCinLimitKind;
TEST(CinLimiterValues, OrdinaryAxisTieOrderAndUnboundedRemainExact) {
  cin_step_test::Fixture f;
  f.translation.fill(0);
  f.rotation.fill(0);
  f.translation[0] = 4;
  f.rotation[0] = 6;
  dt::Result result;
  std::uint32_t bad = UINT32_MAX;
  ASSERT_TRUE(dt::Screen(f.View(), .8, result, bad));
  auto witness = fe::cin_limiter::Capture(f.View(), .8, result, 3, 5);
  EXPECT_EQ(witness.values.kind, Kind::OrdinaryTranslation);
  EXPECT_EQ(witness.values.node, 0u);
  EXPECT_EQ(witness.values.mass_kg, 2);
  EXPECT_EQ(witness.values.inertia_kg_m2, 3);
  EXPECT_EQ(witness.values.minimum_dt_s, result.minimum_dt);
  EXPECT_EQ(witness.epoch, 3u);
  EXPECT_EQ(witness.attempt, 5u);
  f.rotation[0] = 24;
  ASSERT_TRUE(dt::Screen(f.View(), .8, result, bad));
  witness = fe::cin_limiter::Capture(f.View(), .8, result, 3, 5);
  EXPECT_EQ(witness.values.kind, Kind::OrdinaryRotation);
  EXPECT_EQ(witness.values.rotation_stiffness_nm, 24);
  f.translation.fill(0);
  f.rotation.fill(0);
  ASSERT_TRUE(dt::Screen(f.View(), .8, result, bad));
  witness = fe::cin_limiter::Capture(f.View(), .8, result, 3, 5);
  EXPECT_EQ(witness.values.kind, Kind::Unbounded);
  EXPECT_EQ(witness.values.node, UINT32_MAX);
}
TEST(CinLimiterValues, ActualRigidTraceAndSourceImmutability) {
  cin_step_test::Fixture f;
  const auto accepted = f.accepted;
  dt::Result result;
  std::uint32_t bad = UINT32_MAX;
  ASSERT_TRUE(dt::Screen(f.View(), .8, result, bad));
  const auto witness = fe::cin_limiter::Capture(f.View(), .8, result, 4, 9);
  EXPECT_EQ(witness.values.kind, Kind::RigidTrace);
  EXPECT_EQ(witness.values.group, result.limiting_group);
  EXPECT_EQ(witness.values.node, result.limiting_node);
  EXPECT_EQ(witness.values.mass_kg, f.groups[result.limiting_group].mass);
  dt::ScalarLimit checked;
  ASSERT_TRUE(dt::RigidTraceLimit(witness.values.trace_upper_per_s2, .8, checked));
  EXPECT_EQ(checked.dt, result.minimum_dt);
  EXPECT_EQ(f.accepted, accepted);
  result.minimum_dt = std::nextafter(result.minimum_dt, 1.0);
  EXPECT_EQ(fe::cin_limiter::Capture(f.View(), .8, result, 4, 9).values.kind, Kind::Unavailable);
}
TEST(CinLimiterValues, InvalidDiagnosticNeverChangesScreenOrAdmitsUnrequestedProfile) {
  EXPECT_FALSE(fe::ValidCinStructuralStep({fe::NodalCinStructuralProfile::Disabled, 0, true}));
  EXPECT_TRUE(fe::ValidCinStructuralStep({fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8, true}));
  cin_step_test::Fixture f;
  dt::Result result;
  std::uint32_t bad = UINT32_MAX;
  ASSERT_TRUE(dt::Screen(f.View(), .8, result, bad));
  const auto before = result;
  EXPECT_EQ(fe::cin_limiter::Capture(f.View(), .8, result, 0, 0).values.kind, Kind::Unavailable);
  EXPECT_EQ(result.minimum_dt, before.minimum_dt);
  EXPECT_EQ(result.limiting_node, before.limiting_node);
  EXPECT_EQ(result.limiting_group, before.limiting_group);
}
TEST(CinLimiterValues, ForecastChargesFixedControlGrowthIncludingSealAlignment) {
  struct OldControl {
    fe::stability::RowBounds rows;
    fe::NodalAssemblyResult assembly;
    fe::stability::StepLimit limit;
    fe::NodalStatus status = fe::NodalStatus::Ok;
    std::uint32_t node = UINT32_MAX;
  };
  using Control = fe::nodal_detail::Control;
  EXPECT_EQ(offsetof(Control, structural_limiter), sizeof(OldControl));
  EXPECT_EQ(sizeof(Control) - sizeof(OldControl), sizeof(fe::cin_limiter::Witness));
  EXPECT_GT(sizeof(fe::cin_limiter::Witness), 0u);
  EXPECT_LE(sizeof(fe::cin_limiter::Witness), 128u);
  for (const std::size_t nodes : {8u, 2049u, 376930u}) {
    EXPECT_EQ(fe::nodal_seal::ControlBytes(sizeof(Control), nodes) -
        fe::nodal_seal::ControlBytes(sizeof(OldControl), nodes), sizeof(fe::cin_limiter::Witness));
  }
}
} // namespace cin_limiter_test
