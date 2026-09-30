// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "Assertions.h"
#include <type_traits>
namespace type25_normal_test {
static_assert(!std::is_same_v<normal::NativeNormalInput, normal::SiNormalInput>);
namespace {
normal::SiNormalInput Si(const normal::NativeNormalInput& in, normal::UnitScale units) {
  normal::SiNormalInput out;
  const double force = units.mass_kg * units.length_m / (units.time_s * units.time_s);
  out.penetration = in.penetration * units.length_m;
  out.stiffness = in.stiffness * (force / units.length_m);
  out.normal_velocity = in.normal_velocity * (units.length_m / units.time_s);
  out.dt = in.dt * units.time_s; out.time = in.time * units.time_s;
  out.secondary_mass = in.secondary_mass * units.mass_kg;
  for (unsigned i = 0; i < 4; ++i) {
    out.main_mass[i] = in.main_mass[i] * units.mass_kg;
    out.weights[i] = in.weights[i];
  }
  out.friction_viscosity = in.friction_viscosity;
  return out;
}
}
TEST(Type25NormalUnits, NativeMmSecondTonneHasIndependentPhysicalUnitsAndOracle) {
  const normal::UnitScale units{0.001, 1000., 1.};
  const auto c = Basic();
  const auto input = Si(c.input, units);
  normal::SiNormalResult actual;
  ASSERT_EQ(normal::EvaluateSiNormal(c.config, units, input, {}, &actual), normal::NormalStatus::Ok);
  const auto native = Oracle(c.config, c.input, c.history);
  EXPECT_NEAR(actual.normal_force, native.normal_force, std::abs(native.normal_force) * 1e-14);
  EXPECT_NEAR(actual.elastic_energy, native.elastic_energy * 0.001, native.elastic_energy * 1e-16);
  EXPECT_NEAR(actual.force_stiffness, native.force_stiffness * 1000., native.force_stiffness * 1e-11);
  EXPECT_NEAR(actual.damping_coefficient, native.damping_coefficient * 1000., native.damping_coefficient * 1e-11);
  EXPECT_DOUBLE_EQ(actual.history.staged_penetration, native.history.staged_penetration * 0.001);
}
TEST(Type25NormalUnits, NativeEppThresholdIsNotMistakenForMetres) {
  auto c = Basic(); c.input.time = 1; c.input.dt = 0;
  c.config.damping_factor = 0; c.input.normal_velocity = 0;
  c.history = {0.001, 300., 0, 0, 0};
  c.input.penetration = c.history.previous_penetration + 2 * normal::native_constant::epp;
  const normal::UnitScale units{0.001, 1000., 1.};
  const auto input = Si(c.input, units);
  const normal::SiNormalHistory history{1e-6, 300000., 0, 0, 0};
  normal::SiNormalResult actual;
  ASSERT_EQ(normal::EvaluateSiNormal(c.config, units, input, history, &actual), normal::NormalStatus::Ok);
  // Native extra penetration is2e-10mm=2e-13m, above nativeEPP=1e-13m.
  // Treating EPP as1e-10m would incorrectly retain exactly300000N/m.
  EXPECT_LT(actual.history.staged_stiffness, 300000.);
  const auto native = Oracle(c.config, c.input, c.history);
  EXPECT_NEAR(actual.history.staged_stiffness, native.history.staged_stiffness * 1000., 1e-8);
}
TEST(Type25NormalUnits, ExplicitDyadicScalesRoundTripAllDimensionsAndRejectInvalidScales) {
  const auto c = Basic();
  for (auto units : {normal::UnitScale{1, 1, 1}, normal::UnitScale{0.125, 8, 0.5}}) {
    const auto input = Si(c.input, units);
    normal::SiNormalResult actual;
    ASSERT_EQ(normal::EvaluateSiNormal(c.config, units, input, {}, &actual), normal::NormalStatus::Ok);
    const auto native = Oracle(c.config, c.input, c.history);
    const double force = units.mass_kg * units.length_m / (units.time_s * units.time_s);
    EXPECT_DOUBLE_EQ(actual.normal_force, native.normal_force * force);
    EXPECT_DOUBLE_EQ(actual.damping_work, native.damping_work * force * units.length_m);
  }
  normal::SiNormalResult prior; prior.normal_force = 123.;
  for (auto units : {normal::UnitScale{}, normal::UnitScale{-1, 1, 1},
                    normal::UnitScale{1, 1, std::numeric_limits<double>::infinity()}}) {
    auto actual = prior;
    EXPECT_EQ(normal::EvaluateSiNormal(c.config, units, {}, {}, &actual), normal::NormalStatus::InvalidInput);
    Same(actual, prior, true);
  }
}
} // namespace type25_normal_test
