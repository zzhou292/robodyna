// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "Assertions.h"
#include "NativeOracle.h"
#include <array>
#include <cmath>
#include <limits>
#include <type_traits>

namespace type25_friction_test {
namespace {
namespace vector = tl::math::fixed3;
static_assert(!std::is_same_v<n::NativeFrictionInput, n::SiFrictionInput>);

// Independent dimensional expectations; do not use production UnitConversions.
struct Dimensions {
  double length, mass, time, velocity, force, stiffness, energy, damping, area, pressure;
  explicit Dimensions(n::UnitScale u)
      : length(u.length_m), mass(u.mass_kg), time(u.time_s),
        velocity(length / time), force(mass * length / (time * time)),
        stiffness(force / length), energy(force * length), damping(force / velocity),
        area(length * length), pressure(force / area) {}
};
n::SiNormalHistory History(const n::NativeNormalHistory& h, const Dimensions& u) {
  return {h.previous_penetration * u.length, h.previous_stiffness * u.stiffness,
          h.staged_penetration * u.length, h.staged_stiffness * u.stiffness,
          h.damping_half_force * u.force};
}
struct SiPacket {
  n::SiFrictionCoefficients coefficients;
  n::SiFrictionInput input;
  n::SiFrictionHistory history;
};
SiPacket Convert(const Case& c, n::UnitScale units) {
  const Dimensions u(units);
  SiPacket out;
  const auto& in = c.input.normal;
  auto& normal = out.input.normal;
  normal.penetration = in.penetration * u.length;
  normal.stiffness = in.stiffness * u.stiffness;
  normal.dt = in.dt * u.time; normal.time = in.time * u.time;
  normal.secondary_mass = in.secondary_mass * u.mass;
  normal.friction_viscosity = in.friction_viscosity;
  for (unsigned i = 0; i < 4; ++i) {
    normal.main_mass[i] = in.main_mass[i] * u.mass; normal.weights[i] = in.weights[i];
    out.input.main_vertices[i] = vector::Scale(c.input.main_vertices[i], u.length);
  }
  out.input.normal_axis = c.input.normal_axis;
  out.input.relative_velocity = vector::Scale(c.input.relative_velocity, u.velocity);
  normal.normal_velocity = vector::Dot(out.input.normal_axis, out.input.relative_velocity);
  out.input.dt12 = c.input.dt12 * u.time;
  out.history.normal = History(c.history.normal, u);
  out.history.previous_force = vector::Scale(c.history.previous_force, u.force);
  out.history.staged_force = vector::Scale(c.history.staged_force, u.force);
  out.coefficients.base = c.coefficients.base;
  // C1: pressure^-2; C3: pressure^-1; C2/C4/C6: velocity^-1; C5: dimensionless.
  out.coefficients.c[0] = c.coefficients.c[0] / (u.pressure * u.pressure);
  out.coefficients.c[1] = c.coefficients.c[1] / u.velocity;
  out.coefficients.c[2] = c.coefficients.c[2] / u.pressure;
  out.coefficients.c[3] = c.coefficients.c[3] / u.velocity;
  out.coefficients.c[4] = c.coefficients.c[4];
  out.coefficients.c[5] = c.coefficients.c[5] / u.velocity;
  return out;
}
n::NormalStatus Evaluate(const Case& c, n::UnitScale units,
    const SiPacket& packet, n::SiFrictionResult* result) {
  return n::EvaluateSiFriction(c.normal_config, c.controls, units,
      packet.coefficients, packet.input, packet.history, result);
}
void Scaled(const n::NativeFrictionResult& native, const n::SiFrictionResult& si,
            n::UnitScale units) {
  const Dimensions u(units);
  const std::array<double, 43> scales{{
    // P1 normal result: history, weights, stiffness/force/energy/damping.
    u.length, u.stiffness, u.length, u.stiffness, u.force, 1, 1, 1, 1,
    u.stiffness, u.stiffness, u.force, u.energy, u.force, u.energy,
    u.damping, u.stiffness, u.damping,
    // Full coupled history and vector force channels.
    u.length, u.stiffness, u.length, u.stiffness, u.force,
    u.force, u.force, u.force, u.force, u.force, u.force,
    u.force, u.force, u.force, u.force, u.force, u.force,
    u.force, u.force, u.force,
    // Coefficient, limiter, geometry area, pressure and work.
    1, 1, u.area, u.pressure, u.energy}};
  ASSERT_EQ(native.normal.terms_valid, si.normal.terms_valid);
  ASSERT_EQ(native.contact_active, si.contact_active);
  const auto a = Fields(native), b = Fields(si);
  for (std::size_t i = 0; i < a.size(); ++i) {
    SCOPED_TRACE(i);
    const double expected = a[i] * scales[i];
    ASSERT_TRUE(std::isfinite(expected)); ASSERT_TRUE(std::isfinite(b[i]));
    const double magnitude = std::max({std::abs(expected), std::abs(b[i]), 1e-300});
    EXPECT_NEAR(b[i], expected, 64 * std::numeric_limits<double>::epsilon() * magnitude);
  }
}
} // namespace

TEST(Type25FrictionUnits, AllDarmstadCoefficientDimensionsMatchNativeOracle) {
  const n::UnitScale units{0.125, 8, 0.5}; // F=4N, v=.25m/s, pressure=256Pa.
  for (unsigned changed = 0; changed < 6; ++changed) {
    SCOPED_TRACE(changed);
    auto c = Basic();
    c.coefficients = {0.125, {0.125, -0.125, 0.0625, -0.25, 0.125, -0.0625}};
    c.coefficients.c[changed] *= 2;
    const auto packet = Convert(c, units);
    EXPECT_DOUBLE_EQ(packet.coefficients.c[0], c.coefficients.c[0] / 65536.);
    EXPECT_DOUBLE_EQ(packet.coefficients.c[1], c.coefficients.c[1] * 4.);
    EXPECT_DOUBLE_EQ(packet.coefficients.c[2], c.coefficients.c[2] / 256.);
    EXPECT_DOUBLE_EQ(packet.coefficients.c[3], c.coefficients.c[3] * 4.);
    EXPECT_DOUBLE_EQ(packet.coefficients.c[4], c.coefficients.c[4]);
    EXPECT_DOUBLE_EQ(packet.coefficients.c[5], c.coefficients.c[5] * 4.);
    n::SiFrictionResult actual;
    ASSERT_EQ(Evaluate(c, units, packet, &actual), n::NormalStatus::Ok);
    Scaled(Oracle(c), actual, units);
  }
}

TEST(Type25FrictionUnits, ActualMmSecondTonneC6AndPhysicalAreaPressureForceWork) {
  const n::UnitScale units{0.001, 1000, 1};
  const auto c = Basic();
  const auto packet = Convert(c, units);
  EXPECT_DOUBLE_EQ(c.coefficients.c[5], -0.001);
  EXPECT_DOUBLE_EQ(packet.coefficients.c[5], -1.);
  n::SiFrictionResult actual;
  ASSERT_EQ(Evaluate(c, units, packet, &actual), n::NormalStatus::Ok);
  const auto native = Oracle(c);
  Scaled(native, actual, units);
  EXPECT_DOUBLE_EQ(native.contact_area, 100.);
  EXPECT_NEAR(actual.contact_area, 1e-4, 1e-19);
  EXPECT_NEAR(actual.pressure, native.pressure * 1e6, std::abs(native.pressure) * 1e-8);
  EXPECT_NEAR(actual.friction_work, native.friction_work * 0.001,
              std::abs(native.friction_work) * 1e-17);
}

TEST(Type25FrictionUnits, NonzeroCoupledHistoryAndNormalChannelsScaleTogether) {
  for (auto units : {n::UnitScale{1, 1, 1}, n::UnitScale{0.125, 8, 0.5}}) {
    auto c = Basic();
    c.input.normal.time = 0.0001;
    c.history.normal = {0.001, 300., 0.0005, 200., -0.01};
    c.history.previous_force = {0.25, -0.125, 0.0625};
    c.history.staged_force = {-0.5, 0.25, -0.125};
    n::SiFrictionResult actual;
    ASSERT_EQ(Evaluate(c, units, Convert(c, units), &actual), n::NormalStatus::Ok);
    Scaled(Oracle(c), actual, units);
  }
}

TEST(Type25FrictionUnits, Dt12ChangesPredictorAndDt1ChangesWorkIndependently) {
  const n::UnitScale units{0.125, 8, 0.5};
  auto c = Basic();
  c.normal_config.damping_factor = 0;
  c.coefficients = {100., {}};
  ASSERT_NE(c.input.normal.dt, c.input.dt12);
  n::SiFrictionResult base, changed_dt12, changed_dt1;
  ASSERT_EQ(Evaluate(c, units, Convert(c, units), &base), n::NormalStatus::Ok);
  ASSERT_DOUBLE_EQ(base.limiter, 1.);
  Scaled(Oracle(c), base, units);
  auto other = c; other.input.dt12 *= 2;
  ASSERT_EQ(Evaluate(other, units, Convert(other, units), &changed_dt12), n::NormalStatus::Ok);
  ASSERT_DOUBLE_EQ(changed_dt12.limiter, 1.);
  Scaled(Oracle(other), changed_dt12, units);
  EXPECT_DOUBLE_EQ(changed_dt12.tangent_predictor.x, 2 * base.tangent_predictor.x);
  EXPECT_DOUBLE_EQ(changed_dt12.tangent_predictor.y, 2 * base.tangent_predictor.y);
  type25_normal_test::Same(changed_dt12.normal, base.normal, true);
  other = c; other.input.normal.dt *= 2;
  ASSERT_EQ(Evaluate(other, units, Convert(other, units), &changed_dt1), n::NormalStatus::Ok);
  Scaled(Oracle(other), changed_dt1, units);
  EXPECT_DOUBLE_EQ(changed_dt1.tangent_predictor.x, base.tangent_predictor.x);
  EXPECT_DOUBLE_EQ(changed_dt1.tangent_predictor.y, base.tangent_predictor.y);
  EXPECT_DOUBLE_EQ(changed_dt1.friction_work, 2 * base.friction_work);
  EXPECT_GT(base.friction_work, 0.);
}

TEST(Type25FrictionUnits, NativeProjectionUsesConvertedComponentsWithoutFalseMismatch) {
  const n::UnitScale units{0.001, 1000, 1};
  auto c = Basic(); c.input.normal_axis = {0.6, 0.8, 0};
  bool exercised = false;
  // Bounded numerical witnesses, not source IDs or a tolerance on consistency.
  for (unsigned i = 0; i < 64 && !exercised; ++i) {
    auto packet = Convert(c, units);
    packet.input.relative_velocity = {0.0037 + i * 0.00011, -0.0213, 0.0011};
    packet.input.normal.normal_velocity =
        vector::Dot(packet.input.normal_axis, packet.input.relative_velocity);
    auto native = c;
    native.input.relative_velocity = vector::Divide(packet.input.relative_velocity, 0.001);
    RefreshNormalVelocity(native);
    if (native.input.normal.normal_velocity == packet.input.normal.normal_velocity / 0.001) continue;
    exercised = true;
    n::SiFrictionResult actual;
    ASSERT_EQ(Evaluate(c, units, packet, &actual), n::NormalStatus::Ok);
    Scaled(Oracle(native), actual, units);
    // A genuinely inconsistent SI scalar still rejects, with no partial output.
    packet.input.normal.normal_velocity += 1e-4;
    const auto prior = Sentinel<n::SiUnitsTag>(); actual = prior;
    EXPECT_EQ(Evaluate(c, units, packet, &actual), n::NormalStatus::InvalidInput);
    Same(actual, prior, true);
  }
  EXPECT_TRUE(exercised);
}

TEST(Type25FrictionUnits, StoredFloat32BoundaryNormalIsPreservedWithoutRenormalization) {
  // Native DST3_3 can copy a REAL*4 vertex bisector directly to its
  // ISHARP1 horizontal normal. This is a source-supported numerical control,
  // not a claim that these values were captured from a Yaris contact row.
  const auto c = Float32BoundaryNormal();
  const float stored = static_cast<float>(c.input.normal_axis.x);
  std::uint32_t bits = 0;
  static_assert(sizeof(stored) == sizeof(bits));
  std::memcpy(&bits, &stored, sizeof(bits));
  ASSERT_EQ(bits, UINT32_C(0x3f3504f3));
  const double component = stored; // Exact float32 -> float64 promotion.
  EXPECT_DOUBLE_EQ(component, 11863283. / 16777216.);
  EXPECT_DOUBLE_EQ(c.input.normal_axis.x, component);
  EXPECT_DOUBLE_EQ(c.input.normal_axis.y, component);
  EXPECT_DOUBLE_EQ(c.input.normal_axis.z, 0.);
  const double norm_error = std::abs(vector::Dot(c.input.normal_axis, c.input.normal_axis) - 1);
  ASSERT_GT(norm_error, 1e-12);
  ASSERT_LT(norm_error, 1e-6);
  const auto expected = Oracle(c);
  n::NativeFrictionResult native;
  ASSERT_EQ(n::EvaluateNativeFriction(c.normal_config, c.controls, c.coefficients,
      c.input, c.history, &native), n::NormalStatus::Ok);
  Same(native, expected);
  for (auto units : {n::UnitScale{1, 1, 1}, n::UnitScale{0.001, 1000, 1}}) {
    const auto packet = Convert(c, units);
    n::SiFrictionResult actual;
    ASSERT_EQ(Evaluate(c, units, packet, &actual), n::NormalStatus::Ok);
    Scaled(expected, actual, units);
  }
}

TEST(Type25FrictionUnits, InvalidAndOverflowingConversionsPreserveAllOutputFields) {
  const auto c = Basic(); const auto packet = Convert(c, {1, 1, 1});
  const auto prior = Sentinel<n::SiUnitsTag>();
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (auto units : {n::UnitScale{}, n::UnitScale{-1, 1, 1}, n::UnitScale{1, 0, 1},
                    n::UnitScale{1, 1, inf}, n::UnitScale{nan, 1, 1}}) {
    auto actual = prior;
    EXPECT_EQ(Evaluate(c, units, packet, &actual), n::NormalStatus::InvalidInput);
    Same(actual, prior, true);
  }
  // Base factors stay finite; area/pressure or converted values overflow.
  for (auto units : {n::UnitScale{1e200, 1e-200, 1},
                    n::UnitScale{1e-200, 1e200, 1}}) {
    auto actual = prior;
    EXPECT_EQ(Evaluate(c, units, packet, &actual), n::NormalStatus::NonfiniteResult);
    Same(actual, prior, true);
  }
  auto coefficients_overflow = packet;
  coefficients_overflow.coefficients.c[0] = std::numeric_limits<double>::max();
  auto actual = prior;
  EXPECT_EQ(Evaluate(c, {0.125, 8, 0.5}, coefficients_overflow, &actual),
            n::NormalStatus::NonfiniteResult);
  Same(actual, prior, true);
  EXPECT_EQ(Evaluate(c, {1, 1, 1}, packet, nullptr), n::NormalStatus::InvalidInput);
}
} // namespace type25_friction_test
