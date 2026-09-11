// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "../type13_recurrence/NativeOracle.h"
#include "native/Packet.h"
#include <algorithm>
#include <cmath>

namespace type13_resident_test {
inline t::NativeHistory Virgin(const t::Reference& reference) {
  t::NativeHistory history;
  history.transverse_axis = tl::math::fixed3::Column(reference.axes, 1);
  return history;
}
inline void Agreement(const t::Evaluation& actual, const t::Evaluation& native) {
  // Unchanged budget from the qualified TYPE13 native recurrence test.
  const auto a = type13_recurrence_test::Values(actual);
  const auto b = type13_recurrence_test::Values(native);
  for (std::size_t k = 0; k < a.size(); ++k) {
    SCOPED_TRACE(k);
    ASSERT_TRUE(std::isfinite(a[k]));
    ASSERT_TRUE(std::isfinite(b[k]));
    EXPECT_NEAR(a[k], b[k], 2e-11 * std::max(1e-10, std::max(std::fabs(a[k]), std::fabs(b[k]))));
  }
  EXPECT_EQ(actual.native_history.active, native.native_history.active);
  EXPECT_EQ(actual.newly_failed, native.newly_failed);
  for (unsigned k = 0; k < t::ChannelCount; ++k) {
    EXPECT_EQ(actual.native_history.channels[k].curve_position,
              native.native_history.channels[k].curve_position);
  }
}
inline std::array<double, 4> NativeStiffness(const t::Property& property,
                                           const t::Evaluation& native) {
  const auto units = property.units();
  const double force = units.mass_to_kg * units.length_to_m / (units.time_to_s * units.time_to_s);
  const double moment = force * units.length_to_m;
  const double kt = native.stability.translation_stiffness_N_per_m * units.length_to_m / force;
  const double kr = native.stability.rotation_stiffness_Nm_per_rad / moment;
  const double zero = 0, active = native.native_history.active ? 1 : 0;
  const double mass[2]{0, 4}, inertia[2]{0, 7};
  const double element_mass = 11, element_inertia = 13;
  std::array<double, 4> out;
  type13_native_endpoint_stiffness(&kt, &kr, &zero, &zero, mass, inertia,
                                   &element_mass, &element_inertia, &active, out.data());
  out[0] *= force / units.length_to_m;
  out[1] *= force / units.length_to_m;
  out[2] *= moment;
  out[3] *= moment;
  return out;
}
} // namespace type13_resident_test
