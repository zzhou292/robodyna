// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchFields.h"
#include <cstdint>
#include <type_traits>
namespace tl::fea::connector_measurement {
// Fresh per-attempt operands, not parent subtotals or accepted history.
template<unsigned Channels> struct ParentOperands {
  double work[Channels]{}, increment[Channels]{};
  double kick[2]{}, drift[2]{};
  double native_dt = 0;
  std::uint8_t active = 0, newly_failed = 0;
};
static_assert(sizeof(ParentOperands<6>) == 144 && sizeof(ParentOperands<4>) == 112);
static_assert(std::is_trivially_copyable_v<ParentOperands<6>>);
static_assert(std::is_trivially_copyable_v<ParentOperands<4>>);
struct Addends {
  double* values;
  unsigned next = 0;
  TL_SURFACE_HD void operator+=(double value) noexcept { values[next++] = value; }
};
// Literal native RHS (+1) endpoint expressions from TYPE13 and TYPE25 Measure.
// Each output records both += operands independently, preserving the later fold.
template<class Endpoint, class Sum>
TL_SURFACE_HD inline void AccumulateEndpointWork(const std::size_t (&nodes)[2],
    const Endpoint (&rhs)[2], const NodalPreparedView& view, double h,
    Sum& kick, Sum& drift) noexcept {
  namespace f = shell_batch_fields;
  for (unsigned i = 0; i < 2; ++i) {
    const auto n = nodes[i];
    const auto v0 = f::ReadVector(view.base_kinematics.velocity_xyz, n);
    const auto v1 = f::ReadVector(view.kinematics.velocity_xyz, n);
    const auto w0 = f::ReadVector(view.base_kinematics.angular_velocity_xyz, n);
    const auto w1 = f::ReadVector(view.kinematics.angular_velocity_xyz, n);
    const auto dx = f::Difference(f::ReadVector(view.kinematics.position_xyz, n),
        f::ReadVector(view.base_kinematics.position_xyz, n));
    const tl::math::Vec3 rotation{h * w1.x, h * w1.y, h * w1.z};
    kick += view.kick_dt * (f::Dot(rhs[i].force_N, f::Mean(v0, v1)) +
        f::Dot(rhs[i].couple_Nm, f::Mean(w0, w1)));
    drift += f::Dot(rhs[i].force_N, dx) + f::Dot(rhs[i].couple_Nm, rotation);
  }
}
} // namespace tl::fea::connector_measurement
