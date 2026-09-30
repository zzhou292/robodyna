// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "SelectedTensor.h"
#include "TotalGradient.h"
#include "lib_src/elements/solid18/Solid18CurrentGeometry.h"
#include "lib_src/elements/solid18/Solid18SelectiveShear.h"
#include "lib_src/elements/solid18/Solid18RateValues.h"
namespace tl::fea::solid18::total_strain {
namespace detail {
TL_SOLID18_HD inline Status SelectedRate(const PointDerivatives& point,
    const Vec3 (&v)[8], double dt, double (&rate)[6]) noexcept {
  using solid18::detail::RateSum;
  const auto& p = point.regular_per_m;
  const auto& s = point.shear_per_m;
  solid18::detail::QuadraticEngineeringRate(
      RateSum(p[0],v,0), RateSum(p[1],v,1), RateSum(p[2],v,2),
      RateSum(s[0],v,0), RateSum(s[2],v,0), RateSum(s[1],v,1),
      RateSum(s[4],v,1), RateSum(s[3],v,2), RateSum(s[5],v,2), dt, rate);
  for (double x : rate) if (!tl::math::Finite(x)) return Status::NonfiniteResult;
  return Status::Success;
}
}
// Inputs/accepted output are disjoint from scratch; publish staged only on
// success. No history, stored volume or energy is mutated by this value seam.
TL_SOLID18_HD inline Status EvaluateKinematics90Scratch(const Reference& reference,
    const KinematicsInput& input, KinematicsScratch& scratch) noexcept {
  if (!reference.prepared() || !tl::math::Finite(input.dt_s) || input.dt_s < 0)
    return Status::InvalidInput;
  for (unsigned n = 0; n < 8; ++n) {
    if (!solid18::detail::Finite(input.position_m[n]) ||
        !solid18::detail::Finite(input.velocity_m_s[n])) return Status::InvalidInput;
    scratch.native_position_m[n] = input.position_m[reference.source_slot(n)];
    scratch.native_velocity_m_s[n] = input.velocity_m_s[reference.source_slot(n)];
  }
  auto& next = scratch.staged;
  auto status = solid18::detail::NativeCurrentGeometryValues(scratch.native_position_m,
      scratch.native_velocity_m_s, next.geometry, scratch.current);
  if (status != Status::Success) return status;
  status = detail::TotalGradient(reference, scratch.native_position_m, next);
  if (status != Status::Success) return status;
  status = detail::SelectedTensor(next.point);
  if (status != Status::Success) return status;
  solid18::detail::SelectedShearDerivatives(next.geometry);
  for (unsigned r = 0; r < 2; ++r) {
    for (unsigned s = 0; s < 2; ++s) {
      for (unsigned t = 0; t < 2; ++t) {
        const unsigned ip = r+2*s+4*t;
        // ISELECT1 does not populate/use native cross scratch. Public values
        // declare those absent slots zero, never evaluate zero*unused data.
        for (auto& axis : next.geometry.point[ip].cross_per_m)
          for (double& x : axis) x = 0;
        status = detail::SelectedRate(next.geometry.point[ip],
            next.geometry.local_velocity_m_s, input.dt_s,
            next.point[ip].engineering_rate_per_s);
        if (status != Status::Success) return status;
      }
    }
  }
  return Status::Success;
}
inline Status EvaluateKinematics90(const Reference& reference, const KinematicsInput& input,
                                    Kinematics& output) noexcept {
  KinematicsScratch scratch;
  const auto status = EvaluateKinematics90Scratch(reference, input, scratch);
  if (status == Status::Success) output = scratch.staged;
  return status;
}
}  // namespace tl::fea::solid18::total_strain
