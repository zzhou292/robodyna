// SPDX-License-Identifier: AGPL-3.0-or-later
// I25FOR3 selected MFROT2/IFQ10/INCONV1/INTTH0, OpenRadioss (C)2026 Siemens.
#pragma once
#include "FrictionTypes.h"
#include "NormalResponse.h"
#include "lib_src/math/Fixed3Operations.h"
namespace tlfea::contact::radioss_type25 {
namespace friction_detail {
namespace vector = tl::math::fixed3;
TL_MATH_HOST_DEVICE inline bool Supported(const FrictionControls& c) {
  return c.model == 2 && c.formulation == 10 && c.orthotropic == 0 &&
      c.converged == 1 && c.thermal == 0 && c.part_coefficients == 0 && c.alpha == 1;
}
template<class Units> TL_MATH_HOST_DEVICE inline bool Valid(const FrictionHistory<Units>& h) {
  return normal_detail::Valid(h.normal) && vector::Finite(h.previous_force) && vector::Finite(h.staged_force);
}
template<class Units> TL_MATH_HOST_DEVICE inline bool Valid(const FrictionCoefficients<Units>& c) {
  if (!tl::math::Finite(c.base)) return false;
  for (double value : c.c) if (!tl::math::Finite(value)) return false;
  return true;
}
template<class Units> TL_MATH_HOST_DEVICE inline bool ValidGeometry(const FrictionInput<Units>& in) {
  // Native geometry can promote a REAL*4 bisector without renormalization.
  // The arithmetic leaf preserves every finite component, including a zero
  // vector; geometric admissibility belongs to the native geometry/selection stage.
  if (!vector::Finite(in.normal_axis) || !vector::Finite(in.relative_velocity)) return false;
  for (const auto& v : in.main_vertices) if (!vector::Finite(v)) return false;
  // Native VN and the friction projection must describe identical motion.
  return in.normal.normal_velocity == vector::Dot(in.normal_axis, in.relative_velocity);
}
template<class Units> TL_MATH_HOST_DEVICE inline bool Finite(const FrictionResult<Units>& r) {
  return normal_detail::Finite(r.normal) && Valid(r.history) &&
      vector::Finite(r.tangent_predictor) && vector::Finite(r.tangent_force) &&
      vector::Finite(r.native_resultant) && normal_detail::Nonnegative(r.coefficient) &&
      normal_detail::Nonnegative(r.limiter) && r.limiter <= 1 &&
      normal_detail::Nonnegative(r.contact_area) && tl::math::Finite(r.pressure) &&
      tl::math::Finite(r.friction_work);
}
} // namespace friction_detail
// Native force-argument convention. I25ASS3 applies endpoint signs later.
// No extra objective rotation, unilateral force clamp or positive-work clamp.
TL_MATH_HOST_DEVICE inline NormalStatus EvaluateNativeFriction(
    const ResolvedNormalConfig& normal_config, const FrictionControls& controls,
    const NativeFrictionCoefficients& coefficients, const NativeFrictionInput& in,
    const NativeFrictionHistory& history, NativeFrictionResult* output) {
  using namespace friction_detail;
  if (!output || !Valid(history) || !Valid(coefficients) ||
      !normal_detail::Nonnegative(in.dt12)) return NormalStatus::InvalidInput;
  if (!Supported(controls) || in.normal.friction_viscosity != 0)
    return NormalStatus::UnsupportedProfile;
  NativeFrictionResult result;
  result.history = history;
  auto status = EvaluateNativeNormal(normal_config, in.normal, history.normal, &result.normal);
  if (status != NormalStatus::Ok) return status;
  result.history.normal = result.normal.history;
  if (in.normal.penetration == 0) { *output = result; return NormalStatus::Ok; }
  if (!ValidGeometry(in)) return NormalStatus::InvalidInput;
  result.contact_active = true;
  const auto n = in.normal_axis, v = in.relative_velocity;
  const double normal_velocity = vector::Dot(n, v);
  const auto tangential_velocity = vector::Subtract(v, vector::Scale(n, normal_velocity));
  const double speed2 = vector::Dot(tangential_velocity, tangential_velocity);
  const double speed = ::sqrt(normal_detail::Maximum(native_constant::em30, speed2));
  const auto diagonal1 = vector::Subtract(in.main_vertices[2], in.main_vertices[0]);
  const auto diagonal2 = vector::Subtract(in.main_vertices[3], in.main_vertices[1]);
  const auto crossed = vector::Cross(diagonal1, diagonal2);
  result.contact_area = 0.5 * ::sqrt(vector::Dot(crossed, crossed));
  result.pressure = -result.normal.normal_force / result.contact_area;
  const auto& c = coefficients.c;
  double coefficient = coefficients.base + c[0] * ::exp(c[1] * speed) * result.pressure * result.pressure
      + c[2] * ::exp(c[3] * speed) * result.pressure + c[4] * ::exp(c[5] * speed);
  result.coefficient = normal_detail::Maximum(coefficient, native_constant::em30);
  const Vector increment{in.normal.stiffness * v.x * in.dt12,
                         in.normal.stiffness * v.y * in.dt12,
                         in.normal.stiffness * v.z * in.dt12};
  auto trial = vector::Add(history.previous_force, vector::Scale(increment, controls.alpha));
  const double normal_part = vector::Dot(trial, n);
  trial = vector::Subtract(trial, vector::Scale(n, normal_part));
  result.tangent_predictor = trial;
  const double tangent2 = normal_detail::Maximum(vector::Dot(trial, trial), native_constant::em30);
  const auto normal_force = vector::Scale(n, result.normal.normal_force);
  const double normal2 = vector::Dot(normal_force, normal_force);
  const double bound = result.coefficient * ::sqrt(normal2 / tangent2);
  result.limiter = bound < 1. ? bound : 1.;
  result.tangent_force = vector::Scale(trial, result.limiter);
  result.history.staged_force = result.tangent_force;
  result.native_resultant = vector::Add(normal_force, result.tangent_force);
  result.friction_work = 0. + in.normal.dt * vector::Dot(v, result.tangent_force);
  if (!tl::math::Finite(coefficient) || !tl::math::Finite(speed2) ||
      !tl::math::Finite(tangent2) || !tl::math::Finite(normal2) || !tl::math::Finite(bound) ||
      !(result.contact_area > 0) || !Finite(result)) return NormalStatus::NonfiniteResult;
  *output = result;
  return NormalStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
