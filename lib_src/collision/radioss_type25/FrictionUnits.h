// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "IsotropicResponse.h"
#include "UnitConversions.h"
namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline NormalStatus EvaluateSiFriction(
    const ResolvedNormalConfig& normal_config, const FrictionControls& controls,
    UnitScale units, const SiFrictionCoefficients& coefficients, const SiFrictionInput& in,
    const SiFrictionHistory& history, SiFrictionResult* output) {
  namespace vector = tl::math::fixed3;
  units_detail::Factors f;
  if (!output || !friction_detail::Valid(history) || !friction_detail::Valid(coefficients) ||
      !normal_detail::Valid(in.normal) || !normal_detail::Nonnegative(in.dt12) ||
      !units_detail::Make(units, f)) return NormalStatus::InvalidInput;
  if (in.normal.penetration != 0 && !friction_detail::ValidGeometry(in)) return NormalStatus::InvalidInput;
  const double area_unit = f.length * f.length, pressure_unit = f.force / area_unit;
  if (!tl::math::Finite(area_unit) || area_unit <= 0 || !tl::math::Finite(pressure_unit) || pressure_unit <= 0)
    return NormalStatus::NonfiniteResult;
  NativeFrictionInput native_in;
  native_in.normal = units_detail::ToNative(in.normal, f);
  native_in.dt12 = in.dt12 / f.time;
  if (in.normal.penetration != 0) {
    native_in.normal_axis = in.normal_axis;
    native_in.relative_velocity = vector::Divide(in.relative_velocity, f.velocity);
    for (unsigned i = 0; i < 4; ++i) native_in.main_vertices[i] = vector::Divide(in.main_vertices[i], f.length);
    // Convert vector components, then use the original native dot-product order.
    // Converting an already-reduced SI VN can introduce a different rounding.
    native_in.normal.normal_velocity = vector::Dot(native_in.normal_axis, native_in.relative_velocity);
  }
  NativeFrictionHistory native_history;
  native_history.normal = units_detail::ToNative(history.normal, f);
  native_history.previous_force = vector::Divide(history.previous_force, f.force);
  native_history.staged_force = vector::Divide(history.staged_force, f.force);
  NativeFrictionCoefficients native_coefficients;
  native_coefficients.base = coefficients.base;
  native_coefficients.c[0] = coefficients.c[0] * pressure_unit * pressure_unit;
  native_coefficients.c[1] = coefficients.c[1] * f.velocity;
  native_coefficients.c[2] = coefficients.c[2] * pressure_unit;
  native_coefficients.c[3] = coefficients.c[3] * f.velocity;
  native_coefficients.c[4] = coefficients.c[4];
  native_coefficients.c[5] = coefficients.c[5] * f.velocity;
  if (!normal_detail::Valid(native_in.normal) || !normal_detail::Nonnegative(native_in.dt12) ||
      !friction_detail::Valid(native_history) || !friction_detail::Valid(native_coefficients) ||
      (in.normal.penetration != 0 && !friction_detail::ValidGeometry(native_in)))
    return NormalStatus::NonfiniteResult;
  NativeFrictionResult native;
  const auto status = EvaluateNativeFriction(normal_config, controls, native_coefficients,
      native_in, native_history, &native);
  if (status != NormalStatus::Ok) return status;
  SiFrictionResult result;
  result.normal = units_detail::ToSi(native.normal, f);
  result.history.normal = result.normal.history;
  result.history.previous_force = vector::Scale(native.history.previous_force, f.force);
  result.history.staged_force = vector::Scale(native.history.staged_force, f.force);
  result.tangent_predictor = vector::Scale(native.tangent_predictor, f.force);
  result.tangent_force = vector::Scale(native.tangent_force, f.force);
  result.native_resultant = vector::Scale(native.native_resultant, f.force);
  result.coefficient = native.coefficient; result.limiter = native.limiter;
  result.contact_area = native.contact_area * area_unit;
  result.pressure = native.pressure * pressure_unit;
  result.friction_work = native.friction_work * f.energy;
  result.contact_active = native.contact_active;
  if (!friction_detail::Finite(result)) return NormalStatus::NonfiniteResult;
  *output = result;
  return NormalStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
