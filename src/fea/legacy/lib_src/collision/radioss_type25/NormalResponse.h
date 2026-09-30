// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected I25FOR3 normal response, OpenRadioss Copyright (C) 2026 Siemens.
// Donor a62b27e6: lines282-326,339-378,413-452,691-709. See README.md.
#pragma once
#include "Types.h"
#include "NativeConstants.h"
#include "lib_src/math/Quaternion.h"
#include "lib_src/math/HostDevice.h"
namespace tlfea::contact::radioss_type25 {
namespace normal_detail {
TL_MATH_HOST_DEVICE inline double Maximum(double a, double b) { return a < b ? b : a; }
TL_MATH_HOST_DEVICE inline bool Nonnegative(double value) {
  return tl::math::Finite(value) && value >= 0;
}
template<class Units> TL_MATH_HOST_DEVICE inline bool Valid(const NormalHistory<Units>& h) {
  return Nonnegative(h.previous_penetration) && Nonnegative(h.previous_stiffness) &&
      Nonnegative(h.staged_penetration) && Nonnegative(h.staged_stiffness) &&
      tl::math::Finite(h.damping_half_force);
}
template<class Units> TL_MATH_HOST_DEVICE inline bool Valid(const NormalInput<Units>& in) {
  if (!Nonnegative(in.penetration) || !Nonnegative(in.stiffness) ||
      !tl::math::Finite(in.normal_velocity) || !Nonnegative(in.dt) ||
      !Nonnegative(in.time) || !Nonnegative(in.secondary_mass) ||
      !Nonnegative(in.friction_viscosity)) return false;
  for (unsigned i = 0; i < 4; ++i)
    if (!Nonnegative(in.main_mass[i]) || !tl::math::Finite(in.weights[i])) return false;
  return true;
}
template<class Units> TL_MATH_HOST_DEVICE inline bool Finite(const NormalResult<Units>& out) {
  return Valid(out.history) && Nonnegative(out.force_stiffness) &&
      Nonnegative(out.stability_stiffness) && tl::math::Finite(out.normal_force) &&
      Nonnegative(out.elastic_energy) && tl::math::Finite(out.damping_force) &&
      tl::math::Finite(out.damping_work) && Nonnegative(out.damping_coefficient) &&
      Nonnegative(out.separate_elastic_stiffness) &&
      Nonnegative(out.separate_friction_damping);
}
} // namespace normal_detail
// Pure packet response: no geometry, friction force, pair/history ownership,
// mass scaling or global timestep selection. Units are the donor working units.
// Inputs may refer to fields of *output: all reads precede successful publication.
// Invalid/nonfinite input or result leaves the entire prior output unchanged.
TL_MATH_HOST_DEVICE inline NormalStatus EvaluateNativeNormal(const ResolvedNormalConfig& config,
    const NativeNormalInput& in, const NativeNormalHistory& history,
    NativeNormalResult* output) {
  using namespace normal_detail;
  if (!output || !Valid(in) || !Valid(history) ||
      !Nonnegative(config.damping_factor)) return NormalStatus::InvalidInput;
  if (config.stiffness_formulation != 4 || config.damping_flag != 1 ||
      config.initial_penetration != 5 || config.arithmetic_precision != 8 ||
      config.prescribed_contact_force || config.adhesion ||
      config.engine.kdtint < 0 || config.engine.kdtint > 1 ||
      config.engine.idtmins < 0 || config.engine.idtmins > 2 ||
      config.engine.idtmins_int < 0 || config.engine.idtmins_int > 1)
    return NormalStatus::UnsupportedProfile;
  NativeNormalResult result;
  result.history = history;
  result.stability_stiffness = in.stiffness;
  if (in.penetration == 0) { // Native loops skip history, force and energy work.
    *output = result; // Native no-contact path clears interpolation weights.
    return NormalStatus::Ok;
  }
  for (unsigned i = 0; i < 4; ++i) result.weights[i] = in.weights[i];
  const double stiffness0 = in.stiffness;
  double stiffness = stiffness0;
  const double dpenetration = Maximum(0., -in.normal_velocity * in.dt);
  if (in.time != 0) {
    if (in.penetration > history.previous_penetration + dpenetration + native_constant::epp) {
      const double r1 = history.previous_penetration / in.penetration;
      const double r2 = dpenetration / in.penetration;
      stiffness = history.previous_stiffness * r1 + stiffness * r2;
    } else stiffness = history.previous_stiffness;
    result.history.staged_penetration = in.penetration;
    result.history.staged_stiffness = stiffness;
  } else {
    result.history.staged_penetration = Maximum(history.staged_penetration, in.penetration);
    result.history.staged_stiffness = Maximum(history.staged_stiffness, stiffness);
  }
  stiffness = 0.5 * stiffness; // STIGLO<=0 for the admitted source formulation.
  result.force_stiffness = stiffness;
  result.normal_force = -stiffness * in.penetration;
  result.elastic_energy = 0. + 0.5 * stiffness * (in.penetration * in.penetration);
  const double inverse_dt = in.dt > 0 ? 1. / in.dt : 0.;
  double viscous_force = 0.;
  if (config.damping_factor != 0 || in.friction_viscosity != 0) {
    const double main_mass = in.main_mass[0] * in.weights[0] +
        in.main_mass[1] * in.weights[1] + in.main_mass[2] * in.weights[2] +
        in.main_mass[3] * in.weights[3];
    const double viscosity2 = 2. * stiffness * in.secondary_mass * main_mass /
        Maximum(native_constant::em30, in.secondary_mass + main_mass);
    const double factor = stiffness / Maximum(native_constant::em30, stiffness);
    const double viscosity = ::sqrt(viscosity2);
    const double coefficient = factor * config.damping_factor * viscosity;
    const double friction_coefficient = factor * ::sqrt(in.friction_viscosity) * viscosity;
    result.damping_coefficient = coefficient;
    const bool combined = config.engine.kdtint == 0 &&
        config.engine.idtmins != 2 && config.engine.idtmins_int == 0;
    if (combined) stiffness = stiffness0 + coefficient * inverse_dt;
    else {
      result.terms_valid = true;
      result.separate_elastic_stiffness = stiffness0;
      result.separate_friction_damping = friction_coefficient;
      stiffness = stiffness + coefficient * inverse_dt;
    }
    stiffness = Maximum(stiffness, friction_coefficient * inverse_dt);
    viscous_force = coefficient * in.normal_velocity;
    result.normal_force = result.normal_force + viscous_force;
  }
  result.stability_stiffness = stiffness;
  result.damping_force = viscous_force;
  double damping_work = 0.;
  damping_work = damping_work + history.damping_half_force * in.normal_velocity * in.dt;
  result.history.damping_half_force = 0.5 * viscous_force;
  damping_work = damping_work + result.history.damping_half_force * in.normal_velocity * in.dt;
  result.damping_work = damping_work;
  if (!Finite(result) || !tl::math::Finite(dpenetration)) return NormalStatus::NonfiniteResult;
  *output = result;
  return NormalStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
