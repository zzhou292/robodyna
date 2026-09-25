// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NormalResponse.h"
namespace tlfea::contact::radioss_type25 {
// Native length, mass and time measured in SI. Explicit at every SI boundary.
// Yaris wrapper mm/s/metric-tonne is {0.001,1000,1}; no vehicle default is implied.
struct UnitScale { double length_m = 0, mass_kg = 0, time_s = 0; };
namespace units_detail {
struct Factors { double length, mass, time, velocity, stiffness, force, energy, damping; };
TL_MATH_HOST_DEVICE inline bool Make(UnitScale units, Factors& f) {
  if (!tl::math::Finite(units.length_m) || units.length_m <= 0 ||
      !tl::math::Finite(units.mass_kg) || units.mass_kg <= 0 ||
      !tl::math::Finite(units.time_s) || units.time_s <= 0) return false;
  const double time2 = units.time_s * units.time_s;
  if (!tl::math::Finite(time2) || time2 <= 0) return false;
  f = {units.length_m, units.mass_kg, units.time_s,
       units.length_m / units.time_s, units.mass_kg / time2,
       (units.mass_kg * units.length_m) / time2,
       ((units.mass_kg * units.length_m) / time2) * units.length_m,
       units.mass_kg / units.time_s};
  return tl::math::Finite(f.velocity) && f.velocity > 0 &&
      tl::math::Finite(f.stiffness) && f.stiffness > 0 &&
      tl::math::Finite(f.force) && f.force > 0 &&
      tl::math::Finite(f.energy) && f.energy > 0 &&
      tl::math::Finite(f.damping) && f.damping > 0;
}
} // namespace units_detail
TL_MATH_HOST_DEVICE inline NormalStatus EvaluateSiNormal(const ResolvedNormalConfig& config,
    UnitScale units, const SiNormalInput& input, const SiNormalHistory& history,
    SiNormalResult* output) {
  units_detail::Factors f;
  if (!output || !normal_detail::Valid(input) || !normal_detail::Valid(history) ||
      !units_detail::Make(units, f)) return NormalStatus::InvalidInput;
  NativeNormalInput in;
  in.penetration = input.penetration / f.length;
  in.stiffness = input.stiffness / f.stiffness;
  in.normal_velocity = input.normal_velocity / f.velocity;
  in.dt = input.dt / f.time; in.time = input.time / f.time;
  in.secondary_mass = input.secondary_mass / f.mass;
  in.friction_viscosity = input.friction_viscosity;
  for (unsigned i = 0; i < 4; ++i) {
    in.main_mass[i] = input.main_mass[i] / f.mass;
    in.weights[i] = input.weights[i];
  }
  const NativeNormalHistory old{history.previous_penetration / f.length,
      history.previous_stiffness / f.stiffness, history.staged_penetration / f.length,
      history.staged_stiffness / f.stiffness, history.damping_half_force / f.force};
  if (!normal_detail::Valid(in) || !normal_detail::Valid(old))
    return NormalStatus::NonfiniteResult;
  NativeNormalResult native;
  const auto status = EvaluateNativeNormal(config, in, old, &native);
  if (status != NormalStatus::Ok) return status;
  SiNormalResult result;
  result.history = {native.history.previous_penetration * f.length,
      native.history.previous_stiffness * f.stiffness,
      native.history.staged_penetration * f.length,
      native.history.staged_stiffness * f.stiffness,
      native.history.damping_half_force * f.force};
  for (unsigned i = 0; i < 4; ++i) result.weights[i] = native.weights[i];
  result.force_stiffness = native.force_stiffness * f.stiffness;
  result.stability_stiffness = native.stability_stiffness * f.stiffness;
  result.normal_force = native.normal_force * f.force;
  result.elastic_energy = native.elastic_energy * f.energy;
  result.damping_force = native.damping_force * f.force;
  result.damping_work = native.damping_work * f.energy;
  result.damping_coefficient = native.damping_coefficient * f.damping;
  result.separate_elastic_stiffness = native.separate_elastic_stiffness * f.stiffness;
  result.separate_friction_damping = native.separate_friction_damping * f.damping;
  result.terms_valid = native.terms_valid;
  if (!normal_detail::Finite(result)) return NormalStatus::NonfiniteResult;
  *output = result;
  return NormalStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
