// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected one-thickness-point MULAWC packet, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "lib_src/materials/Law44MembranePlasticity.h"
#include "lib_src/materials/failure/ShellConstantPlasticFailure.h"

namespace tl::fea::sections {
struct MembraneLaw44PointInput {
  tl::material::TabulatedShellPlasticityInput material;
  double thickness_m=0, area_m2=0, endpoint_time_s=0;
};
struct MembraneLaw44PointResult {
  tl::material::TabulatedShellPlasticityResult current;
  tl::material::TabulatedShellPlasticityHistory saved;
  tl::material::failure::ConstantPlasticFailureResult failure;
  double reported_thickness_m=0, plastic_work_increment_j=0;
};
// Pure material packet. No surface/parent removal, force masking, numerical
// viscosity, quadrature correction or force-stage shear-work correction.
#if defined(__CUDACC__)
__host__ __device__
#endif
inline bool UpdateZeroShearLaw44Point(
    const tl::material::TabulatedShellPlasticityParameters& parameters,
    tl::material::failure::ConstantPlasticFailureParameters failure_parameters,
    const tl::material::TabulatedShellPlasticityHistory& accepted,
    tl::material::failure::ConstantPlasticFailureHistory failure_history,
    const MembraneLaw44PointInput& input,MembraneLaw44PointResult& output) noexcept {
  using tl::math::Finite;
  if (!Finite(input.thickness_m) || input.thickness_m<=0 ||
      !Finite(input.area_m2) || input.area_m2<=0) return false;
  MembraneLaw44PointResult next;
  if (tl::material::UpdateLaw44ZeroShearPlasticity(parameters,accepted,input.material,next.current)!=
      tl::material::TabulatedShellPlasticityStatus::Ok) return false;
  double thickness=input.thickness_m;
  thickness=thickness+next.current.elastic_thickness_strain*input.thickness_m;
  thickness=thickness+next.current.plastic_thickness_strain*input.thickness_m;
  if (!Finite(thickness) || thickness<=0) return false;
  next.reported_thickness_m=thickness;
  next.plastic_work_increment_j=next.current.plastic_work_density*input.thickness_m*input.area_m2;
  const double increment=next.current.history.plastic_strain-accepted.plastic_strain;
  if (!Finite(next.plastic_work_increment_j) ||
      !tl::material::failure::UpdateConstantPlasticFailure(failure_parameters,failure_history,
          {increment,input.endpoint_time_s,input.material.element_active},next.failure)) return false;
  next.saved=next.current.history;
  const double mask=next.failure.history.point_active?1.:0.;
  for (double& stress:next.saved.stress) stress=stress*mask;
  output=next;
  return true;
}
// Retain the narrower CBADEF1/CBASTRA3 packet admission for existing callers.
#if defined(__CUDACC__)
__host__ __device__
#endif
inline bool UpdateMembraneLaw44Point(
    const tl::material::TabulatedShellPlasticityParameters& parameters,
    tl::material::failure::ConstantPlasticFailureParameters failure_parameters,
    const tl::material::TabulatedShellPlasticityHistory& accepted,
    tl::material::failure::ConstantPlasticFailureHistory failure_history,
    const MembraneLaw44PointInput& input,MembraneLaw44PointResult& output) noexcept {
  if (input.material.strain_increment[3]!=0 || input.material.strain_increment[4]!=0) return false;
  return UpdateZeroShearLaw44Point(parameters,failure_parameters,accepted,failure_history,input,output);
}
} // namespace tl::fea::sections
