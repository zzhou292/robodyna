// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/TabulatedShellPlasticity.h"
#include <cstdint>
namespace tl::fea::qeph {
// Exact resident coefficients without borrowed curve addresses. Serialization
// writes named values, not object padding; replay only rebinds the owned curve.
struct RejectedMaterialParameters {
  std::uint32_t curve_count=0;
  double young_pa=0,poisson_ratio=0,density_kg_m3=0;
  double shear_modulus=0,a11=0,a12=0,three_g=0,sound_speed=0;
  material::TabulatedShellPlasticityRate rate;
  double inverse_rate_c=0,inverse_rate_p=0,angular_cutoff_per_s=0;
  material::ShellPlasticityHardeningKind hardening=material::ShellPlasticityHardeningKind::Tabulated;
  material::Law44LinearHardening linear;
  double plastic_hardening_pa=0;
  material::ShellPlasticityCurveContinuation continuation=material::ShellPlasticityCurveContinuation::StrictDomain;
};
inline RejectedMaterialParameters CaptureMaterialParameters(
    const material::TabulatedShellPlasticityParameters& p) noexcept {
  return {p.curve.count,p.young_pa,p.poisson_ratio,p.density_kg_m3,
    p.shear_modulus,p.a11,p.a12,p.three_g,p.sound_speed,p.rate,
    p.inverse_rate_c,p.inverse_rate_p,p.angular_cutoff_per_s,p.hardening,
    p.linear,p.plastic_hardening_pa,p.continuation};
}
inline material::TabulatedShellPlasticityParameters ReplayMaterialParameters(
    const RejectedMaterialParameters& p,const double* x,const double* y) noexcept {
  material::TabulatedShellPlasticityParameters out;
  out.curve={p.curve_count?x:nullptr,p.curve_count?y:nullptr,p.curve_count};
  out.young_pa=p.young_pa;out.poisson_ratio=p.poisson_ratio;out.density_kg_m3=p.density_kg_m3;
  out.shear_modulus=p.shear_modulus;out.a11=p.a11;out.a12=p.a12;out.three_g=p.three_g;out.sound_speed=p.sound_speed;
  out.rate=p.rate;out.inverse_rate_c=p.inverse_rate_c;out.inverse_rate_p=p.inverse_rate_p;
  out.angular_cutoff_per_s=p.angular_cutoff_per_s;out.hardening=p.hardening;
  out.linear=p.linear;out.plastic_hardening_pa=p.plastic_hardening_pa;out.continuation=p.continuation;
  return out;
}
} // namespace tl::fea::qeph
