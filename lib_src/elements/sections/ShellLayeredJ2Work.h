// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected MULAWC generalized work and instantaneous DM; (C) 2026 Siemens.
#pragma once
#include "ShellLayeredJ2.h"
#include "ShellLayeredWork.h"

namespace tl::fea::sections {
// Startup validates immutable coefficients without evaluating a dt=0 material
// interval or advancing a rate filter before the owner's first real step.
TL_SHELL_SECTION_HD inline bool ValidLayeredJ2Parameters(const PointParameters& p) noexcept {
  PointParameters checked;
  if(tl::material::tabulated_shell_detail::ValidHardening(p)!=PointStatus::Ok) return false;
  const auto status=p.hardening==tl::material::ShellPlasticityHardeningKind::Tabulated?
    tl::material::PrepareTabulatedShellPlasticity(p.young_pa,p.poisson_ratio,p.density_kg_m3,
        p.curve,p.rate,p.continuation,checked):
    tl::material::PrepareLinearLaw44ShellPlasticity(p.young_pa,p.poisson_ratio,p.density_kg_m3,p.linear,p.rate,checked);
  if(status!=PointStatus::Ok) return false;
  return p.shear_modulus==checked.shear_modulus&&p.a11==checked.a11&&p.a12==checked.a12&&
      p.three_g==checked.three_g&&p.sound_speed==checked.sound_speed&&
      p.inverse_rate_c==checked.inverse_rate_c&&p.inverse_rate_p==checked.inverse_rate_p&&
      p.angular_cutoff_per_s==checked.angular_cutoff_per_s;
}
// Family histories remain unchanged. The caller stages h and the section as one
// trial. No plastic-work diagnostic is added to this total stress-work ledger.
template<class HistoryValues>
TL_SHELL_SECTION_HD inline bool MatchesLayeredJ2Resultants(const ShellLayeredJ2History& section,
    const HistoryValues& h) noexcept {
  return MatchesLayeredSectionResultants(section,h);
}
template<class ReferenceInput>
TL_SHELL_SECTION_HD inline bool MatchesLayeredJ2Material(const PointParameters& p,
    const ReferenceInput& r) noexcept {
  return MatchesLayeredMaterial(p,r);
}
template<class HistoryValues>
TL_SHELL_SECTION_HD inline bool ApplyLayeredJ2Work(const ShellLayeredJ2Result& section,
    const double (&dx)[8],double reference_thickness,double area,double viscosity,
    HistoryValues& h) noexcept {
  return ApplyLayeredSectionWork(section,dx,reference_thickness,area,viscosity,h);
}
} // namespace tl::fea::sections
