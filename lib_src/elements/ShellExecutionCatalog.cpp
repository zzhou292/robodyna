// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchPlasticityBindingInternal.h"

namespace tl::fea {
ShellPlasticityBindingReport ShellBatchPlasticityBinding::InitializeExecutionCatalog(
    const ShellBatchBinding& binding, const ShellBatchPlasticityBindingInput& input,
    const ShellPlasticityCatalogLimits& limits) noexcept {
  return InitializeCatalogImpl(binding, input, limits, true, true, true);
}

bool ShellBatchPlasticityBinding::ValidRigidSkinMaterial(
    const ShellPlasticityMaterialInput& m) noexcept {
  using shell_plasticity_binding_detail::Same;
  // These are retained native reference values, never elastic point parameters.
  return tl::math::Finite(m.young_pa) && m.young_pa > 0 &&
      tl::math::Finite(m.density_kg_m3) && m.density_kg_m3 > 0 &&
      tl::math::Finite(m.poisson_ratio) && m.poisson_ratio >= 0 && m.poisson_ratio < .5 &&
      m.curve_id == 0 && m.hardening == material::ShellPlasticityHardeningKind::Tabulated &&
      m.continuation == material::ShellPlasticityCurveContinuation::StrictDomain &&
      Same(m.rate, material::TabulatedShellPlasticityRate{}) &&
      Same(m.linear.initial_yield_pa, 0.) && Same(m.linear.tangent_modulus_pa, 0.);
}

bool ShellBatchPlasticityBinding::MaterialPointCount(
    ShellBindingFamily family, std::size_t index, unsigned* output) const noexcept {
  ShellSectionLaw law;
  if (!output || !Law(family, index, &law)) return false;
  unsigned points;
  switch (law) {
    case ShellSectionLaw::RigidSkin:
    case ShellSectionLaw::GlobalLaw1Npt0: points = 0; break;
    case ShellSectionLaw::LayeredLaw1Nip3:
    case ShellSectionLaw::LayeredLaw44Nip3: points = 3; break;
    case ShellSectionLaw::Law44Nip1: points = 1; break;
    case ShellSectionLaw::Law44QbatFourInPlane: points = 4; break;
    default: return false;
  }
  *output = points;
  return true;
}
} // namespace tl::fea
