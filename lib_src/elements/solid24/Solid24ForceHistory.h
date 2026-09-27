// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid24ForceTypes.h"

namespace tl::fea::solid24::force_detail {
// Compare named input values only. No struct-padding or reduced hash identity.
TL_BRICK_HD inline bool SameFinite(double a,double b) noexcept {
  return a==b && (a!=0 || ::copysign(1.0,a)==::copysign(1.0,b));
}
TL_BRICK_HD inline bool SameMaterial(const Material& a,const Material& b) noexcept {
  return SameFinite(a.mu_pa,b.mu_pa) && SameFinite(a.poisson_ratio,b.poisson_ratio) &&
      SameFinite(a.bulk_pa,b.bulk_pa) && SameFinite(a.density_kg_m3,b.density_kg_m3) &&
      SameFinite(a.tension_cutoff_pa,b.tension_cutoff_pa);
}
TL_BRICK_HD inline bool SameReference(const Reference& a,const Reference& b) noexcept {
  if (!a.prepared() || !b.prepared()) return false;
  const auto& x=a.input(); const auto& y=b.input();
  if (x.source_element_id!=y.source_element_id || x.source_part_id!=y.source_part_id ||
      x.source_section_id!=y.source_section_id || x.source_material_id!=y.source_material_id ||
      !SameFinite(x.density_kg_m3,y.density_kg_m3)) return false;
  const auto& p=x.profile; const auto& q=y.profile;
  if (p.engine_jhbe!=q.engine_jhbe || p.integration_points!=q.integration_points ||
      p.startup_frame!=q.startup_frame || p.rotational_inertia!=q.rotational_inertia ||
      p.ale!=q.ale || p.reference_shape!=q.reference_shape ||
      p.reference_strain!=q.reference_strain || p.working_length!=q.working_length ||
      p.connectivity!=q.connectivity) return false;
  for (unsigned n=0; n<8; ++n) {
    if (x.source_node_id[n]!=y.source_node_id[n]) return false;
    for (unsigned k=0; k<3; ++k)
      if (!SameFinite(tl::fea::solid_common::Component(x.position_m[n],k),
                      tl::fea::solid_common::Component(y.position_m[n],k))) return false;
  }
  return true;
}
TL_BRICK_HD inline bool ValidMaterial(const Reference& r,const Material& p) noexcept {
  Material checked;
  return r.prepared() && r.reference_jacobian() && p.density_kg_m3==r.input().density_kg_m3 &&
      tl::material::law42::Prepare(p.mu_pa,p.poisson_ratio,p.density_kg_m3,
          p.tension_cutoff_pa,checked)==tl::material::law42::Status::Ok && SameMaterial(p,checked);
}
} // namespace tl::fea::solid24::force_detail

namespace tl::fea::solid24 {
TL_BRICK_HD inline ForceStatus InitializeHistory(const Reference& reference,
    const Material& material,History& output) noexcept {
  if (!reference.prepared() || !reference.reference_jacobian()) return ForceStatus::UnsupportedProfile;
  if (!force_detail::ValidMaterial(reference,material)) return ForceStatus::InvalidInput;
  History next;
  next.reference_=reference;
  next.material_=material;
  next.values_.material.density_kg_m3=material.density_kg_m3;
  next.initialized_=true;
  output=next;
  return ForceStatus::Success;
}
} // namespace tl::fea::solid24
