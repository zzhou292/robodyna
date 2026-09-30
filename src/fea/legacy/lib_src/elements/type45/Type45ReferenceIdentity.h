// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45Reference.h"

namespace tl::fea::type45 {
TL_TYPE45_HD inline bool Reference::Matches(const Reference& other) const {
  using detail::Same;
  if (!ready_ || !other.ready_ || !Same(property_,other.property_) ||
      geometry_.source_joint_id!=other.geometry_.source_joint_id ||
      !Same(context_.target_dt_s,other.context_.target_dt_s)) return false;
  for (unsigned i=0; i<3; ++i)
    if (geometry_.source_node_id[i]!=other.geometry_.source_node_id[i] ||
        !Same(geometry_.position_m[i],other.geometry_.position_m[i])) return false;
  for (unsigned i=0; i<2; ++i) {
    const auto& a=context_.main[i];
    const auto& b=other.context_.main[i];
    if (a.role!=b.role || a.source_body_id!=b.source_body_id ||
        !Same(a.position_m,b.position_m) || !Same(a.mass_kg,b.mass_kg) ||
        !Same(a.inertia_kg_m2,b.inertia_kg_m2) ||
        !Same(a.translational_stiffness_n_m,b.translational_stiffness_n_m) ||
        !Same(a.rotational_stiffness_nm,b.rotational_stiffness_nm) ||
        !Same(damping_[i].mass_kg,other.damping_[i].mass_kg) ||
        !Same(damping_[i].mean_principal_inertia_kg_m2,
              other.damping_[i].mean_principal_inertia_kg_m2)) return false;
  }
  return true;
}
} // namespace tl::fea::type45
