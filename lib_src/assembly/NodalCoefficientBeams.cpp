// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCoefficientInternal.h"
namespace tl::fea::coefficient_detail {
CoefficientReport Beams(const Beam18NodeContributions* beams,NodalCoefficientNode* nodes) noexcept {
  if(!beams) return {};
  const auto& domain=*beams->domain();
  for(const auto& record:beams->records()) {
    const auto& term=record.value; const auto node=term.global_node;
    if(node>=domain.node_count()||term.source_node_id!=domain.nodes()[node].source_id)
      return {S::IdentityMismatch,"Beam18 checked record domain differs",P::Beam18,record.model_parent,record.endpoint,node};
    auto& row=nodes[node]; auto& c=row.coefficients;
    const auto mass=term.coefficients.mass_kg,inertia=term.coefficients.native_total_inertia_kg_m2;
    if(!Positive(mass)||!Positive(inertia)||!AddPair(c.mass,c.isotropic_inertia,mass,inertia)||
        !AddPair(c.beam18.mass,c.beam18.isotropic_inertia,mass,inertia))
      return {S::NonfiniteResult,"Beam18 coefficient addition failed",P::Beam18,record.model_parent,record.endpoint,node};
    ++row.occurrences.beam18;
  }
  return {};
}
} // namespace tl::fea::coefficient_detail
