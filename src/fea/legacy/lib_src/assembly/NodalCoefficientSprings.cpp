// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCoefficientInternal.h"

namespace tl::fea::coefficient_detail {
CoefficientReport Springs(NodalCoefficientSources input,NodalCoefficientNode* nodes) noexcept {
  const auto& domain=*input.shells->domain();
  if(input.type25) for(std::size_t e=0;e<input.type25->connection_count();++e) {
    const auto& connection=input.type25->connections()[e];
    for(unsigned local=0;local<2;++local) {
      const auto& term=input.type25->endpoint_mass()[2*e+local];
      const auto n=connection.global_node[local];
      if(n>=domain.node_count()||term.global_node!=n||
          term.source_node_id!=connection.source_node_id[local]||
          term.source_node_id!=domain.nodes()[n].source_id||
          term.source_element_id!=connection.source_element_id||
          term.source_property_id!=input.type25->properties()[connection.property_index].source_property_id)
        return {S::IdentityMismatch,"TYPE25 endpoint source association differs",P::Type25,e,local,n};
      if(!nodal_domain_detail::SamePosition(connection.position[local],domain.nodes()[n].position))
        return {S::PositionMismatch,"TYPE25 endpoint coordinate bits differ",P::Type25,e,local,n};
      auto& row=nodes[n];
      auto& c=row.coefficients;
      const auto mass=term.mass_kg,j=term.isotropic_inertia_kg_m2;
      if(!Positive(mass)||!Positive(j)||!AddPair(c.mass,c.isotropic_inertia,mass,j)||
          !AddPair(c.type25.mass,c.type25.isotropic_inertia,mass,j))
        return {S::NonfiniteResult,"TYPE25 coefficient addition failed",P::Type25,e,local,n};
      ++row.occurrences.type25;
    }
  }
  if(input.type13) for(const auto& record:input.type13->records()) {
    const auto& term=record.value;
    const auto n=term.global_node;
    if(n>=domain.node_count()||term.source_node_id!=domain.nodes()[n].source_id)
      return {S::IdentityMismatch,"TYPE13 checked record domain differs",P::Type13,record.model_connection,record.endpoint,n};
    auto& row=nodes[n];
    auto& c=row.coefficients;
    const auto mass=term.coefficients.mass_kg,j=term.coefficients.isotropic_inertia_kg_m2;
    const auto added=term.coefficients.added_inertia_kg_m2;
    if(!Positive(mass)||!Positive(j)||!Nonnegative(added)||added>j||
        !AddPair(c.mass,c.isotropic_inertia,mass,j)||
        !AddPair(c.type13.mass,c.type13.isotropic_inertia,mass,j)||!Add(c.type13.added_inertia,added))
      return {S::NonfiniteResult,"TYPE13 coefficient addition failed",P::Type13,record.model_connection,record.endpoint,n};
    ++row.occurrences.type13;
  }
  return {};
}
} // namespace tl::fea::coefficient_detail
