// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCoefficientInternal.h"

namespace tl::fea::coefficient_detail {
CoefficientReport ElementMasses(const ElementMassContributions* input,
    NodalCoefficientNode* nodes) noexcept {
  if(!input) return {};
  for(std::size_t i=0;i<input->records().size();++i) {
    const auto& point=input->records()[i];
    auto& node=nodes[point.source.domain_node];
    // Authoritative mass uses each original row, never its producer subtotal.
    // Native TYPE5 leaves nodal scalar J entirely untouched, including zero bits.
    if(!Add(node.coefficients.mass,point.mass_kg)||
        !Add(node.coefficients.element_mass,point.mass_kg))
      return {S::NonfiniteResult,"Element mass accumulation overflow",P::ElementMass,
              i,0,point.source.domain_node};
    ++node.occurrences.element_mass;
  }
  return {};
}
} // namespace tl::fea::coefficient_detail
