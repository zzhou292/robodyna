// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCoefficientInternal.h"

namespace tl::fea::coefficient_detail {
CoefficientReport Solids(const SolidNodeContributions* input, NodalCoefficientNode* nodes) noexcept {
  if (!input) return {};
  for (std::size_t e = 0; e < input->parents().size(); ++e) {
    const auto& parent = input->parents()[e];
    for (unsigned k = 0; k < parent.node_count; ++k) {
      const auto n = parent.domain_node[k];
      auto& row = nodes[n];
      double* subtotal = nullptr;
      std::uint64_t* occurrences = nullptr;
      P producer;
      switch (parent.family) {
        case SolidCoefficientFamily::Solid18:
          subtotal = &row.coefficients.solid18_mass;
          occurrences = &row.occurrences.solid18; producer = P::Solid18; break;
        case SolidCoefficientFamily::Solid24:
          subtotal = &row.coefficients.solid24_mass;
          occurrences = &row.occurrences.solid24; producer = P::Solid24; break;
        case SolidCoefficientFamily::Solid6z:
          subtotal = &row.coefficients.solid6z_mass;
          occurrences = &row.occurrences.solid6z; producer = P::Solid6z; break;
        default: return {S::IdentityMismatch, "Unknown closed solid family"};
      }
      // Each native source-slot term is added once. No reconstruction from
      // element totals and no invented inertia for a translation-only node.
      if (!Add(row.coefficients.mass, parent.mass_kg[k]) || !Add(*subtotal, parent.mass_kg[k]))
        return {S::NonfiniteResult, "Solid mass accumulation overflow", producer, e, k, n};
      ++*occurrences;
    }
  }
  return {};
}
}  // namespace tl::fea::coefficient_detail
