// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tl::fea::type13 {
bool Batch::Impl::OutputDisjoint(const void* output, std::size_t bytes) const noexcept {
  using trial_identity::Disjoint;
  const auto& model = *source.model();
  if (!Disjoint(output, bytes, this, sizeof(*this)) ||
      !Disjoint(output, bytes, staging.get(), Count() * sizeof(Evaluation)) ||
      !Disjoint(output, bytes, model.nodes(), model.node_count() * sizeof(ModelNode)) ||
      !Disjoint(output, bytes, model.connections(), Count() * sizeof(ModelConnection)) ||
      !Disjoint(output, bytes, source.records().data(),
                source.record_count() * sizeof(Type13NodeContribution)) ||
      !Disjoint(output, bytes, source.domain()->nodes().data(),
                source.domain()->node_count() * sizeof(NodalDomainNode))) {
    return false;
  }
  for (std::size_t p = 0; p < model.property_count(); ++p) {
    const auto* declaration = model.property_declaration(p);
    if (!Disjoint(output, bytes, model.property(p), sizeof(Property)) ||
        !Disjoint(output, bytes, declaration, sizeof(ModelPropertyInput))) {
      return false;
    }
    for (const auto& curve : declaration->input.curves) {
      if (!Disjoint(output, bytes, curve.points, curve.count * sizeof(CurvePoint))) {
        return false;
      }
    }
  }
  for (std::size_t e = 0; e < Count(); ++e) {
    if (!Disjoint(output, bytes, model.startup(e), sizeof(Startup))) {
      return false;
    }
  }
  return true;
}
} // namespace tl::fea::type13
