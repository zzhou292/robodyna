// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../../solid18/Solid18History.h"
#include "../../solid24/Solid24ForceHistory.h"
#include "../../solid6z/Solid6zForceInitialize.h"
#include "../../solid18/law44/History.h"
#include "../../solid18/total_strain/ForceChecks.h"

namespace tl::fea::solids::model_detail {
namespace {
std::size_t* PrepareParent(const Input18& input, std::size_t local,
    std::size_t material, Storage& out) {
  auto& parent = out.parent18[local];
  parent.reference = input.reference;
  parent.material_index = material;
  solid18::History initial;
  return solid18::InitializeHistory(parent.reference, out.material36[material].value, initial) == solid18::Status::Success
      ? parent.domain_nodes : nullptr;
}
std::size_t* PrepareParent(const Input24& input, std::size_t local,
    std::size_t material, Storage& out) {
  auto& parent = out.parent24[local];
  parent.reference = input.reference;
  parent.material_index = material;
  solid24::History initial;
  return solid24::InitializeHistory(parent.reference, out.material42[material].value, initial) == solid24::ForceStatus::Success
      ? parent.domain_nodes : nullptr;
}
std::size_t* PrepareParent(const Input6z& input, std::size_t local,
    std::size_t material, Storage& out) {
  auto& parent = out.parent6z[local];
  parent.reference = input.reference;
  parent.profile = input.profile;
  parent.material_index = material;
  solid6z::History initial;
  return solid6z::InitializeHistory(parent.reference, out.material42[material].value, parent.profile, initial) == solid6z::Status::Success
      ? parent.domain_nodes : nullptr;
}
std::size_t* PrepareParent(const Input18Law44& input, std::size_t local,
    std::size_t material, Storage& out) {
  auto& parent = out.parent44[local];
  parent.reference = input.reference;
  parent.material_index = material;
  solid18::law44::History initial;
  return solid18::law44::InitializeHistory(parent.reference, out.material44[material].value, initial) == solid18::Status::Success
      ? parent.domain_nodes : nullptr;
}
std::size_t* PrepareParent(const Input18Law90& input, std::size_t local,
    std::size_t material, Storage& out) {
  auto& parent = out.parent90[local];
  parent.reference = input.reference;
  parent.material_index = material;
  // Prepared material/reference scope only. Actual TIME0 force/history belongs
  // to InitializeForce90 and is not fabricated by an immutable model.
  return solid18::total_strain::force_detail::ValidMaterial(parent.reference, out.material90[material].value)
      ? parent.domain_nodes : nullptr;
}
}
ModelReport CopyParents(ModelInput input, const Scratch& scratch,
    const SolidNodeContributions& coefficients, Storage& out) {
  const auto rows = coefficients.parents();
  for (std::size_t i = 0; i < rows.size(); ++i) {
    auto* nodes = Visit(input, i, [&](const auto& parent, std::size_t local) {
      return PrepareParent(parent, local, scratch.material_indices[i], out);
    });
    if (!nodes) return Error(ModelStatus::InvalidInput, "Solid reference/material/profile is unsupported", input, i);
    for (unsigned n = 0; n < 8; ++n) nodes[n] = n < rows[i].node_count ? rows[i].domain_node[n] : SIZE_MAX;
  }
  return {};
}
} // namespace tl::fea::solids::model_detail
