// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type13NodeContributions.h"
#include "NodalDomainIdentity.h"
#include "../math/Fixed3Operations.h"

namespace tl::fea::type13_contribution_detail {
inline NodalDomainReport CheckNodes(const type13::Model& model,const NodalNodeDomain& domain) noexcept {
  using S=NodalDomainStatus;
  for(std::size_t n=0;n<model.node_count();++n) {
    const auto& node=model.nodes()[n];
    const auto found=domain.Find(node.source_id);
    if(node.global_node!=SIZE_MAX&&found!=node.global_node) {
      return {S::MissingSource,"TYPE13 declared owner index and source NID differ from domain",n};
    }
  }
  return {};
}
inline NodalDomainReport CheckOrientation(const type13::Model& model,const NodalNodeDomain& domain,
    std::size_t node_index) noexcept {
  const auto& node=model.nodes()[node_index];
  const auto owner=domain.Find(node.source_id);
  if(owner==SIZE_MAX) return {}; // Reference-only source node is outside this domain.
  // Reference currently stores only two prepared SI endpoint positions. For N3
  // reuse its exact one-way Scale operation; never reconstruct native geometry.
  const auto metres=tl::math::fixed3::Scale(node.position_native,model.units().length_to_m);
  if(!tl::math::fixed3::Finite(metres)||
      !nodal_domain_detail::SamePosition(metres,domain.nodes()[owner].position)) {
    return {NodalDomainStatus::PositionMismatch,"TYPE13 orientation-node coordinate bits differ from domain",node_index};
  }
  return {};
}
} // namespace tl::fea::type13_contribution_detail
