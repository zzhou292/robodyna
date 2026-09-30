// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
namespace tl::fea::beam18::model_detail {
ModelReport BindParent(const NodalNodeDomain& domain,const Reference& reference,Parent& output,
    std::size_t parent) noexcept {
  Parent next; next.reference=reference;
  for(unsigned local=0;local<2;++local) {
    const auto node=domain.Find(reference.input().source_node_id[local]);
    if(node==SIZE_MAX) return {S::IdentityMismatch,"Beam endpoint NID absent from domain",parent,local};
    if(!nodal_domain_detail::SamePosition(reference.geometry().endpoint_m[local],domain.nodes()[node].position))
      return {S::PositionMismatch,"Beam endpoint SI coordinate bits differ from domain",parent,local};
    next.domain_nodes[local]=node;
  }
  const auto n3=domain.Find(reference.input().source_node_id[2]);
  if(n3!=SIZE_MAX) {
    const auto p=reference.input().position[2];
    const auto scale=reference.input().units==WorkingUnits::TonneMillimetreSecond?.001:1.;
    const Vec3 si{p.x*scale,p.y*scale,p.z*scale};
    if(!nodal_domain_detail::SamePosition(si,domain.nodes()[n3].position))
      return {S::PositionMismatch,"Present orientation N3 differs from source SI coordinates",parent,2};
  }
  output=next; return {};
}
} // namespace tl::fea::beam18::model_detail
