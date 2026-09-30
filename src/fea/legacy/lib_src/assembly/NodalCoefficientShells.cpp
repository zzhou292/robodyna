// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCoefficientInternal.h"

namespace tl::fea::coefficient_detail {
namespace {
template<class Reference,std::size_t N>
CoefficientReport Parent(const ShellNodeMap& map,const Reference& reference,
    const std::array<std::size_t,N>& indices,P producer,std::size_t parent,
    NodalCoefficientNode* rows) noexcept {
  for(std::size_t local=0;local<N;++local) {
    const auto node=map.owner_index(indices[local]);
    if(node>=map.owner_node_count())
      return {S::IdentityMismatch,"Shell owner map index differs",producer,parent,local,node};
    auto& row=rows[node];
    auto& c=row.coefficients;
    const auto mass=reference.nodal_mass[local],j=reference.isotropic_inertia[local];
    const auto physical=reference.physical_inertia[local],added=reference.added_inertia[local];
    if(!Positive(mass)||!Positive(j)||!Positive(physical)||!Positive(added)||
        !AddPair(c.mass,c.isotropic_inertia,mass,j)||
        !AddPair(c.shell.mass,c.shell.isotropic_inertia,mass,j)||
        !Add(c.shell.physical_inertia,physical)||!Add(c.shell.added_inertia,added))
      return {S::NonfiniteResult,"Native shell coefficient addition failed",producer,parent,local,node};
    if(producer==P::Qeph) ++row.occurrences.qeph;
    else if(producer==P::T3) ++row.occurrences.t3;
    else ++row.occurrences.qbat;
  }
  return {};
}
}
CoefficientReport Shells(const ShellNodeMap& map,NodalCoefficientNode* nodes) noexcept {
  const auto& binding=*map.shells();
  for(std::size_t i=0;i<binding.qeph_count();++i) {
    const auto r=Parent(map,binding.qeph_reference(i),binding.qeph_nodes(i),P::Qeph,i,nodes);
    if(!r) return r;
  }
  for(std::size_t i=0;i<binding.t3_count();++i) {
    const auto r=Parent(map,binding.t3_reference(i),binding.t3_nodes(i),P::T3,i,nodes);
    if(!r) return r;
  }
  for(std::size_t i=0;i<binding.qbat_count();++i) {
    const auto r=Parent(map,binding.qbat_reference(i).quadrilateral(),binding.qbat_nodes(i),P::Qbat,i,nodes);
    if(!r) return r;
  }
  return {};
}
} // namespace tl::fea::coefficient_detail
