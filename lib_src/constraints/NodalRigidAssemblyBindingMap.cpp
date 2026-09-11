// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidAssemblyBindingInternal.h"
#include <cmath>
#include <numeric>

namespace tl::fea::rigid_binding_detail {
namespace {
Report Member(Storage& s,std::size_t g,std::size_t out,std::uint64_t id,std::size_t expected=SIZE_MAX) {
  const auto& ledger=*s.parts.coefficients();
  const auto& domain=*ledger.domain();
  const auto node=domain.Find(id);
  if(node==SIZE_MAX||(expected!=SIZE_MAX&&node!=expected))
    return Fail(S::IdentityMismatch,"Rigid source NID differs from its physical domain index",g,out);
  const auto& row=ledger.nodes()[node];
  if(!HasCoefficientProducer(row.occurrences))
    return Fail(S::MissingCoefficient,"Rigid member has no real nodal coefficient producer",g,out);
  const auto& c=row.coefficients;
  if(!std::isfinite(c.mass)||c.mass<0||!std::isfinite(c.isotropic_inertia)||c.isotropic_inertia<0)
    return Fail(S::InvalidInput,"Rigid member coefficients must remain finite and nonnegative",g,out);
  s.members[out]={id,node,domain.nodes()[node].position,c.mass,c.isotropic_inertia};
  return {};
}
}
Report Bind(Storage& s,const NodalRigidGroupModel* plain) {
  const auto& topology=*s.parts.topology();
  std::size_t cursor=0;
  for(std::size_t g=0;g<topology.root_count();++g) {
    const auto& source=topology.roots()[g];
    const auto& value=s.parts.roots()[g].value;
    auto& row=s.groups[g];
    row={RigidBindingSourceKind::Part,topology.parts()[source.part_index].source_part_id,0,
      cursor,source.member_count,value.raw.mass,value.raw.center,value.principal};
    for(std::size_t k=0;k<source.member_count;++k) {
      const auto id=topology.root_members()[source.member_offset+k];
      auto report=Member(s,g,cursor,id);
      if(!report)return report;
      if(s.parts.RootForDomainNode(s.members[cursor].domain_node)!=g)
        return Fail(S::IdentityMismatch,"PART aggregate root association differs",g,cursor);
      ++cursor;
    }
  }
  util::SourceIdentityIndex<0> others;
  if(topology.other_rigid_member_count())
    others.Prepare(topology.other_rigid_member_count(),[&](std::size_t i){return topology.other_rigid_members()[i];});
  if(plain)for(std::size_t p=0;p<plain->group_count();++p) {
    const auto g=topology.root_count()+p;
    const auto& source=plain->groups()[p];
    s.groups[g]={RigidBindingSourceKind::NodalGroup,source.source_group_id,source.source_node_set_id,
      cursor,source.member_count,source.total_mass_kg,source.center,source.principal};
    for(std::size_t k=0;k<source.member_count;++k) {
      const auto& value=plain->members()[source.member_offset+k];
      if(others.First(value.source_node_id)==SIZE_MAX)
        return Fail(S::IdentityMismatch,"Plain member is absent from the complete declared other-rigid census",g,cursor);
      auto report=Member(s,g,cursor,value.source_node_id,value.global_node);
      if(!report)return report;
      const auto& member=s.members[cursor];
      if(!nodal_domain_detail::SamePosition(member.position,value.position)||
          member.mass_kg!=value.mass_kg||member.isotropic_inertia_kg_m2!=value.total_inertia_kg_m2)
        return Fail(S::IdentityMismatch,"Prepared plain geometry or coefficients differ from the common ledger",g,cursor);
      ++cursor;
    }
  }
  if(cursor!=s.member_count)return Fail(S::IdentityMismatch,"Incomplete combined rigid membership");
  std::iota(s.lookup,s.lookup+s.member_count,0);
  std::sort(s.lookup,s.lookup+s.member_count,[&](std::size_t a,std::size_t b) {
    return s.members[a].domain_node<s.members[b].domain_node;
  });
  for(std::size_t k=1;k<s.member_count;++k)
    if(s.members[s.lookup[k-1]].domain_node==s.members[s.lookup[k]].domain_node)
      return Fail(S::DuplicateMembership,"Physical node occurs in multiple combined rigid bodies",SIZE_MAX,s.lookup[k]);
  return {};
}
} // namespace tl::fea::rigid_binding_detail
