// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidAssemblyBindingInternal.h"

namespace tl::fea::rigid_binding_detail {
Report Forecast(const rigid::NodalRigidPartAssemblyModel& parts,const NodalRigidGroupModel* plain,
    RigidBindingLimits limits,std::size_t header,Layout& out) noexcept {
  if(!parts.prepared()||(plain&&!plain->prepared()))
    return Fail(S::InvalidInput,"Prepared PART and optional plain rigid models are required");
  const RigidBindingLimits hard;
  const auto& topology=*parts.topology();
  const auto count=topology.root_count()+(plain?plain->group_count():0);
  const auto members=topology.member_count()+(plain?plain->member_count():0);
  const auto nodes=parts.coefficients()->domain()->node_count();
  if(!limits.max_groups||limits.max_groups>hard.max_groups||
      !limits.max_members||limits.max_members>hard.max_members||
      limits.max_members_per_group<2||limits.max_members_per_group>hard.max_members_per_group||
      !limits.max_nodes||limits.max_nodes>hard.max_nodes||
      !limits.max_host_bytes||limits.max_host_bytes>hard.max_host_bytes||
      count>limits.max_groups||members>limits.max_members||nodes>limits.max_nodes||members>nodes)
    return Fail(S::ResourceLimit,"Complete rigid binding inventory exceeds explicit bounds");
  if((plain?plain->member_count():0)!=topology.other_rigid_member_count()||
      (plain&&(plain->global_node_count()!=nodes||
        plain->source_units().mass_to_kg!=parts.source_units().mass_to_kg||
        plain->source_units().length_to_m!=parts.source_units().length_to_m)))
    return Fail(S::IdentityMismatch,"Plain rigid inventory, domain extent or source units differ");
  for(std::size_t g=0;g<count;++g) {
    const auto n=g<topology.root_count()?topology.roots()[g].member_count:
      plain->groups()[g-topology.root_count()].member_count;
    if(n<2||n>limits.max_members_per_group)
      return Fail(S::ResourceLimit,"Rigid root membership exceeds admitted scope",g);
  }
  Layout next;
  util::BoundedArenaLayout arena(limits.max_host_bytes),owned(limits.max_host_bytes),startup(limits.max_host_bytes);
  util::ArenaRegion unused;
  const auto backing=parts.owned_payload_bytes();
  if(backing<sizeof(parts)||!arena.Append<RigidBindingGroup>(count,next.groups)||
      !arena.Append<RigidBindingMember>(members,next.members)||
      !arena.Append<std::size_t>(members,next.lookup)||
      !owned.Append<std::byte>(header,unused)||
      !owned.Append<std::byte>(backing-sizeof(parts),unused)||
      !owned.Append<std::byte>(arena.bytes(),unused)||
      !startup.Append<std::byte>(owned.bytes(),unused)||
      !startup.Append<std::byte>(util::SourceIdentityIndex<0>::Bytes(topology.other_rigid_member_count()),unused))
    return Fail(S::ResourceLimit,"Rigid binding retained backing and startup index exceed byte cap");
  next.arena=arena.bytes();next.owned=owned.bytes();next.startup=startup.bytes();
  out=next;return {};
}
} // namespace tl::fea::rigid_binding_detail
