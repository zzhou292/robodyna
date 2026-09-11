// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidGroupStorage.h"

namespace tl::fea::nodal_detail {
RigidStorage::~RigidStorage() { if (arena) cudaFree(arena); }
void RigidStorage::InitializeState(double* tail) const noexcept {
  for (std::size_t g=0;g<properties.size();++g)
    rigid::WriteGroupState(tail+rigid::GroupStateValues*g,
      {properties[g].center,initial_velocity,{},properties[g].principal.axes});
}

NodalReport ForecastRigidStorage(const NodalRigidAssemblyBinding& binding,const NodalStateConfig& config,
    RigidStorageLayout& output) noexcept {
  if (!binding.prepared()||binding.domain()->node_count()!=config.node_count||binding.groups().size()==0)
    return {NodalStatus::InvalidInput,"Prepared rigid binding differs from the physical owner extent"};
  if(config.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart)
    return {NodalStatus::UnsupportedTemporalScheme,"Combined rigid bodies require staggered initialization"};
  if(config.rigid_limits.profile!=NodalRigidOwnerProfile::PartAssembly)
    return {NodalStatus::ResourceLimit,"Combined rigid bodies require explicit assembly owner limits"};
  RigidStorageLayout next;
  if (!next.Initialize(config.node_count,binding.groups().size(),binding.members().size(),
      config.rigid_limits,sizeof(RigidStorage)))
    return {NodalStatus::ResourceLimit,"Combined rigid body counts or storage exceed owner limits"};
  for (const auto& group : binding.groups()) {
    if (group.member_count < 2 || group.member_count > MaxRigidAssemblyMembersPerGroup)
      return {NodalStatus::ResourceLimit,"Assembly group exceeds its explicit member capacity"};
  }
  output = next;
  return {NodalStatus::Ok,"Combined rigid storage forecast prepared"};
}
NodalReport PrepareRigidStorage(const NodalRigidAssemblyBinding& binding,const NodalStateConfig& config,
    HostNodalKinematicsView input,const double* inverse_mass,const NodalDofConfig& dofs,
    const RigidStorageLayout& layout,std::unique_ptr<RigidStorage>& output) {
  auto next=std::make_unique<RigidStorage>();
  next->info={binding.parts()->topology()->source_instance_id(),binding.groups().size(),binding.members().size(),
    binding.parts()->roots().size(),binding.plain_source_instance_id()};
  next->units=binding.parts()->source_units();
  next->properties.assign(binding.groups().begin(),binding.groups().end());
  next->source_members.assign(binding.members().begin(),binding.members().end());
  const auto report = CompleteRigidStorage(config,input,inverse_mass,dofs,layout,*next);
  if (report.status!=NodalStatus::Ok)return report;
  output = std::move(next);
  return {NodalStatus::Ok,"Combined rigid source association prepared"};
}

NodalReport CompleteRigidStorage(const NodalStateConfig& config,HostNodalKinematicsView input,
    const double* inverse_mass,const NodalDofConfig& dofs,const RigidStorageLayout& layout,RigidStorage& next) {
  if (next.properties.empty() || next.source_members.empty() ||
      next.source_members.front().domain_node >= config.node_count)
    return {NodalStatus::InvalidInput,"Compact rigid source has no valid first member"};
  next.groups.reserve(next.properties.size());
  next.members.reserve(next.source_members.size());
  next.member_nodes.resize(config.node_count,0);
  next.snapshots.resize(next.properties.size());
  const auto first = next.source_members[0].domain_node;
  next.initial_velocity={input.velocity_xyz[3*first],input.velocity_xyz[3*first+1],input.velocity_xyz[3*first+2]};
  std::size_t member_offset = 0;
  for (const auto& group:next.properties) {
    if (group.member_offset != member_offset || group.member_offset > next.source_members.size() ||
        group.member_count > next.source_members.size()-group.member_offset)
      return {NodalStatus::InvalidInput,"Compact rigid member ranges are incomplete"};
    member_offset += group.member_count;
    const bool part = group.source_kind==RigidBindingSourceKind::Part;
    const bool dependent = group.dependent_coefficients;
    next.groups.push_back({std::uint32_t(group.member_offset),std::uint32_t(group.member_count),
      group.mass_kg,group.principal.inertia,dependent});
    for (std::size_t k=0;k<group.member_count;++k) {
      const auto& member=next.source_members[group.member_offset+k];
      const auto i = member.domain_node;
      const double expected_mass=dependent&&member.mass_kg==0?0:1/member.mass_kg;
      const double expected_j=dependent&&member.isotropic_inertia_kg_m2==0?0:1/member.isotropic_inertia_kg_m2;
      if(i>=config.node_count||next.member_nodes[i]||dofs.translation_fixed_bits[i]||dofs.rotation_fixed[i]||
          (dofs.rotation_present&&!dofs.rotation_present[i])||
          inverse_mass[i]!=expected_mass||dofs.inverse_inertia[i]!=expected_j)
        return {NodalStatus::InvalidInput,"Rigid member coefficients or free kinematic DOFs differ",std::uint32_t(i)};
      const double x[]{member.position.x,member.position.y,member.position.z};
      const double v[]{next.initial_velocity.x,next.initial_velocity.y,next.initial_velocity.z};
      for (unsigned a=0;a<3;++a)
        if(input.position_xyz[3*i+a]!=x[a]||input.velocity_xyz[3*i+a]!=v[a]||
            (input.angular_velocity_xyz&&input.angular_velocity_xyz[3*i+a]!=0))
          return {NodalStatus::InvalidInput,"Rigid startup requires source coordinates and uniform translation with zero spin",std::uint32_t(i)};
      next.member_nodes[i]=part?rigid::PartMemberNode:
          (dependent?rigid::PhysicalPlainMemberNode:rigid::PlainMemberNode);
      next.members.push_back({std::uint32_t(i),member.mass_kg,member.isotropic_inertia_kg_m2});
    }
  }
  if (member_offset != next.source_members.size())
    return {NodalStatus::InvalidInput,"Compact rigid source has trailing members"};
  if (!RigidHostPayload(sizeof(RigidStorage),next.properties.capacity(),next.source_members.capacity(),
      next.groups.capacity(),next.members.capacity(),next.member_nodes.capacity(),next.snapshots.capacity(),
      config.rigid_limits.max_host_bytes,next.owned_host_bytes))
    return {NodalStatus::ResourceLimit,"Actual compact rigid storage exceeds the host payload budget"};
  next.members_offset = layout.members.offset;
  next.nodes_offset = layout.node_mask.offset;
  next.immutable_bytes=layout.device_bytes;
  return {NodalStatus::Ok,"Compact rigid association prepared"};
}
} // namespace tl::fea::nodal_detail
