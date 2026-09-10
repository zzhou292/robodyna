// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidGroupStorage.h"
#include "FENodalStateStorage.h"
#include "NodalTrialIdentity.h"
#include <cmath>
#include <cstring>

namespace tl::fea::nodal_detail {
RigidStorage::~RigidStorage() { if(arena) cudaFree(arena); }
cudaError_t RigidStorage::Upload(cudaStream_t stream) {
  auto error=cudaMalloc(&arena,immutable_bytes); if(error!=cudaSuccess) return error;
  auto* bytes=static_cast<unsigned char*>(arena);
  device={reinterpret_cast<const RigidGroupRange*>(bytes),reinterpret_cast<const RigidMemberMetric*>(bytes+members_offset),
    bytes+nodes_offset,static_cast<std::uint32_t>(groups.size()),static_cast<std::uint32_t>(members.size()),units.length_to_m};
  error=cudaMemcpyAsync(bytes,groups.data(),groups.size()*sizeof(RigidGroupRange),cudaMemcpyHostToDevice,stream);
  if(error!=cudaSuccess) return error;
  error=cudaMemcpyAsync(bytes+members_offset,members.data(),members.size()*sizeof(RigidMemberMetric),cudaMemcpyHostToDevice,stream);
  if(error!=cudaSuccess) return error;
  return cudaMemcpyAsync(bytes+nodes_offset,member_nodes.data(),member_nodes.size(),cudaMemcpyHostToDevice,stream);
}
void RigidStorage::InitializeState(double* tail) const noexcept {
  for(std::size_t g=0;g<properties.size();++g)
    rigid::WriteGroupState(tail+rigid::GroupStateValues*g,
      {properties[g].center,initial_velocity,{},properties[g].principal.axes});
}

NodalReport PrepareRigidStorage(const NodalRigidGroupModel& model,const NodalStateConfig& config,
    HostNodalKinematicsView input,const double* inverse_mass,const NodalDofConfig& dofs,
    std::unique_ptr<RigidStorage>& output) {
  if(!model.prepared()||model.global_node_count()!=config.node_count||!model.group_count())
    return {NodalStatus::InvalidInput,"Rigid model is not complete for this node inventory"};
  if(config.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart)
    return {NodalStatus::UnsupportedTemporalScheme,"Rigid groups require staggered physical initialization"};
  if(model.group_count()>MaxOwnerRigidGroups||model.member_count()>config.node_count)
    return {NodalStatus::ResourceLimit,"Rigid model exceeds owner group/member limits"};
  auto next=std::make_unique<RigidStorage>();
  next->info={model.source_instance_id(),model.group_count(),model.member_count()};
  next->units=model.source_units();
  next->properties.assign(model.groups(),model.groups()+model.group_count());
  next->source_members.assign(model.members(),model.members()+model.member_count());
  next->groups.reserve(model.group_count()); next->members.reserve(model.member_count());
  next->member_nodes.resize(config.node_count,0); next->snapshots.resize(model.group_count());
  const auto first=model.members()[0].global_node;
  next->initial_velocity={input.velocity_xyz[3*first],input.velocity_xyz[3*first+1],input.velocity_xyz[3*first+2]};
  for(const auto& group:next->properties) {
    if(group.member_count<2||group.member_count>MaxOwnerRigidMembersPerGroup)
      return {NodalStatus::ResourceLimit,"Rigid group requires 2 to 256 complete members"};
    next->groups.push_back({static_cast<std::uint32_t>(group.member_offset),static_cast<std::uint32_t>(group.member_count),
                           group.total_mass_kg,group.principal.inertia});
  }
  for(const auto& member:next->source_members) {
    const auto i=member.global_node;
    if(i>=config.node_count||next->member_nodes[i]||dofs.translation_fixed_bits[i]||dofs.rotation_fixed[i]||
        inverse_mass[i]!=1/member.mass_kg||dofs.inverse_inertia[i]!=1/member.total_inertia_kg_m2)
      return {NodalStatus::InvalidInput,"Rigid member mass/inertia or free-DOF association differs from owner",static_cast<std::uint32_t>(i)};
    const double x[]{member.position.x,member.position.y,member.position.z};
    const double v[]{next->initial_velocity.x,next->initial_velocity.y,next->initial_velocity.z};
    for(unsigned a=0;a<3;++a)
      if(input.position_xyz[3*i+a]!=x[a]||input.velocity_xyz[3*i+a]!=v[a]||
          (input.angular_velocity_xyz&&input.angular_velocity_xyz[3*i+a]!=0))
        return {NodalStatus::InvalidInput,"Rigid startup requires exact reference positions and uniform translation with zero spin",static_cast<std::uint32_t>(i)};
    next->member_nodes[i]=1;
    next->members.push_back({static_cast<std::uint32_t>(i),member.mass_kg,member.total_inertia_kg_m2});
  }
  // Counts were bounded above; aligned native structs precede byte membership.
  static_assert(sizeof(RigidGroupRange)%alignof(RigidMemberMetric)==0,"Aligned immutable member array");
  next->members_offset=next->groups.size()*sizeof(RigidGroupRange);
  next->nodes_offset=next->members_offset+next->members.size()*sizeof(RigidMemberMetric);
  next->immutable_bytes=next->nodes_offset+next->member_nodes.size();
  output=std::move(next); return {NodalStatus::Ok,"Rigid startup association prepared"};
}
} // namespace tl::fea::nodal_detail

namespace tl::fea {
NodalReport FENodalState::Impl::StageRigidSnapshot(const double* state) {
  auto& r=*rigid_groups; const auto count=r.info.group_count;
  const auto offset=19*config.node_count,values=rigid::GroupStateValues*count;
  auto report=Check(cudaMemcpyAsync(staging.data()+offset,state+offset,values*sizeof(double),cudaMemcpyDeviceToHost,stream));
  if(report.status!=NodalStatus::Ok) return report;
  report=Check(cudaStreamSynchronize(stream)); if(report.status!=NodalStatus::Ok) return report;
  for(std::size_t g=0;g<count;++g) {
    const auto value=rigid::ReadGroupState(staging.data()+offset+rigid::GroupStateValues*g);
    if(!rigid::ValidGroupState(value)) return Reject(NodalStatus::InvalidOutput,"Rigid state readback is invalid");
    r.snapshots[g]={r.properties[g].source_group_id,r.properties[g].source_node_set_id,value};
  }
  return {NodalStatus::Ok,"Rigid snapshot staged"};
}
NodalRigidGroupInfo FENodalState::rigid_groups() const noexcept {
  return impl_&&impl_->rigid_groups?impl_->rigid_groups->info:NodalRigidGroupInfo{};
}
NodalReport FENodalState::CopyAcceptedRigidGroups(NodalRigidGroupSnapshotBuffer out,NodalStamp* stamp) {
  if(!impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  auto& s=*impl_;
  if(!s.usable) return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  if(!s.rigid_groups) return {NodalStatus::InvalidInput,"Owner has no rigid groups"};
  auto& r=*s.rigid_groups; const auto count=r.info.group_count;
  if(!out.groups||!stamp) return {NodalStatus::InvalidInput,"Missing rigid snapshot output"};
  if(out.capacity_groups<count) return {NodalStatus::ResourceLimit,"Rigid snapshot capacity is insufficient"};
  const auto bytes=count*sizeof(NodalRigidGroupSnapshot);
  if(!trial_identity::Disjoint(out.groups,bytes,stamp,sizeof(*stamp)))
    return {NodalStatus::InvalidInput,"Rigid snapshot outputs overlap or overflow"};
  const auto report=s.StageRigidSnapshot(s.accepted); if(report.status!=NodalStatus::Ok) return report;
  std::memcpy(out.groups,r.snapshots.data(),bytes); *stamp=s.stamp;
  return {NodalStatus::Ok,"Accepted rigid snapshot copied"};
}
NodalReport FENodalState::CopyPreparedRigidGroups(const NodalTrialToken& token,NodalRigidGroupSnapshotBuffer out,
    NodalPreparedView* prepared) {
  if(!impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  auto& s=*impl_;
  if(!s.usable) return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  if(!s.rigid_groups) return {NodalStatus::InvalidInput,"Owner has no rigid groups"};
  auto& r=*s.rigid_groups;
  if(!out.groups||!prepared) return {NodalStatus::InvalidInput,"Missing rigid candidate output"};
  if(out.capacity_groups<r.info.group_count) return {NodalStatus::ResourceLimit,"Rigid candidate capacity is insufficient"};
  const auto bytes=r.info.group_count*sizeof(NodalRigidGroupSnapshot);
  if(!trial_identity::Disjoint(out.groups,bytes,prepared,sizeof(*prepared))||
      !trial_identity::Disjoint(out.groups,bytes,&token,sizeof(token))||
      !trial_identity::Disjoint(prepared,sizeof(*prepared),&token,sizeof(token)))
    return {NodalStatus::InvalidInput,"Rigid candidate outputs overlap each other or the token, or overflow"};
  NodalPreparedView next;
  auto report=BorrowPrepared(token,&next); if(report.status!=NodalStatus::Ok) return report;
  report=s.StageRigidSnapshot(s.trial); if(report.status!=NodalStatus::Ok) return report;
  std::memcpy(out.groups,r.snapshots.data(),bytes); *prepared=next;
  return {NodalStatus::Ok,"Prepared rigid snapshot copied"};
}
} // namespace tl::fea
