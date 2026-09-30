// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidGroupStorage.h"
#include "FENodalStateStorage.h"
#include "NodalTrialIdentity.h"
#include <cmath>
#include <cstring>

namespace tl::fea::nodal_detail {
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

NodalReport ForecastRigidStorage(const NodalRigidGroupModel& model,const NodalStateConfig& config,
    RigidStorageLayout& output) noexcept {
  if(!model.prepared()||model.global_node_count()!=config.node_count||!model.group_count())
    return {NodalStatus::InvalidInput,"Rigid model is not complete for this node inventory"};
  if(model.physical_coefficients())
    return {NodalStatus::InvalidInput,"Physical rigid coefficients require the complete assembly binding"};
  if(config.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart)
    return {NodalStatus::UnsupportedTemporalScheme,"Rigid groups require staggered physical initialization"};
  if (config.rigid_limits.profile != NodalRigidOwnerProfile::PlainGroups)
    return {NodalStatus::ResourceLimit,"Plain groups require plain owner limits"};
  RigidStorageLayout next;
  if(!next.Initialize(config.node_count,model.group_count(),model.member_count(),config.rigid_limits,sizeof(RigidStorage)))
    return {NodalStatus::ResourceLimit,"Rigid count or host payload exceeds explicit owner limits"};
  for(std::size_t g=0;g<model.group_count();++g)
    if(model.groups()[g].member_count<2||model.groups()[g].member_count>MaxOwnerRigidMembersPerGroup)
      return {NodalStatus::ResourceLimit,"Rigid group requires 2 to 256 complete members"};
  output=next;return {NodalStatus::Ok,"Rigid storage forecast prepared"};
}
NodalReport PrepareRigidStorage(const NodalRigidGroupModel& model,const NodalStateConfig& config,
    HostNodalKinematicsView input,const double* inverse_mass,const NodalDofConfig& dofs,
    const RigidStorageLayout& layout,std::unique_ptr<RigidStorage>& output) {
  auto next=std::make_unique<RigidStorage>();
  next->info={model.source_instance_id(),model.group_count(),model.member_count()};
  next->units=model.source_units();
  next->properties.reserve(model.group_count());
  next->source_members.reserve(model.member_count());
  for(std::size_t g=0;g<model.group_count();++g) {
    const auto& p=model.groups()[g];
    next->properties.push_back({RigidBindingSourceKind::NodalGroup,p.source_group_id,p.source_node_set_id,
      p.member_offset,p.member_count,p.total_mass_kg,p.center,p.principal});
  }
  for(std::size_t n=0;n<model.member_count();++n) {
    const auto& m=model.members()[n];
    next->source_members.push_back({m.source_node_id,m.global_node,m.position,m.mass_kg,m.total_inertia_kg_m2});
  }
  const auto report=CompleteRigidStorage(config,input,inverse_mass,dofs,layout,*next);
  if(report.status!=NodalStatus::Ok)return report;
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
    r.snapshots[g]={r.properties[g].source_id,r.properties[g].source_node_set_id,value,r.properties[g].source_kind};
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
