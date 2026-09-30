// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FENodalState.h"
#include "NodalRigidStorageLayout.h"
#include "../constraints/NodalRigidGroupModel.h"
#include "../constraints/NodalRigidAssemblyBinding.h"
#include "../constraints/NodalRigidGroupState.h"
#include <vector>

namespace tl::fea::nodal_detail {
constexpr std::size_t MaxOwnerRigidMembersPerGroup=MaxRigidMembersPerGroup;
using RigidGroupRange=rigid::GroupRange;
using RigidMemberMetric=rigid::MemberMetric;
using RigidDeviceView=rigid::GroupDeviceView;
// Only immutable device metadata is separate. Evolving group values are a
// tail of the SAME nodal accepted/trial slabs and use their existing swap.
// Host staging and copied full source metadata are sized once at startup.
struct RigidStorage {
  ~RigidStorage();
  RigidStorage()=default;
  RigidStorage(const RigidStorage&)=delete;
  RigidStorage& operator=(const RigidStorage&)=delete;
  NodalRigidGroupInfo info;
  NodalRigidSourceUnits units;
  std::vector<RigidBindingGroup> properties;
  std::vector<RigidBindingMember> source_members;
  std::vector<RigidGroupRange> groups;
  std::vector<RigidMemberMetric> members;
  std::vector<std::uint8_t> member_nodes;
  std::vector<NodalRigidGroupSnapshot> snapshots;
  tl::math::Vec3 initial_velocity{};
  void* arena=nullptr;
  std::size_t immutable_bytes=0,members_offset=0,nodes_offset=0,owned_host_bytes=0;
  RigidDeviceView device;
  cudaError_t Upload(cudaStream_t);
  void InitializeState(double*) const noexcept;
};
NodalReport ForecastRigidStorage(const NodalRigidGroupModel&,const NodalStateConfig&,RigidStorageLayout&) noexcept;
NodalReport ForecastRigidStorage(const NodalRigidAssemblyBinding&,const NodalStateConfig&,RigidStorageLayout&) noexcept;
NodalReport PrepareRigidStorage(const NodalRigidGroupModel&,const NodalStateConfig&,
    HostNodalKinematicsView,const double* inverse_mass,const NodalDofConfig&,const RigidStorageLayout&,
    std::unique_ptr<RigidStorage>& output);
NodalReport PrepareRigidStorage(const NodalRigidAssemblyBinding&,const NodalStateConfig&,
    HostNodalKinematicsView,const double* inverse_mass,const NodalDofConfig&,const RigidStorageLayout&,
    std::unique_ptr<RigidStorage>& output);
NodalReport ValidateRigidAssemblyOwner(const NodalRigidAssemblyBinding&,HostNodalKinematicsView,
    const double*,const NodalDofConfig&,bool cin) noexcept;
// Shared compact source association and range construction after typed startup.
NodalReport CompleteRigidStorage(const NodalStateConfig&,HostNodalKinematicsView,const double*,
    const NodalDofConfig&,const RigidStorageLayout&,RigidStorage&);
} // namespace tl::fea::nodal_detail
