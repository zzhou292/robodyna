// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FENodalStateStorage.h"
#include "NodalRigidGroupStorage.h"

namespace tl::fea {
NodalReport FENodalState::ValidateNonRigidNodes(const std::size_t* nodes,std::size_t count) const noexcept {
  if(!impl_)return {NodalStatus::NotInitialized,"Owner is not initialized"};
  const auto& s=*impl_;
  if(!s.usable)return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  if(count>s.stamp.node_count)
    return {NodalStatus::ResourceLimit,"Node query count exceeds the actual owner"};
  if(!nodes||!count)return {NodalStatus::InvalidInput,"Node query requires a nonempty input range"};
  if(s.rigid_groups&&s.rigid_groups->member_nodes.size()!=s.stamp.node_count)
    return {NodalStatus::InvalidInput,"Owner rigid membership inventory is incomplete"};
  for(std::size_t i=0;i<count;++i) {
    const auto node=nodes[i];
    if(node>=s.stamp.node_count)return {NodalStatus::InvalidInput,"Node query index exceeds the actual owner",UINT32_MAX};
    if(s.rigid_groups&&s.rigid_groups->member_nodes[node])
      return {NodalStatus::InvalidInput,"Node belongs to an actual owner rigid group",static_cast<std::uint32_t>(node)};
  }
  return {NodalStatus::Ok,"OK"};
}
NodalReport FENodalState::ValidateFreeRotationalNodes(const std::size_t* nodes,
    std::size_t count) const noexcept {
  if (!impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  const auto& state=*impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  const auto n=state.stamp.node_count;
  if (count>n) return {NodalStatus::ResourceLimit,"Rotational node query exceeds the actual owner"};
  if (!nodes || !count || !state.stamp.has_rotations ||
      state.constraint_staging.size()!=(state.stamp.has_rotation_presence?4:3)*n) {
    return {NodalStatus::InvalidInput,"Rotational node query requires the complete extended owner"};
  }
  for (std::size_t i=0;i<count;++i) {
    const auto node=nodes[i];
    if (node>=n) return {NodalStatus::InvalidInput,"Rotational node query index exceeds the actual owner"};
    if (state.constraint_staging[node] || state.constraint_staging[n+node] ||
        state.constraint_staging[2*n+node] ||
        (state.stamp.has_rotation_presence && state.constraint_staging[3*n+node]!=1)) {
      return {NodalStatus::InvalidInput,"Mapped node requires present free world DOFs",static_cast<std::uint32_t>(node)};
    }
  }
  return {NodalStatus::Ok,"Actual owner nodes have present free rotational DOFs"};
}
} // namespace tl::fea
