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
} // namespace tl::fea
