// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FENodalStateStorage.h"
#include "NodalRigidGroupStorage.h"
#include "NodalTrialIdentity.h"
#include "../math/ScalarBits.h"
#include <cmath>

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
NodalReport FENodalState::ValidateFreeTranslationalNodes(const std::size_t* nodes,
    std::size_t count) const noexcept {
  if (!impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  const auto& state=*impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  const auto n=state.stamp.node_count;
  if (count>n) return {NodalStatus::ResourceLimit,"Translation node query exceeds the actual owner"};
  const std::size_t fields=state.stamp.has_rotations ? (state.stamp.has_rotation_presence?4:3) : 1;
  if (!nodes || !count || state.constraint_staging.size()!=fields*n)
    return {NodalStatus::InvalidInput,"Translation node query requires complete immutable masks"};
  for (std::size_t i=0;i<count;++i) {
    const auto node=nodes[i];
    if (node>=n) return {NodalStatus::InvalidInput,"Translation node query index exceeds the actual owner"};
    if (state.constraint_staging[node] ||
        (state.stamp.has_rotations && state.constraint_staging[n+node]))
      return {NodalStatus::InvalidInput,"Participant requires free world translations",static_cast<std::uint32_t>(node)};
  }
  return {NodalStatus::Ok,"Actual owner nodes have free world translations"};
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
NodalReport FENodalState::ValidatePresentRotationalNodes(const std::size_t* nodes,
    std::size_t count) const noexcept {
  if (!impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  const auto& s=*impl_;
  if (!s.usable) return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  const auto n=s.stamp.node_count;
  if(count>n)return {NodalStatus::ResourceLimit,"Rotational node query exceeds owner"};
  if(!nodes||!count||!s.stamp.has_rotations||s.constraint_staging.size()!=(s.stamp.has_rotation_presence?4:3)*n)
    return {NodalStatus::InvalidInput,"Rotational node query needs complete extended owner"};
  for(std::size_t i=0;i<count;++i) {
    const auto node=nodes[i];if(node>=n)return {NodalStatus::InvalidInput,"Rotational node query exceeds owner"};
    if(s.stamp.has_rotation_presence&&s.constraint_staging[3*n+node]!=1)
      return {NodalStatus::InvalidInput,"Mapped node needs present rotational DOF",std::uint32_t(node)};
  }
  return {NodalStatus::Ok,"Actual owner nodes have present rotational DOFs"};
}
NodalReport FENodalState::ValidateInitialConstrainedTranslation(const NodalStamp& expected,
    const double* velocity,std::size_t count,tl::math::Vec3 common) const noexcept {
  if(!impl_)return {NodalStatus::NotInitialized,"Owner is not initialized"};
  const auto& s=*impl_;
  if(!s.usable)return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  if(!trial_identity::SameStamp(expected,s.stamp)||s.stamp.epoch||s.stamp.time!=0||
      s.stamp.velocity_time!=0||s.stamp.reactions_valid||!s.stamp.has_rotations||
      s.stamp.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart||
      s.stamp.velocity_phase!=NodalVelocityPhase::Collocated||s.phase!=nodal_detail::Phase::Idle)
    return {NodalStatus::StaleTrial,"Constrained startup requires fresh collocated accepted owner"};
  const auto n=s.stamp.node_count;
  if(!velocity||count!=n||s.constraint_staging.size()!=(s.stamp.has_rotation_presence?4:3)*n||
      !std::isfinite(common.x)||!std::isfinite(common.y)||!std::isfinite(common.z))
    return {NodalStatus::InvalidInput,"Constrained startup input differs from complete owner"};
  const double declared[]{common.x,common.y,common.z};
  for(std::size_t node=0;node<n;++node)for(unsigned axis=0;axis<3;++axis) {
    const double expected_velocity=(s.constraint_staging[n+node]&(1u<<axis))?0.:declared[axis];
    if(!tl::math::SameScalarBits(velocity[3*node+axis],expected_velocity))
      return {NodalStatus::InvalidInput,"Initial velocity differs from actual constrained projection",std::uint32_t(node)};
  }
  return {NodalStatus::Ok,"Initial velocity matches immutable owner fixed masks"};
}
} // namespace tl::fea
