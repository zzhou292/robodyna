#pragma once
// Extracted from qualified QephBatchIdentity.cpp; exact comparisons only.
// No element history, state ownership, authentication of fabricated pointers,
// temporal policy or CUDA work. Keep source order when adopting in QEPH.
#include "FENodalState.h"
#include <cstdint>
namespace tl::fea::trial_identity {
inline bool SameStamp(const NodalStamp& a,const NodalStamp& b) noexcept {
  return a.owner_id==b.owner_id&&a.epoch==b.epoch&&a.node_count==b.node_count&&a.time==b.time&&
    a.fixed_dt==b.fixed_dt&&a.has_rotations==b.has_rotations&&a.reactions_valid==b.reactions_valid&&
    a.reaction_base_epoch==b.reaction_base_epoch&&a.reaction_time==b.reaction_time&&
    a.temporal_scheme==b.temporal_scheme&&a.velocity_phase==b.velocity_phase&&
    a.velocity_time==b.velocity_time&&a.reaction_kick_dt==b.reaction_kick_dt;
}
inline bool SameView(const DeviceNodalKinematicsView& a,const DeviceNodalKinematicsView& b) {
  return a.position_xyz==b.position_xyz&&a.velocity_xyz==b.velocity_xyz&&
    a.angular_velocity_xyz==b.angular_velocity_xyz&&a.orientation_wxyz==b.orientation_wxyz&&
    a.node_count==b.node_count&&a.base_epoch==b.base_epoch;
}
inline bool SamePrepared(const NodalPreparedView& a,const NodalPreparedView& b) noexcept {
  return SameView(a.kinematics,b.kinematics)&&SameView(a.base_kinematics,b.base_kinematics)&&
    a.stream==b.stream&&a.owner_id==b.owner_id&&a.attempt==b.attempt&&a.proposed_time==b.proposed_time&&
    a.temporal_scheme==b.temporal_scheme&&a.velocity_phase==b.velocity_phase&&a.base_velocity_phase==b.base_velocity_phase&&
    a.base_time==b.base_time&&a.velocity_time==b.velocity_time&&a.base_velocity_time==b.base_velocity_time&&a.kick_dt==b.kick_dt;
}
// Source identity only: a retained record is not permission to dereference an
// expired view or reuse its attempt/force destinations. No device access.
inline bool SameAssemblySources(const NodalAssemblyView& a,const NodalAssemblyView& b) noexcept {
  return SameView(a.accepted,b.accepted)&&a.mass.inverse_mass==b.mass.inverse_mass&&
    a.mass.fixed==b.mass.fixed&&a.mass.node_count==b.mass.node_count&&
    a.mass.base_epoch==b.mass.base_epoch&&a.mass.model==b.mass.model&&
    a.inverse_inertia==b.inverse_inertia&&a.translation_fixed_bits==b.translation_fixed_bits&&
    a.rotation_fixed==b.rotation_fixed&&a.stream==b.stream&&a.owner_id==b.owner_id&&
    a.temporal_scheme==b.temporal_scheme&&a.velocity_phase==b.velocity_phase&&
    a.position_time==b.position_time&&a.velocity_time==b.velocity_time;
}
inline bool ValidKinematics(const DeviceNodalKinematicsView& v,std::size_t n,std::uint64_t epoch) noexcept {
  return v.node_count==n&&v.base_epoch==epoch&&v.position_xyz&&v.velocity_xyz&&
    v.angular_velocity_xyz&&v.orientation_wxyz;
}

inline bool Disjoint(const void* a,std::size_t an,const void* b,std::size_t bn) noexcept {
  const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);
  return a&&b&&an<=UINTPTR_MAX-x&&bn<=UINTPTR_MAX-y&&(x+an<=y||y+bn<=x);
}
} // namespace tl::fea::trial_identity
