// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidGroupMath.h"
#include "NodalRigidAssemblyTypes.h"
#include <cstdint>

namespace tl::fea {
// Values only. At an accepted endpoint the principal frame belongs to the
// previous force stage; center is at the endpoint and v/omega at its midpoint.
// Initialization is the physical, collocated exception. The owner's stamp
// describes these phases; this object owns no clock or transaction identity.
struct NodalRigidGroupState {
  tl::math::Vec3 center{},velocity{},omega{};
  tl::math::Matrix3 principal_axes{};
};
struct NodalRigidGroupSnapshot {
  std::uint64_t source_group_id=0,source_node_set_id=0;
  NodalRigidGroupState state{};
  RigidBindingSourceKind source_kind=RigidBindingSourceKind::NodalGroup;
};
struct NodalRigidGroupSnapshotBuffer {
  NodalRigidGroupSnapshot* groups=nullptr;
  std::size_t capacity_groups=0;
};
struct NodalRigidGroupInfo {
  std::uint64_t source_instance_id=0;
  std::size_t group_count=0,member_count=0;
  std::size_t part_group_count=0;
  std::uint64_t plain_source_instance_id=0;
};
inline bool SameRigidGroupInfo(NodalRigidGroupInfo a,NodalRigidGroupInfo b) noexcept {
  return a.source_instance_id==b.source_instance_id&&a.group_count==b.group_count&&a.member_count==b.member_count&&
    a.part_group_count==b.part_group_count&&a.plain_source_instance_id==b.plain_source_instance_id;
}
namespace rigid {
constexpr std::size_t GroupStateValues=18;
inline constexpr std::uint8_t PlainMemberNode = 1;
inline constexpr std::uint8_t PartMemberNode = 2;
struct GroupRange {
  std::uint32_t offset=0,count=0;
  double mass=0;
  tl::math::Vec3 principal_inertia{};
  bool dependent_coefficients=false;
};
struct MemberMetric { std::uint32_t node=0; double mass=0,inertia=0; };
struct GroupDeviceView {
  const GroupRange* groups=nullptr;
  const MemberMetric* members=nullptr;
  const std::uint8_t* member_nodes=nullptr;
  std::uint32_t group_count=0,member_count=0;
  double source_length_to_m=0; // Required only by the two-member finite-rotation switch.
};
#if defined(__CUDACC__)
#define TL_RIGID_STATE_HD __host__ __device__
#else
#define TL_RIGID_STATE_HD
#endif
TL_RIGID_STATE_HD inline NodalRigidGroupState ReadGroupState(const double* p) {
  NodalRigidGroupState s;
  s.center={p[0],p[1],p[2]}; s.velocity={p[3],p[4],p[5]}; s.omega={p[6],p[7],p[8]};
  for(unsigned i=0;i<9;++i) s.principal_axes.v[i]=p[9+i];
  return s;
}
TL_RIGID_STATE_HD inline void WriteGroupState(double* p,const NodalRigidGroupState& s) {
  p[0]=s.center.x; p[1]=s.center.y; p[2]=s.center.z;
  p[3]=s.velocity.x; p[4]=s.velocity.y; p[5]=s.velocity.z;
  p[6]=s.omega.x; p[7]=s.omega.y; p[8]=s.omega.z;
  for(unsigned i=0;i<9;++i) p[9+i]=s.principal_axes.v[i];
}
TL_RIGID_STATE_HD inline bool ValidGroupState(const NodalRigidGroupState& s) {
  return detail::Finite(s.center)&&detail::Finite(s.velocity)&&detail::Finite(s.omega)&&
    detail::Orthonormal(s.principal_axes);
}
#undef TL_RIGID_STATE_HD
} // namespace rigid
} // namespace tl::fea
