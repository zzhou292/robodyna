#pragma once
#include "NodalRigidOwnerLimits.h"
#include "NodalStateLimits.h"
#include "../constraints/NodalRigidGroupModel.h"
#include "../constraints/NodalRigidGroupState.h"
#include "lib_utils/BoundedArena.h"

namespace tl::fea::nodal_detail {
// Each host vector is a separate allocation. Count its actual payload without
// artificial inter-vector alignment. Allocator metadata is outside this ledger.
inline bool RigidHostPayload(std::size_t object_bytes,std::size_t properties,
    std::size_t source_members,std::size_t groups,std::size_t members,
    std::size_t node_mask,std::size_t snapshots,std::size_t cap,std::size_t& bytes) noexcept {
  util::BoundedArenaLayout budget(cap);util::ArenaRegion unused;
  const std::size_t counts[]{1,properties,source_members,groups,members,node_mask,snapshots};
  const std::size_t widths[]{object_bytes,sizeof(NodalRigidGroupProperties),sizeof(NodalRigidGroupMember),
    sizeof(rigid::GroupRange),sizeof(rigid::MemberMetric),1,sizeof(NodalRigidGroupSnapshot)};
  for(unsigned i=0;i<7;++i) {
    if(widths[i]&&counts[i]>SIZE_MAX/widths[i])return false;
    if(!budget.Append<std::byte>(counts[i]*widths[i],unused))return false;
  }
  bytes=budget.bytes();return true;
}
struct RigidStorageLayout {
  util::ArenaRegion groups,members,node_mask;
  std::size_t device_bytes=0,host_bytes=0;
  bool Initialize(std::size_t nodes,std::size_t group_count,std::size_t member_count,
      NodalRigidOwnerLimits limits,std::size_t object_bytes) noexcept {
    if(!nodes||nodes>MaxActiveNodalStateNodes||!group_count||!member_count||!object_bytes||
        !limits.max_groups||limits.max_groups>MaxActiveRigidGroups||
        !limits.max_members||limits.max_members>MaxActiveRigidMembers||
        !limits.max_host_bytes||limits.max_host_bytes>MaxRigidOwnerHostBytes||
        group_count>limits.max_groups||member_count>limits.max_members||member_count>nodes||
        group_count>member_count/2||member_count>group_count*MaxRigidMembersPerGroup)return false;
    RigidStorageLayout next;util::BoundedArenaLayout device(MaxActiveNodalStateDeviceBytes);
    if(!device.Append<rigid::GroupRange>(group_count,next.groups)||
        !device.Append<rigid::MemberMetric>(member_count,next.members)||
        !device.Append<std::uint8_t>(nodes,next.node_mask)||
        !RigidHostPayload(object_bytes,group_count,member_count,group_count,member_count,
          nodes,group_count,limits.max_host_bytes,next.host_bytes))return false;
    next.device_bytes=device.bytes();*this=next;return true;
  }
};
} // namespace tl::fea::nodal_detail
