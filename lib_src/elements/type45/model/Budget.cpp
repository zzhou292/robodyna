// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"

namespace tl::fea::type45::model_detail {
ModelReport Preflight(const NodalRigidAssemblyBinding& rigid,ModelInput input,ModelLimits limits,
                       std::size_t header,Layout& output) {
  if(!rigid.prepared() || !input.source_instance_id ||
      input.source_instance_id!=rigid.domain()->source_instance_id())
    return Error(ModelStatus::SourceMismatch,"Prepared matching rigid/domain source required");
  const ModelLimits hard;
  const auto count=input.joints.size();
  if(!limits.max_joints || limits.max_joints>hard.max_joints ||
      !limits.max_nodes || limits.max_nodes>hard.max_nodes ||
      !limits.max_host_bytes || limits.max_host_bytes>hard.max_host_bytes ||
      count>limits.max_joints || rigid.domain()->node_count()>limits.max_nodes)
    return Error(ModelStatus::ResourceLimit,"Joint model extent exceeds the selected caps");
  if(!count) return Error(ModelStatus::InvalidInput,"Joint model requires at least one joint");
  util::BoundedArenaLayout arena(limits.max_host_bytes),owned(limits.max_host_bytes),startup(limits.max_host_bytes);
  util::ArenaRegion unused;
  Layout next;
  const auto backing=rigid.owned_payload_bytes();
  if(backing<sizeof(rigid) || !arena.Append<Joint>(count,next.joints) ||
      !owned.Append<std::byte>(header,unused) ||
      !owned.Append<std::byte>(backing-sizeof(rigid),unused) ||
      !owned.Append<std::byte>(arena.bytes(),unused) ||
      !startup.Append<std::byte>(owned.bytes(),unused) ||
      !startup.Append<std::byte>(Index::Bytes(count),unused))
    return Error(ModelStatus::ResourceLimit,"Complete joint model bytes exceed cap before borrowed reads");
  const auto address=reinterpret_cast<std::uintptr_t>(input.joints.data());
  if(!address || address%alignof(JointInput) || count>(UINTPTR_MAX-address)/sizeof(JointInput))
    return Error(ModelStatus::InvalidInput,"Invalid joint pointer/count range");
  next.arena=arena.bytes(); next.owned=owned.bytes(); next.startup=startup.bytes();
  output=next;
  return {};
}
} // namespace tl::fea::type45::model_detail
