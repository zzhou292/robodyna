// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Arena.h"

namespace tl::fea::type45::resident_detail {
bool MakeLayout(std::size_t count,const BatchConfig& config,ArenaLayout& output) noexcept {
  const auto& l=config.limits;
  const BatchLimits hard;
  if(!l.max_joints || l.max_joints>hard.max_joints || !l.max_nodes || l.max_nodes>hard.max_nodes ||
      !l.max_host_bytes || l.max_host_bytes>hard.max_host_bytes ||
      !l.max_device_bytes || l.max_device_bytes>hard.max_device_bytes ||
      !count || count>l.max_joints || !config.owner.node_count || config.owner.node_count>l.max_nodes ||
      !config.owner.rigid_groups.group_count || config.owner.rigid_groups.group_count>1024) return false;
  util::BoundedArenaLayout device(l.max_device_bytes),host(l.max_host_bytes);
  ArenaLayout next;
  if(!device.Append<Storage>(1,next.header) || !device.Append<Joint>(count,next.joints) ||
      !device.Append<State>(count,next.slab[0]) || !device.Append<State>(count,next.slab[1]) ||
      !device.Append<AutomaticStiffnessContext>(count,next.contexts) || !device.Append<Status>(count,next.status) ||
      !host.Append<State>(count,next.staging) || !host.Append<AutomaticStiffnessContext>(count,next.host_contexts) ||
      !host.Append<NodalCinPhysicalMain>(config.owner.rigid_groups.group_count,next.mains) ||
      !shell_physical_owner::ForecastProof(config.owner.node_count,config.cin_attachment_count,
          l.max_host_bytes,next.proof)) return false;
  next.bytes=device.bytes();next.staging_bytes=host.bytes();output=next;return true;
}
Storage RebasedHeader(void* base,const ArenaLayout& layout) noexcept {
  Storage next;
  next.joints=util::ArenaPointer<Joint>(base,layout.joints);
  for(unsigned i=0;i<2;++i) next.slab[i]=util::ArenaPointer<State>(base,layout.slab[i]);
  next.contexts=util::ArenaPointer<AutomaticStiffnessContext>(base,layout.contexts);
  next.status=util::ArenaPointer<Status>(base,layout.status);
  next.count=layout.joints.count;
  return next;
}
} // namespace tl::fea::type45::resident_detail
