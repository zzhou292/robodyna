// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type25BatchArena.h"

namespace tl::fea::type25::batch_detail {
bool MakeLayout(std::size_t p,std::size_t c,std::size_t n,std::size_t cap,ArenaLayout& output) noexcept {
  return MakeLayout(p,c,n,cap,output,CapacityProfile::Legacy);
}
bool MakeLayout(std::size_t p,std::size_t c,std::size_t n,std::size_t cap,ArenaLayout& output,CapacityProfile profile) noexcept {
  const auto hard=Bounds(profile);
  if(!ValidProfile(profile)||!p||p>hard.properties||!c||c>hard.connections||!n||n>hard.nodes||!cap||cap>hard.batch_device_bytes)return false;
  ArenaLayout next;util::BoundedArenaLayout layout(cap);
  if(!layout.Append<Storage>(1,next.header)||!layout.Append<Property>(p,next.properties)||
     !layout.Append<DeviceElement>(c,next.elements)||!layout.Append<DeviceNode>(n,next.nodes)||
     !layout.Append<Evaluation>(c,next.slab[0])||!layout.Append<Evaluation>(c,next.slab[1])||
     !layout.Append<Status>(c,next.status))return false;
  next.bytes=layout.bytes();output=next;return true;
}
Storage RebasedHeader(void* base,const ArenaLayout& l) noexcept {
  Storage s;s.model.properties=util::ArenaPointer<Property>(base,l.properties);
  s.model.elements=util::ArenaPointer<DeviceElement>(base,l.elements);
  s.model.nodes=util::ArenaPointer<DeviceNode>(base,l.nodes);
  for(unsigned i=0;i<2;++i)s.slab[i].element=util::ArenaPointer<Evaluation>(base,l.slab[i]);
  s.candidate_status=util::ArenaPointer<Status>(base,l.status);return s;
}
} // namespace tl::fea::type25::batch_detail
