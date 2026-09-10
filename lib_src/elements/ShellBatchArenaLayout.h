// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utils/BoundedArena.h"

namespace tl::fea::shell_batch_detail {
// Common layout arithmetic only; family records and numerical operations remain
// independently typed. No owner, selector, CUDA call or global capacity array.
template<class Storage,class Element,class Result,class Vec3,class Status>
struct BatchArenaLayout {
  util::ArenaRegion header,element,position,mass,inertia,physical,added,slab[2],status;
  std::size_t bytes=0;
  bool Initialize(std::size_t elements,std::size_t nodes,std::size_t cap) noexcept {
    BatchArenaLayout next;
    util::BoundedArenaLayout layout(cap);
    if(!elements||!nodes||!layout.template Append<Storage>(1,next.header)||
       !layout.template Append<Element>(elements,next.element)||!layout.template Append<Vec3>(nodes,next.position)||
       !layout.template Append<double>(nodes,next.mass)||!layout.template Append<double>(nodes,next.inertia)||
       !layout.template Append<double>(nodes,next.physical)||!layout.template Append<double>(nodes,next.added)||
       !layout.template Append<Result>(elements,next.slab[0])||!layout.template Append<Result>(elements,next.slab[1])||
       !layout.template Append<Status>(elements,next.status)) return false;
    next.bytes=layout.bytes(); *this=next; return true;
  }
  Storage* Construct(util::HostArena& arena) const noexcept {
    auto* output=arena.template Construct<Storage>(header);
    if(!output) return nullptr;
    output->model.element=arena.template Construct<Element>(element);
    output->model.initial_position=arena.template Construct<Vec3>(position);
    output->model.mass=arena.template Construct<double>(mass);
    output->model.inertia=arena.template Construct<double>(inertia);
    output->model.physical=arena.template Construct<double>(physical);
    output->model.added=arena.template Construct<double>(added);
    for(unsigned i=0;i<2;++i) output->slab[i].element=arena.template Construct<Result>(slab[i]);
    output->candidate_status=arena.template Construct<Status>(status);
    return output->model.element&&output->model.initial_position&&output->model.mass&&output->model.inertia&&
      output->model.physical&&output->model.added&&output->slab[0].element&&output->slab[1].element&&output->candidate_status?output:nullptr;
  }
  Storage Rebase(const Storage& host,void* device_base) const noexcept {
    Storage result=host;
    result.model.element=util::ArenaPointer<Element>(device_base,element);
    result.model.initial_position=util::ArenaPointer<Vec3>(device_base,position);
    result.model.mass=util::ArenaPointer<double>(device_base,mass);
    result.model.inertia=util::ArenaPointer<double>(device_base,inertia);
    result.model.physical=util::ArenaPointer<double>(device_base,physical);
    result.model.added=util::ArenaPointer<double>(device_base,added);
    for(unsigned i=0;i<2;++i) result.slab[i].element=util::ArenaPointer<Result>(device_base,slab[i]);
    result.candidate_status=util::ArenaPointer<Status>(device_base,status);
    return result;
  }
};
} // namespace tl::fea::shell_batch_detail
