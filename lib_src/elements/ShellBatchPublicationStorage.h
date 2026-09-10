// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchPublication.h"
#include "ShellBatchBinding.h"
#include "ShellResidentLimits.h"
#include "lib_utils/BoundedArena.h"
#include <type_traits>

namespace tl::fea::shell_publication_detail {
struct Model {
  std::size_t node_count=0;
  double *mass=nullptr,*inertia=nullptr,*physical=nullptr,*added=nullptr;
};
struct Control {
  ShellPublicationStatus status=ShellPublicationStatus::Success;
  ShellBatchKinetic base,endpoint;
};
struct Storage { Model model; Control control; };
static_assert(std::is_trivially_copyable_v<Storage>,"Only native mass and scalar kinetic scratch");
static_assert(sizeof(Storage)<256,"Only the active native-mass pointer header and scalar scratch");
struct Layout {
  util::ArenaRegion header,mass,inertia,physical,added;
  std::size_t bytes=0;
  bool Initialize(std::size_t nodes,std::size_t cap) noexcept {
    if(!nodes||nodes>MaxShellResidentNodes) return false;
    Layout next; util::BoundedArenaLayout layout(cap);
    if(!layout.Append<Storage>(1,next.header)||!layout.Append<double>(nodes,next.mass)||
       !layout.Append<double>(nodes,next.inertia)||!layout.Append<double>(nodes,next.physical)||
       !layout.Append<double>(nodes,next.added)) return false;
    next.bytes=layout.bytes(); *this=next; return true;
  }
  Storage* Construct(util::HostArena& arena) const noexcept {
    auto* output=arena.Construct<Storage>(header); if(!output) return nullptr;
    output->model.node_count=mass.count;
    output->model.mass=arena.Construct<double>(mass); output->model.inertia=arena.Construct<double>(inertia);
    output->model.physical=arena.Construct<double>(physical); output->model.added=arena.Construct<double>(added);
    return output->model.mass&&output->model.inertia&&output->model.physical&&output->model.added?output:nullptr;
  }
  Storage Rebase(const Storage& host,void* device) const noexcept {
    Storage result=host;
    result.model.mass=util::ArenaPointer<double>(device,mass); result.model.inertia=util::ArenaPointer<double>(device,inertia);
    result.model.physical=util::ArenaPointer<double>(device,physical); result.model.added=util::ArenaPointer<double>(device,added);
    return result;
  }
};
void LaunchMeasure(Storage*,NodalPreparedView);
} // namespace tl::fea::shell_publication_detail
