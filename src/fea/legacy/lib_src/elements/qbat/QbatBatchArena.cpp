// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatBatchArena.h"
#include "../ShellBatchPlasticityBinding.h"

namespace tl::fea::qbat::batch_detail {
bool Layout::Initialize(std::size_t parents,std::size_t nodes,std::size_t points,
                        std::size_t cap) noexcept {
  if(!parents||parents>MaxVehicleShellResidentParents||!nodes||nodes>MaxVehicleShellResidentNodes||
      points==1||points>MaxShellPlasticityCurvePoints||!cap||cap>MaxVehicleShellResidentDeviceBytes) {
    return false;
  }
  Layout next;
  if(!next.common.Initialize(parents,nodes,cap)) return false;
  util::BoundedArenaLayout layout(cap);
  util::ArenaRegion prefix;
  if(!layout.Append<unsigned char>(next.common.bytes,prefix)||
      !layout.Append<double>(points,next.curve_x)||!layout.Append<double>(points,next.curve_y)) {
    return false;
  }
  if(!next.activity.Initialize(layout.bytes(),parents,cap)) return false;
  next.bytes=next.activity.bytes;
  *this=next;
  return true;
}

bool Layout::InitializeMapped(std::size_t parents,std::size_t nodes,std::size_t points,
    std::size_t cap) noexcept {
  Layout next;
  if(!next.Initialize(parents,nodes,points,cap) ||
      !next.assembly.Initialize(next.bytes,parents,nodes,cap)) return false;
  next.bytes=next.assembly.bytes;
  *this=next;
  return true;
}

Storage* Layout::Construct(util::HostArena& arena) const noexcept {
  auto* storage=common.Construct(arena);
  if(!storage) return nullptr;
  storage->model.curve_points=curve_x.count;
  if(curve_x.count) {
    storage->model.curve_x=arena.Construct<double>(curve_x);
    storage->model.curve_y=arena.Construct<double>(curve_y);
    if(!storage->model.curve_x||!storage->model.curve_y) return nullptr;
  }
  if(!assembly.Construct(arena,storage->assembly) ||
      !activity.Construct(arena,storage->activity)) return nullptr;
  return storage;
}

Storage Layout::Rebase(const Storage& host,void* device) const noexcept {
  auto output=common.Rebase(host,device);
  output.model.curve_x=curve_x.count?util::ArenaPointer<double>(device,curve_x):nullptr;
  output.model.curve_y=curve_y.count?util::ArenaPointer<double>(device,curve_y):nullptr;
  output.assembly=assembly.Rebase(device);
  output.activity=activity.Rebase(device);
  return output;
}
} // namespace tl::fea::qbat::batch_detail
