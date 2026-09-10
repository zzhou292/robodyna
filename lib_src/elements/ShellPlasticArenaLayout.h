// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchPlasticityBinding.h"
#include "ShellResidentLimits.h"
#include "lib_utils/BoundedArena.h"

namespace tl::fea::shell_batch_plasticity_detail {
struct DeviceStorage {
  double *curve_x=nullptr,*curve_y=nullptr;
  sections::PointParameters* parameters=nullptr;
  ShellBatchSectionState* section[2]{nullptr,nullptr};
};
static_assert(std::is_trivially_copyable_v<DeviceStorage>);
static_assert(sizeof(DeviceStorage)<128,"Optional plasticity has a small pointer-only header");
struct Layout {
  util::ArenaRegion header,curve_x,curve_y,parameters,section[2];
  std::size_t bytes=0;
  bool Initialize(std::size_t count,std::size_t points,std::size_t cap) noexcept {
    if(!count||count>MaxShellResidentParents||points<2||points>MaxShellPlasticityCurvePoints) return false;
    Layout next; util::BoundedArenaLayout layout(cap);
    if(!layout.Append<DeviceStorage>(1,next.header)||!layout.Append<double>(points,next.curve_x)||
       !layout.Append<double>(points,next.curve_y)||!layout.Append<sections::PointParameters>(count,next.parameters)||
       !layout.Append<ShellBatchSectionState>(count,next.section[0])||!layout.Append<ShellBatchSectionState>(count,next.section[1])) return false;
    next.bytes=layout.bytes(); *this=next; return true;
  }
  DeviceStorage* Construct(util::HostArena& arena) const noexcept {
    auto* output=arena.Construct<DeviceStorage>(header); if(!output) return nullptr;
    output->curve_x=arena.Construct<double>(curve_x); output->curve_y=arena.Construct<double>(curve_y);
    output->parameters=arena.Construct<sections::PointParameters>(parameters);
    for(unsigned i=0;i<2;++i) output->section[i]=arena.Construct<ShellBatchSectionState>(section[i]);
    return output->curve_x&&output->curve_y&&output->parameters&&output->section[0]&&output->section[1]?output:nullptr;
  }
  DeviceStorage Rebase(const DeviceStorage& host,void* device) const noexcept {
    DeviceStorage result=host;
    result.curve_x=util::ArenaPointer<double>(device,curve_x); result.curve_y=util::ArenaPointer<double>(device,curve_y);
    result.parameters=util::ArenaPointer<sections::PointParameters>(device,parameters);
    for(unsigned i=0;i<2;++i) result.section[i]=util::ArenaPointer<ShellBatchSectionState>(device,section[i]);
    return result;
  }
};
} // namespace tl::fea::shell_batch_plasticity_detail
