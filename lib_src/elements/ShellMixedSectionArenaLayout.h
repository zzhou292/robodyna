#pragma once
#include "ShellBatchLayeredSection.h"
#include "ShellGlobalLaw1Profile.h"
#include "ShellPlasticArenaLayout.h"

namespace tl::fea::shell_batch_plasticity_detail {
// A separate opt-in layout keeps the legacy DeviceStorage ABI/allocation intact.
// Each array uses the original native family parent index. Inactive typed slots
// are private capacity, never history values exposed through public readback.
struct MixedDeviceStorage {
  DeviceStorage plastic;
  ShellSectionLaw* law=nullptr;
  ShellGlobalLaw1Profile* global_law1=nullptr; // Immutable; absent for all-legacy collections.
  material::ShellElasticLaw1PointParameters* elastic_parameters=nullptr;
  sections::ShellLayeredLaw1History* elastic_section[2]{nullptr,nullptr};
};
static_assert(std::is_trivially_copyable_v<MixedDeviceStorage>);
struct MixedLayout {
  util::ArenaRegion header,law,curve_x,curve_y,plastic_parameters,elastic_parameters,global_profiles;
  util::ArenaRegion plastic_section[2],elastic_section[2];
  std::size_t bytes=0;
  bool Initialize(std::size_t count,std::size_t points,std::size_t cap,bool global=false) noexcept {
    if(!count||count>MaxVehicleShellResidentParents||points==1||points>MaxShellPlasticityCurvePoints) return false;
    MixedLayout next;util::BoundedArenaLayout arena(cap);
    if(!arena.Append<MixedDeviceStorage>(1,next.header)||!arena.Append<ShellSectionLaw>(count,next.law)||
       !arena.Append<double>(points,next.curve_x)||!arena.Append<double>(points,next.curve_y)||
       !arena.Append<sections::PointParameters>(count,next.plastic_parameters)||
       !arena.Append<material::ShellElasticLaw1PointParameters>(count,next.elastic_parameters)) return false;
    if(global&&!arena.Append<ShellGlobalLaw1Profile>(count,next.global_profiles))return false;
    for(unsigned slab=0;slab<2;++slab)
      if(!arena.Append<ShellBatchSectionState>(count,next.plastic_section[slab])||
         !arena.Append<sections::ShellLayeredLaw1History>(count,next.elastic_section[slab])) return false;
    next.bytes=arena.bytes();*this=next;return true;
  }
  MixedDeviceStorage* Construct(util::HostArena& arena) const noexcept {
    auto* out=arena.Construct<MixedDeviceStorage>(header);if(!out)return nullptr;
    out->law=arena.Construct<ShellSectionLaw>(law);
    out->global_law1=global_profiles.count?arena.Construct<ShellGlobalLaw1Profile>(global_profiles):nullptr;
    out->plastic.curve_x=curve_x.count?arena.Construct<double>(curve_x):nullptr;
    out->plastic.curve_y=curve_y.count?arena.Construct<double>(curve_y):nullptr;
    out->plastic.parameters=arena.Construct<sections::PointParameters>(plastic_parameters);
    out->elastic_parameters=arena.Construct<material::ShellElasticLaw1PointParameters>(elastic_parameters);
    for(unsigned slab=0;slab<2;++slab) {
      out->plastic.section[slab]=arena.Construct<ShellBatchSectionState>(plastic_section[slab]);
      out->elastic_section[slab]=arena.Construct<sections::ShellLayeredLaw1History>(elastic_section[slab]);
      if(!out->plastic.section[slab]||!out->elastic_section[slab])return nullptr;
    }
    return out->law&&out->plastic.parameters&&out->elastic_parameters&&
      (!global_profiles.count||out->global_law1)&&
      (!curve_x.count||(out->plastic.curve_x&&out->plastic.curve_y))?out:nullptr;
  }
  MixedDeviceStorage Rebase(const MixedDeviceStorage& host,void* device) const noexcept {
    auto out=host;
    out.law=util::ArenaPointer<ShellSectionLaw>(device,law);
    out.global_law1=global_profiles.count?util::ArenaPointer<ShellGlobalLaw1Profile>(device,global_profiles):nullptr;
    out.plastic.curve_x=curve_x.count?util::ArenaPointer<double>(device,curve_x):nullptr;
    out.plastic.curve_y=curve_y.count?util::ArenaPointer<double>(device,curve_y):nullptr;
    out.plastic.parameters=util::ArenaPointer<sections::PointParameters>(device,plastic_parameters);
    out.elastic_parameters=util::ArenaPointer<material::ShellElasticLaw1PointParameters>(device,elastic_parameters);
    for(unsigned slab=0;slab<2;++slab) {
      out.plastic.section[slab]=util::ArenaPointer<ShellBatchSectionState>(device,plastic_section[slab]);
      out.elastic_section[slab]=util::ArenaPointer<sections::ShellLayeredLaw1History>(device,elastic_section[slab]);
    }
    return out;
  }
};
} // namespace tl::fea::shell_batch_plasticity_detail
