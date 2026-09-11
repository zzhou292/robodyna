#pragma once
#include "NodalStateLimits.h"
#include "lib_utils/BoundedArena.h"
#include <cstdint>
#include <limits>

namespace tl::fea::nodal_detail {
// Forecast the EXISTING separate allocations; offsets describe a byte budget,
// not a packed allocation ABI. Control/masks/rigid metadata need no inter-buffer
// alignment padding because cudaMalloc gives each its own aligned allocation.
struct StateLayout {
  util::ArenaRegion accepted,trial,scratch,inverse,fixed,control,rigid,cin;
  std::size_t bytes=0;
  bool Initialize(std::size_t nodes,bool rotations,std::size_t group_values,
      std::size_t immutable_rigid_bytes,std::size_t capture_values,
      std::size_t control_bytes,std::size_t cap,
      std::size_t cin_state_values=0,std::size_t cin_device_bytes=0) noexcept {
    if(!nodes||nodes>MaxActiveNodalStateNodes||!cap||cap>MaxActiveNodalStateDeviceBytes||
       !control_bytes||(!rotations&&(group_values||immutable_rigid_bytes||capture_values))) return false;
    const std::size_t nodal_values=(rotations?19:6)*nodes;
    if(group_values>std::numeric_limits<std::size_t>::max()-nodal_values) return false;
    if(cin_state_values>std::numeric_limits<std::size_t>::max()-nodal_values-group_values) return false;
    if((cin_state_values||cin_device_bytes)&&(!rotations||!cin_state_values||!cin_device_bytes)) return false;
    const auto state_values=nodal_values+group_values+cin_state_values;
    if(capture_values>state_values||capture_values>std::numeric_limits<std::size_t>::max()-11*nodes) return false;
    StateLayout next; util::BoundedArenaLayout budget(cap);
    if(!budget.Append<double>(state_values,next.accepted)||!budget.Append<double>(state_values,next.trial)||
       !budget.Append<double>(11*nodes+capture_values,next.scratch)||
       !budget.Append<double>((rotations?2:1)*nodes,next.inverse)||
       !budget.Append<std::uint8_t>((rotations?3:1)*nodes,next.fixed)||
       !budget.Append<std::byte>(control_bytes,next.control)||
       !budget.Append<std::byte>(immutable_rigid_bytes,next.rigid)||
       !budget.Append<std::byte>(cin_device_bytes,next.cin)) return false;
    next.bytes=budget.bytes(); *this=next; return true;
  }
};
} // namespace tl::fea::nodal_detail
