// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellCollectionLimits.h"

namespace tl::fea {
inline constexpr std::size_t MaxShellResidentParents=1024;
inline constexpr std::size_t MaxShellResidentNodes=2048;
inline constexpr std::size_t MaxShellResidentDeviceBytes=8*1024*1024;
inline constexpr std::size_t MaxShellResidentHostBytes=32*1024*1024;
// Old callers retain 128-parent/node admission. A larger immutable collection
// requires explicit count limits and enough of the existing config device cap.
// Host cap includes retained scope plus a conservative startup staging peak;
// allocator bookkeeping and CUDA runtime/driver memory are outside this payload.
struct ShellResidentLimits {
  std::size_t max_parents=MaxShellCollectionParents,max_nodes=MaxShellCollectionNodes;
  std::size_t max_host_bytes=8*1024*1024;
};
inline bool ValidShellResidentLimits(const ShellResidentLimits& limits,std::size_t parents,
    std::size_t nodes,std::size_t device_cap) noexcept {
  return limits.max_parents&&limits.max_parents<=MaxShellResidentParents&&
    limits.max_nodes&&limits.max_nodes<=MaxShellResidentNodes&&
    limits.max_host_bytes&&limits.max_host_bytes<=MaxShellResidentHostBytes&&
    parents&&parents<=limits.max_parents&&nodes&&nodes<=limits.max_nodes&&
    device_cap&&device_cap<=MaxShellResidentDeviceBytes;
}
} // namespace tl::fea
