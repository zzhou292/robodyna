// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>

namespace tl::fea {
// Existing host catalog/default bounds; keep their fixed scratch independent.
inline constexpr std::size_t MaxShellHostParents=1024;
inline constexpr std::size_t MaxShellHostNodes=2048;
// Binding-only hard bounds. These confer no catalog or resident capacity.
inline constexpr std::size_t MaxVehicleShellBindingParents=524288;
inline constexpr std::size_t MaxVehicleShellBindingNodes=524288;
inline constexpr std::size_t MaxVehicleShellBindingOwnedBytes=1024u*1024u*1024u;
inline constexpr std::size_t MaxVehicleShellBindingScratchBytes=64u*1024u*1024u;
struct ShellHostBindingLimits {
  std::size_t max_parents=MaxShellHostParents;
  std::size_t max_nodes=MaxShellHostNodes;
  std::size_t max_owned_bytes=4*1024*1024;
  // Startup indexes/seen flags only, separate from immutable owned payload.
  // Includes their inline objects, dynamic arrays and reserved control bytes.
  std::size_t max_startup_scratch_bytes=MaxVehicleShellBindingScratchBytes;
  static constexpr ShellHostBindingLimits Vehicle() noexcept {
    return {MaxVehicleShellBindingParents,MaxVehicleShellBindingNodes,
            MaxVehicleShellBindingOwnedBytes,MaxVehicleShellBindingScratchBytes};
  }
};

} // namespace tl::fea
