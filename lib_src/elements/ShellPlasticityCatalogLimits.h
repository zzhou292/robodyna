// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>

namespace tl::fea {
// HOST catalog admission only. These do not enlarge resident shell/contact
// arenas, curve pools, integration histories, or the legacy binding API.
inline constexpr std::size_t MaxVehiclePlasticityCatalogParents=524288;
inline constexpr std::size_t MaxVehiclePlasticityCatalogNodes=524288;
inline constexpr std::size_t MaxPlasticityCatalogDefinitions=1024;
// Budget profile values, not hard byte ceilings; entity counts bound allocation.
inline constexpr std::size_t MaxVehiclePlasticityCatalogOwnedBytes=256ULL*1024*1024;
inline constexpr std::size_t MaxVehiclePlasticityCatalogScratchBytes=32ULL*1024*1024;
struct ShellPlasticityCatalogLimits {
  std::size_t max_parents=1024,max_nodes=2048,max_definitions=1024;
  std::size_t max_owned_bytes=4ULL*1024*1024;
  // Transient indexes/seen arrays and the inline staging Data object; retained
  // backing arrays are charged once to max_owned_bytes, not again here.
  std::size_t max_startup_scratch_bytes=MaxVehiclePlasticityCatalogScratchBytes;
  static constexpr ShellPlasticityCatalogLimits Vehicle() noexcept {
    return {MaxVehiclePlasticityCatalogParents,MaxVehiclePlasticityCatalogNodes,
      MaxPlasticityCatalogDefinitions,MaxVehiclePlasticityCatalogOwnedBytes,
      MaxVehiclePlasticityCatalogScratchBytes};
  }
};
} // namespace tl::fea
