#pragma once
#include <cstddef>
#include <cstdint>

namespace tlfea::contact {
inline constexpr std::uint32_t MaxNodalWallParents=128,MaxNodalWallNodes=128;
inline constexpr std::uint32_t MaxNodalWallWeightParents=1024,MaxNodalWallWeightNodes=2048;
inline constexpr std::uint32_t MaxVehicleWallWeightParents=524288,MaxVehicleWallWeightNodes=524288;
inline constexpr std::size_t MaxVehicleWallWeightOwnedBytes=128*1024*1024;
inline constexpr std::size_t MaxVehicleWallWeightScratchBytes=32*1024*1024;
enum class NodalWallWeightProfile { Legacy,Vehicle };
struct NodalWallWeightLimits {
  std::uint32_t max_parents=MaxNodalWallWeightParents,max_nodes=MaxNodalWallWeightNodes;
  std::size_t max_owned_bytes=4*1024*1024;
  NodalWallWeightProfile profile=NodalWallWeightProfile::Legacy;
  std::size_t max_startup_bytes=MaxVehicleWallWeightScratchBytes;
  static constexpr NodalWallWeightLimits Vehicle() noexcept {
    return {MaxVehicleWallWeightParents,MaxVehicleWallWeightNodes,MaxVehicleWallWeightOwnedBytes,
      NodalWallWeightProfile::Vehicle,MaxVehicleWallWeightScratchBytes};
  }
};
} // namespace tlfea::contact
