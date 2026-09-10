#pragma once
#include "NodalWallContactStorage.h"
#include "lib_utils/BoundedStartupArray.h"

namespace tlfea::contact::nodal_wall_device_detail {
static_assert(MaxVehicleNodalWallDeviceParents<=UINT32_MAX/4,"Native incidence slots fit uint32_t");
// The caller has admitted global counts and a fresh private model. Global node
// indices are already a dense bounded identity, so no source-ID sort is needed.
inline constexpr std::size_t IncidenceStartupBytes(std::size_t global_nodes) noexcept {
  using Index=tl::util::BoundedStartupArray<std::uint32_t,0>;
  return sizeof(Index)+Index::ExtraBytes(global_nodes);
}
NodalWallDeviceReport BuildVehicleIncidence(Model&);
} // namespace tlfea::contact::nodal_wall_device_detail
