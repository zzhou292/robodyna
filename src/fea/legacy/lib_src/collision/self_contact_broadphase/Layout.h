#pragma once
#include "../SelfContactBroadphase.h"
#include "../HydroelasticBroadphaseTypes.cuh"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::self_contact_broadphase {
struct Parent {
  std::uint32_t nodes[4]{};
  unsigned arity = 0;
  double half_thickness = 0;
};
struct ScratchRequirements {
  std::size_t sort = 0, scan = 0, pair_sort = 0;
};
struct Control {
  std::uint32_t invalid_parent = UINT32_MAX;
  std::uint64_t count = 0;
};
struct Layout {
  tl::util::ArenaRegion parents, boxes, sorted_boxes, keys, sorted_keys, indices, sorted_indices;
  tl::util::ArenaRegion counts, offsets, pair_keys, sorted_pair_keys, control, cub_temp;
  SelfContactBroadphaseForecast forecast;
};
// Fixed startup/query descriptor staging; parent upload is charged separately.
inline constexpr std::size_t QueryStagingBytes = sizeof(Layout) + sizeof(ScratchRequirements) +
    sizeof(SelfContactBroadphasePreflight) + sizeof(tl::util::HostArena) +
    2 * sizeof(SelfContactBroadphaseReport) + sizeof(SelfContactBroadphaseLimits) + 128;
SelfContactBroadphaseReport CheckSource(const SelfContactSurfaceBinding&,
    SelfContactBroadphaseLimits) noexcept;
SelfContactBroadphaseReport MakeLayout(const SelfContactSurfaceBinding&,
    SelfContactBroadphaseLimits, ScratchRequirements, std::size_t, Layout&) noexcept;
bool CopyParents(const SelfContactSurfaceBinding&, Parent*, std::size_t) noexcept;
} // namespace tlfea::contact::self_contact_broadphase
