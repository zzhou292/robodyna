#pragma once
#include "NodalForceAssembly.h"

#if defined(__CUDACC__)
#define TL_REPEATED_ASSEMBLY_HD __host__ __device__
#else
#define TL_REPEATED_ASSEMBLY_HD
#endif

namespace tl::fea {
// Stage an ordered contribution whose slots may repeat a physical node, as in
// a native tied triangle. The topology/owner must separately admit those slots.
// Reuse the existing single-node scatter on private scratch, retaining native
// ((old + slot3) + slot4) arithmetic instead of old + (slot3 + slot4).
// As with AccumulateNodalForces, destination ranges are disjoint and must not
// overlap inputs. Every rejection leaves all original destination values intact.
namespace nodal_force_detail {
template<unsigned Count, bool WithRotation>
TL_REPEATED_ASSEMBLY_HD inline NodalForceAssemblyStatus AccumulateRepeated(
    const std::size_t* nodes, const tl::math::Vec3* force, const tl::math::Vec3* couple,
    DeviceNodalForceView view, int sign = 1) {
  static_assert(Count > 0 && Count <= 8, "Bounded native slot scatter");
  constexpr unsigned Channels = WithRotation ? 6 : 3;
  if (!nodes || !force || !view.node_count || !view.force_x || !view.force_y ||
      !view.force_z || (WithRotation && (!couple || !view.couple_x || !view.couple_y || !view.couple_z)) ||
      (sign != 1 && sign != -1)) return NodalForceAssemblyStatus::InvalidView;
  double* arrays[] = {view.force_x,view.force_y,view.force_z,
                      view.couple_x,view.couple_y,view.couple_z};
  for (unsigned c = 0; c < Channels; ++c) {
    for (unsigned other = 0; other < c; ++other) {
      if (arrays[c] == arrays[other]) return NodalForceAssemblyStatus::InvalidView;
    }
  }
  double staged[Channels][Count]{};
  std::size_t unique_nodes[Count]{}, local_nodes[Count]{};
  unsigned unique_count = 0;
  for (unsigned n = 0; n < Count; ++n) {
    if (nodes[n] >= view.node_count) return NodalForceAssemblyStatus::InvalidConnectivity;
    unsigned local = 0;
    while (local < unique_count && unique_nodes[local] != nodes[n]) ++local;
    if (local == unique_count) {
      unique_nodes[unique_count++] = nodes[n];
      for (unsigned c = 0; c < Channels; ++c) staged[c][local] = arrays[c][nodes[n]];
    }
    local_nodes[n] = local;
  }
  DeviceNodalForceView scratch{staged[0],staged[1],staged[2],nullptr,nullptr,nullptr,
                              unique_count,view.base_epoch};
  if constexpr (WithRotation) {
    scratch.couple_x = staged[3]; scratch.couple_y = staged[4]; scratch.couple_z = staged[5];
  }
  for (unsigned n = 0; n < Count; ++n) {
    const auto status = Accumulate<1,WithRotation>(local_nodes+n,force+n,
        WithRotation ? couple+n : nullptr,scratch,sign);
    if (status != NodalForceAssemblyStatus::Success) return status;
  }
  for (unsigned n = 0; n < unique_count; ++n) {
    for (unsigned c = 0; c < Channels; ++c) arrays[c][unique_nodes[n]] = staged[c][n];
  }
  return NodalForceAssemblyStatus::Success;
}
} // namespace nodal_force_detail
template<unsigned Count>
TL_REPEATED_ASSEMBLY_HD inline NodalForceAssemblyStatus AccumulateRepeatedNodalForces(
    const std::size_t* nodes, const tl::math::Vec3* force, const tl::math::Vec3* couple,
    DeviceNodalForceView view, int sign = 1) {
  static_assert(Count > 0 && Count <= 4, "Bounded tied triangle force/couple slots");
  return nodal_force_detail::AccumulateRepeated<Count,true>(nodes,force,couple,view,sign);
}
// Source-slot topology is admitted by the typed element. This value operation
// preserves all eight ordered terms, including repeated H8 connectivity, and
// never reads or writes rotational channels. The strict distinct-node API stays
// unchanged. A failed element leaves every original force destination intact.
template<unsigned Count>
TL_REPEATED_ASSEMBLY_HD inline NodalForceAssemblyStatus AccumulateRepeatedNodalTranslationalForces(
    const std::size_t* nodes, const tl::math::Vec3* force,
    DeviceNodalForceView view, int sign = 1) {
  return nodal_force_detail::AccumulateRepeated<Count,false>(nodes,force,nullptr,view,sign);
}
} // namespace tl::fea
#undef TL_REPEATED_ASSEMBLY_HD
