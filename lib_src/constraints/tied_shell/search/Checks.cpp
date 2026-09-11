// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <climits>
#include <cmath>
#include <limits>

namespace tl::constraints::tied_shell::driver_detail {
namespace {
template<class T> bool Range(const T* pointer, std::size_t count) noexcept {
  const auto address = reinterpret_cast<std::uintptr_t>(pointer);
  return pointer && count <= SIZE_MAX / sizeof(T) && address % alignof(T) == 0 &&
      address <= UINTPTR_MAX - count * sizeof(T);
}
}
SearchDriverReport CheckCounts(const SearchDriverInput& in, const SearchDriverLimits& limits) noexcept {
  const SearchDriverLimits hard;
  const std::size_t actual[]{limits.max_nodes, limits.max_masters, limits.max_secondaries,
      limits.max_pairs, limits.max_host_bytes, limits.max_device_bytes};
  const std::size_t maximum[]{hard.max_nodes, hard.max_masters, hard.max_secondaries,
      hard.max_pairs, hard.max_host_bytes, hard.max_device_bytes};
  for (unsigned i = 0; i < 6; ++i) {
    if (!actual[i] || actual[i] > maximum[i])
      return Error(SearchDriverStatus::ResourceLimit, "Invalid tied search limits");
  }
  if (limits.axis > 2)
    return Error(SearchDriverStatus::InvalidInput, "Invalid tied search sweep axis");
  if (!in.node_count || !in.master_count || !in.secondary_count ||
      in.node_count > limits.max_nodes || in.master_count > limits.max_masters ||
      in.secondary_count > limits.max_secondaries)
    return Error(SearchDriverStatus::ResourceLimit, "Tied search count exceeds admission");
  const auto primitives = in.master_count + in.secondary_count;
  if (in.node_count > std::size_t(INT_MAX / 3) || primitives > std::size_t(INT_MAX / 4))
    return Error(SearchDriverStatus::ResourceLimit, "Tied search exceeds broadphase index domain");
  if (!Range(in.working_positions, in.node_count) || !Range(in.masters, in.master_count) ||
      !Range(in.secondary_nodes, in.secondary_count))
    return Error(SearchDriverStatus::InvalidInput, "Invalid tied search borrowed extent");
  if (!std::isfinite(in.working_length_to_m) || in.working_length_to_m <= 0 ||
      in.maximum_secondary_shell_thickness != 0)
    return Error(SearchDriverStatus::InvalidInput, "Tied search requires explicit units and zero secondary thickness");
  return {};
}
SearchDriverReport CheckValues(const SearchDriverInput& in) {
  namespace v = tl::math::fixed3;
  for (std::size_t n = 0; n < in.node_count; ++n) {
    if (!v::Finite(in.working_positions[n]))
      return Error(SearchDriverStatus::InvalidInput, "Nonfinite tied working position");
    if (!v::Finite(v::Scale(in.working_positions[n], in.working_length_to_m)))
      return Error(SearchDriverStatus::NumericalFailure, "Tied SI coordinate conversion overflows");
  }
  for (std::size_t m = 0; m < in.master_count; ++m) {
    const auto& row = in.masters[m];
    const bool triangle = row.topology == MasterTopology::TriangleRepeatedThird;
    if ((!triangle && row.topology != MasterTopology::Quad) ||
        !std::isfinite(row.bounds_thickness) || row.bounds_thickness <= 0 ||
        !std::isfinite(row.projection_thickness) || row.projection_thickness <= 0 ||
        (triangle && row.nodes[2] != row.nodes[3]))
      return Error(SearchDriverStatus::InvalidInput, "Invalid tied master topology or thickness", SIZE_MAX, m);
    for (unsigned j = 0; j < 4; ++j) {
      if (row.nodes[j] >= in.node_count)
        return Error(SearchDriverStatus::InvalidInput, "Tied master node is out of range", SIZE_MAX, m);
      if (triangle && j == 3) continue;
      for (unsigned k = 0; k < j; ++k) {
        if (row.nodes[k] == row.nodes[j])
          return Error(SearchDriverStatus::InvalidInput, "Repeated tied physical master node", SIZE_MAX, m);
      }
    }
  }
  std::vector<unsigned char> seen(in.node_count, 0);
  for (std::size_t s = 0; s < in.secondary_count; ++s) {
    const auto node = in.secondary_nodes[s];
    if (node >= in.node_count || seen[node])
      return Error(SearchDriverStatus::InvalidInput, "Invalid or repeated tied NSV node", s);
    seen[node] = 1;
  }
  return {};
}
} // namespace tl::constraints::tied_shell::driver_detail
