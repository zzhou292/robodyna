// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceGatherTypes.h"
#include "lib_utils/OrderedNodeIncidence.h"

namespace tl::fea::cin_advance::force_gather {
// Source rows have passed immutable attachment and owner-role admission.
// Dense offsets are temporary startup scratch. Compaction changes only the
// node directory, never the encoded source-row/slot sequence for a node.
inline bool BuildIncidence(cin::StageView source, std::uint32_t* dense_offsets,
    std::uint32_t* nodes, std::uint32_t* offsets, std::uint32_t* incidence,
    std::size_t capacity, std::uint32_t& master_count) noexcept {
  const auto maximum = source.node_count < 4ull * source.row_count
      ? source.node_count : 4ull * source.row_count;
  if (!source.rows || !source.dependent_nodes || !nodes || !offsets ||
      !source.row_count || source.row_count > UINT32_MAX / 4 || capacity != maximum)
    return false;
  // Preserve the source's two explicit topologies. No general collapsed quad
  // or dependent/master hierarchy is admitted by this construction helper.
  for (std::uint32_t row = 0; row < source.row_count; ++row) {
    const auto& value = source.rows[row];
    if (value.secondary >= source.node_count || !source.dependent_nodes[value.secondary]) return false;
    for (unsigned slot = 0; slot < 4; ++slot) {
      const auto node = value.masters[slot];
      if (node >= source.node_count || source.dependent_nodes[node]) return false;
      for (unsigned prior = 0; prior < slot; ++prior)
        if (node == value.masters[prior] && !(slot == 3 && prior == 2)) return false;
    }
  }
  if (!util::BuildOrderedNodeIncidence<4>(source.row_count, source.node_count,
      [&](std::size_t row, unsigned slot) { return source.rows[row].masters[slot]; },
      dense_offsets, std::size_t(source.node_count) + 1, incidence, 4ull * source.row_count)) return false;
  std::uint32_t count = 0;
  for (std::uint32_t node = 0; node < source.node_count; ++node) {
    if (dense_offsets[node] == dense_offsets[node + 1]) continue;
    nodes[count] = node;
    offsets[count++] = dense_offsets[node];
  }
  offsets[count] = 4 * source.row_count;
  master_count = count;
  return true;
}
} // namespace tl::fea::cin_advance::force_gather
