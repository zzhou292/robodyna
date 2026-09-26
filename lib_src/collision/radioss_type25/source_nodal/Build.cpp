// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25NodalSeed.h"
#include "../coefficients/Common.h"
#include "../search/Ranges.h"
#include "HostRanges.h"
#include "lib_utils/BoundedArena.h"
#include <cstring>
#include <new>

namespace tlfea::contact::radioss_type25::source_nodal {
namespace {
struct Layout {
  tl::util::ArenaRegion nodes;
  Forecast forecast;
};

Report Prepare(const Input& input, Limits limits, Layout& output) noexcept {
  const Limits maximum;
  if (!limits.nodes || limits.nodes > maximum.nodes ||
      limits.volume_occurrences > maximum.volume_occurrences ||
      limits.stiffness_occurrences > maximum.stiffness_occurrences ||
      !limits.scratch_bytes || limits.scratch_bytes > maximum.scratch_bytes ||
      input.node_count > limits.nodes || input.volume_count > limits.volume_occurrences ||
      input.stiffness_count > limits.stiffness_occurrences) return {Status::ResourceLimit};
  if (!input.node_count || !search::detail::Span(input.volumes, input.volume_count) ||
      !search::detail::Span(input.stiffness, input.stiffness_count)) return {Status::InvalidInput};

  Layout next;
  tl::util::BoundedArenaLayout arena(limits.scratch_bytes);
  if (!arena.Append<NativeNodalSeed>(input.node_count, next.nodes)) return {Status::ResourceLimit};
  next.forecast.scratch_bytes = arena.bytes();
  next.forecast.output_bytes = input.node_count * sizeof(NativeNodalSeed);
  for (std::size_t i = 0; i < input.volume_count; ++i) {
    const auto& row = input.volumes[i];
    if (row.node >= input.node_count || !coefficient_detail::Finite(row.volume) ||
        !coefficient_detail::Finite(row.bulk_volume)) {
      return {Status::InvalidInput, Channel::Volume, i, row.node};
    }
  }
  for (std::size_t i = 0; i < input.stiffness_count; ++i) {
    const auto& row = input.stiffness[i];
    if (row.node >= input.node_count || !coefficient_detail::Finite(row.stiffness)) {
      return {Status::InvalidInput, Channel::Stiffness, i, row.node};
    }
  }
  output = next;
  return {Status::Ok};
}

using detail::Range;
using detail::Disjoint;
bool Separate(const Input& input, const Layout& layout, void* scratch, std::size_t bytes, Output out) noexcept {
  if (bytes < layout.forecast.scratch_bytes || !scratch ||
      reinterpret_cast<std::uintptr_t>(scratch) % alignof(std::max_align_t) ||
      bytes > UINTPTR_MAX - reinterpret_cast<std::uintptr_t>(scratch) ||
      out.node_count != input.node_count || !search::detail::Span(out.nodes, out.node_count)) return false;
  const Range reads[]{
      {&input, sizeof(input)},
      {input.volumes, input.volume_count * sizeof(NativeVolumeOccurrence)},
      {input.stiffness, input.stiffness_count * sizeof(NativeStiffnessOccurrence)}};
  const Range writes[]{{scratch, bytes}, {out.nodes, layout.forecast.output_bytes}};
  for (const auto write : writes) {
    for (const auto read : reads) if (!Disjoint(write, read)) return false;
  }
  return Disjoint(writes[0], writes[1]);
}
} // namespace

Report Preflight(const Input& input, Limits limits, Forecast& output) noexcept {
  Layout layout;
  const auto report = Prepare(input, limits, layout);
  if (report.status == Status::Ok) output = layout.forecast;
  return report;
}

Report Accumulate(const Input& input, Limits limits, void* scratch, std::size_t bytes, Output output) noexcept {
  Layout layout;
  const auto report = Prepare(input, limits, layout);
  if (report.status != Status::Ok) return report;
  if (!Separate(input, layout, scratch, bytes, output)) return {Status::InvalidInput};
  auto* staged = ::new (static_cast<void*>(tl::util::ArenaPointer<NativeNodalSeed>(scratch, layout.nodes)))
      NativeNodalSeed[input.node_count]{};
  for (std::size_t i = 0; i < input.volume_count; ++i) {
    const auto& row = input.volumes[i];
    auto& node = staged[row.node];
    node.volume = node.volume + row.volume;
    node.bulk_volume = node.bulk_volume + row.bulk_volume;
    if (!coefficient_detail::Finite(node.volume) || !coefficient_detail::Finite(node.bulk_volume)) {
      return {Status::NonfiniteResult, Channel::Volume, i, row.node};
    }
  }
  for (std::size_t i = 0; i < input.stiffness_count; ++i) {
    const auto& row = input.stiffness[i];
    auto& node = staged[row.node];
    node.existing_stiffness = node.existing_stiffness + row.stiffness;
    if (!coefficient_detail::Finite(node.existing_stiffness)) {
      return {Status::NonfiniteResult, Channel::Stiffness, i, row.node};
    }
  }
  std::memcpy(output.nodes, staged, layout.forecast.output_bytes);
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::source_nodal
