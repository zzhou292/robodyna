// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Values.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::search_startup::detail {
struct OutputLayout {
  tl::util::ArenaRegion extent, main_offsets, secondary_offsets;
  tl::util::ArenaRegion removed_nodes, removed_mains, contact;
};
struct Layout {
  Forecast forecast;
  OutputLayout output;
  tl::util::ArenaRegion points, secondary_index, secondary_gap;
  tl::util::ArenaRegion node_offsets, node_mains, cursors;
  tl::util::ArenaRegion tag, expanded, distance, gap;
  tl::util::ArenaRegion segment_tag, current, next, visited, discovered;
};
struct Data {
  double* extent;
  std::uint32_t* main_offsets;
  std::uint32_t* secondary_offsets;
  std::uint32_t* removed_nodes;
  std::uint32_t* removed_mains;
  int* contact;
};
struct Work {
  Vector* points;
  std::uint32_t* secondary_index; double* secondary_gap;
  std::uint32_t* node_offsets; std::uint32_t* node_mains; std::uint32_t* cursors;
  int* tag; int* expanded; double* distance; double* gap;
  int* segment_tag;
  std::uint32_t* current; std::uint32_t* next; std::uint32_t* visited; std::uint32_t* discovered;
};
Report MakeLayout(std::size_t nodes, std::size_t primaries, std::size_t secondary,
    Limits, Layout&) noexcept;
Data Construct(tl::util::HostArena&, const OutputLayout&) noexcept;
Work ConstructWork(tl::util::HostArena&, const Layout&) noexcept;
Report Admit(const Input&, const Layout&, Limits, const tl::util::HostArena&,
    const tl::util::HostArena&, const Snapshot*) noexcept;
Report Prepare(const Input&, Work, double& maximum_secondary_gap, double& total_gap) noexcept;
Report Removals(const Input&, Limits, Work, Data, std::size_t capacity,
    double maximum_secondary_gap) noexcept;
} // namespace tlfea::contact::radioss_type25::search_startup::detail
