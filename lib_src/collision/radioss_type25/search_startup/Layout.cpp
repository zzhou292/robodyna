// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25SearchStartup.h"
#include "Internal.h"
#include <climits>
namespace tlfea::contact::radioss_type25::search_startup::detail {
namespace {
bool Output(tl::util::BoundedArenaLayout& a, std::size_t p, std::size_t g,
    std::size_t s, std::size_t capacity, OutputLayout& out) noexcept {
  return a.Append<double>(p,out.extent) && a.Append<std::uint32_t>(g+1,out.main_offsets) &&
      a.Append<std::uint32_t>(s+1,out.secondary_offsets) &&
      a.Append<std::uint32_t>(capacity,out.removed_nodes) &&
      a.Append<std::uint32_t>(capacity,out.removed_mains) && a.Append<int>(s,out.contact);
}
}
Report MakeLayout(std::size_t nodes, std::size_t p, std::size_t s, Limits limits,
    Layout& out) noexcept {
  if (!nodes || !p || !s) return {Status::InvalidInput};
  const Limits hard;
  if (nodes>limits.max_nodes || nodes>hard.max_nodes || p>hard.max_mains/2 ||
      2*p>limits.max_mains || s>limits.max_secondaries || s>hard.max_secondaries ||
      !limits.max_neighbor_visits || limits.max_neighbor_visits>hard.max_neighbor_visits ||
      !limits.max_margin_iterations || limits.max_margin_iterations>hard.max_margin_iterations ||
      limits.max_removals>hard.max_removals || !limits.max_output_bytes || !limits.max_scratch_bytes)
    return {Status::ResourceLimit};
  const auto g=2*p;
  if (g>SIZE_MAX/s || 4*g>std::size_t(INT_MAX)) return {Status::ResourceLimit};
  const auto capacity=std::min(limits.max_removals,g*s);
  Layout next;
  tl::util::BoundedArenaLayout persistent(SIZE_MAX),scratch(SIZE_MAX);
  OutputLayout staged;
  if (!Output(persistent,p,g,s,capacity,next.output) || !Output(scratch,p,g,s,capacity,staged) ||
      !scratch.Append<Vector>(nodes,next.points) ||
      !scratch.Append<std::uint32_t>(nodes,next.secondary_index) ||
      !scratch.Append<double>(nodes,next.secondary_gap) ||
      !scratch.Append<std::uint32_t>(nodes+1,next.node_offsets) ||
      !scratch.Append<std::uint32_t>(4*g,next.node_mains) ||
      !scratch.Append<std::uint32_t>(std::max(nodes,s),next.cursors) ||
      !scratch.Append<int>(nodes,next.tag) || !scratch.Append<int>(nodes,next.expanded) ||
      !scratch.Append<double>(nodes,next.distance) || !scratch.Append<double>(nodes,next.gap) ||
      !scratch.Append<int>(g,next.segment_tag) ||
      !scratch.Append<std::uint32_t>(g,next.current) || !scratch.Append<std::uint32_t>(g,next.next) ||
      !scratch.Append<std::uint32_t>(g,next.visited) ||
      !scratch.Append<std::uint32_t>(nodes,next.discovered)) return {Status::ResourceLimit};
  next.forecast={Status::Ok,persistent.bytes(),scratch.bytes(),capacity};
  if (persistent.bytes()>limits.max_output_bytes || scratch.bytes()>limits.max_scratch_bytes)
    next.forecast.status=Status::ResourceLimit;
  out=next;return {next.forecast.status};
}
Data Construct(tl::util::HostArena& a, const OutputLayout& l) noexcept {
  return {a.Construct<double>(l.extent),a.Construct<std::uint32_t>(l.main_offsets),
      a.Construct<std::uint32_t>(l.secondary_offsets),a.Construct<std::uint32_t>(l.removed_nodes),
      a.Construct<std::uint32_t>(l.removed_mains),a.Construct<int>(l.contact)};
}
Work ConstructWork(tl::util::HostArena& a, const Layout& l) noexcept {
  return {a.Construct<Vector>(l.points),a.Construct<std::uint32_t>(l.secondary_index),
      a.Construct<double>(l.secondary_gap),a.Construct<std::uint32_t>(l.node_offsets),
      a.Construct<std::uint32_t>(l.node_mains),a.Construct<std::uint32_t>(l.cursors),
      a.Construct<int>(l.tag),a.Construct<int>(l.expanded),a.Construct<double>(l.distance),
      a.Construct<double>(l.gap),a.Construct<int>(l.segment_tag),
      a.Construct<std::uint32_t>(l.current),a.Construct<std::uint32_t>(l.next),
      a.Construct<std::uint32_t>(l.visited),a.Construct<std::uint32_t>(l.discovered)};
}
} // namespace tlfea::contact::radioss_type25::search_startup::detail
namespace tlfea::contact::radioss_type25::search_startup {
Forecast Preflight(std::size_t nodes, std::size_t primaries, std::size_t secondaries,
    Limits limits) noexcept {
  detail::Layout layout;const auto report=detail::MakeLayout(nodes,primaries,secondaries,limits,layout);
  if (layout.forecast.output_bytes) return layout.forecast;
  Forecast result;result.status=report.status;return result;
}
} // namespace tlfea::contact::radioss_type25::search_startup
