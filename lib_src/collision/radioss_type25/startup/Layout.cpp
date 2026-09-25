// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <climits>
namespace tlfea::contact::radioss_type25::startup::detail {
namespace {
bool Output(tl::util::BoundedArenaLayout& arena,std::size_t p,OutputLayout& out) noexcept {
  const auto g=2*p,refs=4*g;
  return arena.Append<Main>(g,out.mains) &&
      arena.Append<std::uint32_t>(g,out.expanded_to_primary) &&
      arena.Append<std::uint32_t>(p,out.primary_to_partner) &&
      arena.Append<StoredNormal>(4*g,out.normals) &&
      arena.Append<NormalReference>(refs,out.references) &&
      arena.Append<std::uint32_t>(refs+1,out.normal_offsets) &&
      arena.Append<std::uint32_t>(4*g,out.normal_mains);
}
}
Report MakeLayout(std::size_t nodes,std::size_t p,Limits limits,Layout& output) noexcept {
  const Limits hard;
  if (!nodes || !p) return {Status::InvalidInput};
  if (!limits.max_nodes || limits.max_nodes>hard.max_nodes || !limits.max_primary_faces ||
      limits.max_primary_faces>hard.max_primary_faces || !limits.max_output_bytes ||
      !limits.max_scratch_bytes || nodes>limits.max_nodes || p>limits.max_primary_faces ||
      p>std::size_t(INT_MAX)/8 || nodes>std::size_t(INT_MAX)) return {Status::ResourceLimit};
  Layout next; const auto g=2*p;
  tl::util::BoundedArenaLayout persistent(SIZE_MAX),scratch(SIZE_MAX),ready(SIZE_MAX);
  OutputLayout staging;
  if (!Output(persistent,p,next.output) || !Output(scratch,p,staging) ||
      !scratch.Append<Vector>(nodes,next.points) || !scratch.Append<Edge>(4*g,next.edges) ||
      !scratch.Append<FaceKey>(p,next.face_keys) ||
      !scratch.Append<Identity>(nodes>p?nodes:p,next.identities) ||
      !scratch.Append<int>(4*g,next.parents) || !scratch.Append<int>(4*g,next.tags) ||
      !scratch.Append<std::uint32_t>(nodes,next.node_references)) return {Status::ResourceLimit};
  tl::util::ArenaRegion normal_region,reference_region;
  if (!ready.Append<StoredNormal>(4*g,normal_region) ||
      !ready.Append<NormalReference>(4*g,reference_region)) return {Status::ResourceLimit};
  next.forecast={Status::Ok,persistent.bytes(),scratch.bytes(),ready.bytes(),scratch.bytes(),g,4*g,4*g};
  // Output's exact same prefix layout is used for private staging. The dead
  // sorted-edge region is later placement-constructed as neighbor-normal scratch;
  // no edge pointer survives that phase transition and no extra allocation exists.
  static_assert(sizeof(Edge)>=sizeof(StoredNormal));
  if (persistent.bytes()>limits.max_output_bytes || ready.bytes()>limits.max_output_bytes ||
      scratch.bytes()>limits.max_scratch_bytes) next.forecast.status=Status::ResourceLimit;
  output=next; return {next.forecast.status};
}
Data Construct(tl::util::HostArena& arena,const OutputLayout& p) noexcept {
  return {arena.Construct<Main>(p.mains),arena.Construct<std::uint32_t>(p.expanded_to_primary),
      arena.Construct<std::uint32_t>(p.primary_to_partner),arena.Construct<StoredNormal>(p.normals),
      arena.Construct<NormalReference>(p.references),arena.Construct<std::uint32_t>(p.normal_offsets),
      arena.Construct<std::uint32_t>(p.normal_mains)};
}
} // namespace tlfea::contact::radioss_type25::startup::detail
namespace tlfea::contact::radioss_type25::startup {
Forecast Preflight(std::size_t nodes,std::size_t primary,Limits limits) noexcept {
  detail::Layout layout; const auto report=detail::MakeLayout(nodes,primary,limits,layout);
  if (layout.forecast.output_bytes) return layout.forecast;
  Forecast result; result.status=report.status; return result;
}
} // namespace tlfea::contact::radioss_type25::startup
