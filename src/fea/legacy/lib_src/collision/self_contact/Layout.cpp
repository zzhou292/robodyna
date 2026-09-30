// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "lib_utils/SourceIdentityIndex.h"
#include <algorithm>

namespace tlfea::contact::self_contact {
SelfContactSurfaceReport MakeLayout(const tl::fea::ShellPhysicalBinding& physical,
    const SelfContactSurfaceInput& input, SelfContactSurfaceLimits limits,
    std::size_t implementation_bytes, Layout& out) noexcept {
  using S = SelfContactSurfaceStatus;
  if (!physical.prepared() || !physical.catalog() || !physical.mapping() ||
      !physical.domain() || !input.parents || !input.parent_count ||
      input.profile != SelfContactSurfaceProfile::FrictionlessReferenceThicknessShellSubsetV1)
    return {S::InvalidInput, SIZE_MAX, "Prepared physical shell subset required"};
  const auto hard = SelfContactSurfaceLimits::Vehicle();
  if (!limits.max_parents || limits.max_parents > hard.max_parents ||
      !limits.max_nodes || limits.max_nodes > hard.max_nodes ||
      !limits.max_vertices || limits.max_vertices > hard.max_vertices ||
      !limits.max_edges || limits.max_edges > hard.max_edges ||
      !limits.max_host_bytes || limits.max_host_bytes > hard.max_host_bytes ||
      input.parent_count > limits.max_parents ||
      input.parent_count > physical.catalog()->parent_count() ||
      physical.domain()->node_count() > limits.max_nodes)
    return {S::ResourceLimit, SIZE_MAX, "Surface source counts or limits exceed scope"};
  const auto address = reinterpret_cast<std::uintptr_t>(input.parents);
  if (input.parent_count > (UINTPTR_MAX - address) / sizeof(SelfContactParentSelection))
    return {S::InvalidInput, SIZE_MAX, "Borrowed selection address range overflows"};
  // Bounds above prove all products and uint32 indices, before borrowed reads.
  Layout next;
  auto& f = next.forecast;
  f.parent_count = input.parent_count;
  f.corner_capacity = 4 * input.parent_count;
  f.vertex_capacity = std::min({f.corner_capacity, physical.domain()->node_count(), limits.max_vertices});
  f.edge_capacity = std::min(f.corner_capacity, limits.max_edges);
  tl::util::BoundedArenaLayout arena(limits.max_host_bytes);
  if (!arena.Append<SelfContactSurfaceParent>(f.parent_count, next.parents) ||
      !arena.Append<SelfContactVertex>(f.vertex_capacity, next.vertices) ||
      !arena.Append<SelfContactEdge>(f.edge_capacity, next.edges) ||
      !arena.Append<SelfContactVertexUse>(f.corner_capacity, next.vertex_uses) ||
      !arena.Append<SelfContactEdgeUse>(f.corner_capacity, next.edge_uses) ||
      !arena.Append<std::uint32_t>(f.parent_count, next.faces))
    return {S::ResourceLimit, SIZE_MAX, "Surface arena exceeds byte cap"};
  const auto retained = physical.owned_payload_bytes();
  if (retained < sizeof(physical)) return {S::ResourceLimit, SIZE_MAX, "Invalid physical payload report"};
  f.arena_bytes = arena.bytes();
  f.retained_source_bytes = retained - sizeof(physical);
  f.validation_scratch_bytes = tl::util::SourceIdentityIndex<0>::Bytes(f.parent_count);
  tl::util::BoundedArenaLayout budget(limits.max_host_bytes);
  tl::util::ArenaRegion ignored;
  if (!budget.Append<unsigned char>(sizeof(SelfContactSurfaceBinding) + implementation_bytes + 64, ignored) ||
      !budget.Append<unsigned char>(f.retained_source_bytes, ignored) ||
      !budget.Append<unsigned char>(f.arena_bytes, ignored))
    return {S::ResourceLimit, SIZE_MAX, "Surface and complete retained physical source exceed cap"};
  f.owned_payload_bytes = budget.bytes();
  if (!budget.Append<unsigned char>(f.validation_scratch_bytes, ignored))
    return {S::ResourceLimit, SIZE_MAX, "Surface startup index exceeds complete cap"};
  f.startup_payload_bytes = budget.bytes();
  out = next;
  return {};
}
} // namespace tlfea::contact::self_contact
