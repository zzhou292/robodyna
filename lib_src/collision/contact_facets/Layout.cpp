// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tlfea::contact::contact_facets {
FixedContactFacetReport MakeLayout(const SelfContactSurfaceBinding& surface, FixedContactFacetConfig config,
    FixedContactFacetLimits limits, std::size_t implementation_bytes, Layout& output) noexcept {
  using S = FixedContactFacetStatus;
  if (!surface.prepared() || config.profile != FixedContactFacetProfile::WeightedReferenceSurfaceV1 || config.level > 2)
    return {S::InvalidInput, "Prepared native source and fixed level 0/1/2 required"};
  const auto hard = FixedContactFacetLimits::Vehicle();
  if (!limits.max_parents || limits.max_parents > hard.max_parents ||
      !limits.max_facets || limits.max_facets > hard.max_facets ||
      !limits.max_host_bytes || limits.max_host_bytes > hard.max_host_bytes ||
      surface.parents().size() > limits.max_parents)
    return {S::ResourceLimit, "Fixed facet counts or caps exceed scope"};
  const unsigned n = 1u << config.level;
  Layout next;
  auto& forecast = next.forecast;
  forecast.parents = surface.parents().size();
  forecast.q4_template_vertices = (n + 1) * (n + 1);
  forecast.t3_template_vertices = (n + 1) * (n + 2) / 2;
  forecast.q4_template_facets = 2 * n * n;
  forecast.t3_template_facets = n * n;
  for (const auto& parent : surface.parents()) {
    if (parent.arity != 3 && parent.arity != 4)
      return {S::InvalidInput, "Native source parent arity differs"};
    const auto count = parent.arity == 4 ? forecast.q4_template_facets : forecast.t3_template_facets;
    if (count > limits.max_facets - forecast.facets)
      return {S::ResourceLimit, "Complete physical facet count exceeds cap"};
    forecast.facets += count;
  }
  tl::util::BoundedArenaLayout arena(limits.max_host_bytes);
  if (!arena.Append<Vertex>(forecast.q4_template_vertices, next.q4_vertices) ||
      !arena.Append<Vertex>(forecast.t3_template_vertices, next.t3_vertices) ||
      !arena.Append<Triangle>(forecast.q4_template_facets, next.q4_triangles) ||
      !arena.Append<Triangle>(forecast.t3_template_facets, next.t3_triangles))
    return {S::ResourceLimit, "Fixed template arena exceeds cap"};
  forecast.template_arena_bytes = arena.bytes();
  const auto retained = surface.forecast().owned_payload_bytes;
  if (retained < sizeof(surface)) return {S::ResourceLimit, "Invalid retained source forecast"};
  forecast.retained_source_bytes = retained - sizeof(surface);
  tl::util::BoundedArenaLayout budget(limits.max_host_bytes);
  tl::util::ArenaRegion ignored;
  if (!budget.Append<std::byte>(sizeof(FixedContactFacetBinding) + implementation_bytes + 64, ignored) ||
      !budget.Append<std::byte>(forecast.retained_source_bytes, ignored) ||
      !budget.Append<std::byte>(forecast.template_arena_bytes, ignored))
    return {S::ResourceLimit, "Complete retained source and fixed template exceed cap"};
  forecast.owned_payload_bytes = budget.bytes();
  // Includes the typed layout and success-only descriptor copy, plus a bounded
  // allowance for scalar/interval query temporaries. Caller output storage is
  // external; no per-parent output array is allocated by this binding.
  forecast.scratch_payload_bytes = sizeof(Layout) + sizeof(FixedContactFacet) + 512;
  if (!budget.Append<std::byte>(forecast.scratch_payload_bytes, ignored))
    return {S::ResourceLimit, "Fixed startup/query staging exceeds complete cap"};
  forecast.startup_payload_bytes = budget.bytes();
  output = next;
  return {};
}
} // namespace tlfea::contact::contact_facets
