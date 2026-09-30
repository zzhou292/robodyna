// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "SelfContactActiveUseTypes.h"
#include <limits>

namespace tlfea::contact {
struct SelfContactActiveUseCountInput {
  std::size_t q4_parents = 0, t3_parents = 0;
  std::size_t source_vertices = 0, source_edges = 0;
  unsigned level = 0;
};

// Exact fixed-template counts. Every product/addition is checked before the
// result is published or any parent/facet payload is read by the binding.
inline bool CountSelfContactActiveUses(SelfContactActiveUseCountInput input,
    SelfContactActiveUseCounts* output) noexcept {
  if (!output || input.level > 2) return false;
  const auto add = [](std::size_t a, std::size_t b, std::size_t& out) {
    if (b > std::numeric_limits<std::size_t>::max() - a) return false;
    out = a + b;
    return true;
  };
  const auto multiply = [](std::size_t a, std::size_t b, std::size_t& out) {
    if (a && b > std::numeric_limits<std::size_t>::max() / a) return false;
    out = a * b;
    return true;
  };
  const std::size_t n = std::size_t{1} << input.level;
  const std::size_t q4_facets = 2*n*n, t3_facets = n*n;
  const std::size_t q4_vertices = (n+1)*(n+1);
  const std::size_t t3_vertices = (n+1)*(n+2)/2;
  const std::size_t q4_edges = 3*n*n+2*n;
  const std::size_t t3_edges = 3*n*(n+1)/2;
  std::size_t q = 0, t = 0;
  SelfContactActiveUseCounts next;
  if (!add(input.q4_parents, input.t3_parents, next.parents) ||
      !multiply(input.q4_parents, q4_facets, q) ||
      !multiply(input.t3_parents, t3_facets, t) || !add(q, t, next.facets) ||
      !multiply(input.q4_parents, q4_vertices, q) ||
      !multiply(input.t3_parents, t3_vertices, t) || !add(q, t, next.vertex_uses) ||
      !multiply(input.q4_parents, q4_edges, q) ||
      !multiply(input.t3_parents, t3_edges, t) || !add(q, t, next.edge_uses))
    return false;

  // Canonical boundary topology merges only by S0 source identity. Parent
  // interiors remain parent-owned; equal coordinates never enter these counts.
  std::size_t edge_points = 0, q4_interior = 0, t3_interior = 0;
  if (!multiply(input.source_edges, n-1, edge_points) ||
      !multiply(input.q4_parents, (n-1)*(n-1), q4_interior) ||
      !multiply(input.t3_parents, (n-1)*(n>1 ? n-2 : 0)/2, t3_interior) ||
      !add(input.source_vertices, edge_points, next.vertices) ||
      !add(next.vertices, q4_interior, next.vertices) ||
      !add(next.vertices, t3_interior, next.vertices))
    return false;
  std::size_t boundary = 0, q4_internal = 0, t3_internal = 0;
  if (!multiply(input.source_edges, n, boundary) ||
      !multiply(input.q4_parents, q4_edges-4*n, q4_internal) ||
      !multiply(input.t3_parents, t3_edges-3*n, t3_internal) ||
      !add(boundary, q4_internal, next.edges) ||
      !add(next.edges, t3_internal, next.edges))
    return false;
  *output = next;
  return true;
}
} // namespace tlfea::contact
