// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <tuple>

namespace tlfea::contact::contact_facets {
namespace {
bool Less(const FacetVertexKey& a, const FacetVertexKey& b) noexcept {
  return std::tie(a.source_instance_id, a.kind, a.first, a.second, a.numerator,
      a.denominator, a.level, a.grid_i, a.grid_j) <
      std::tie(b.source_instance_id, b.kind, b.first, b.second, b.numerator,
      b.denominator, b.level, b.grid_i, b.grid_j);
}
}
FacetVertexKey VertexKey(const SelfContactSurfaceBinding& surface, const SelfContactSurfaceParent& parent,
    const Vertex& vertex, unsigned level) noexcept {
  FacetVertexKey key;
  key.source_instance_id = surface.physical()->domain()->source_instance_id();
  unsigned support[4]{}, count = 0;
  for (unsigned i = 0; i < parent.arity; ++i)
    if (vertex.weights[i] > 0) support[count++] = i;
  if (count == 1) {
    key.first = surface.vertices()[parent.vertices[support[0]]].source_node_id;
  } else if (count == 2) {
    key.kind = FacetVertexKind::SourceEdge;
    const auto a = surface.vertices()[parent.vertices[support[0]]].source_node_id;
    const auto b = surface.vertices()[parent.vertices[support[1]]].source_node_id;
    key.first = a < b ? a : b;
    key.second = a < b ? b : a;
    key.denominator = 1u << level;
    key.numerator = static_cast<unsigned>(vertex.weights[support[a < b ? 1 : 0]] * key.denominator);
    while (key.denominator > 1 && key.numerator % 2 == 0) {
      key.numerator /= 2;
      key.denominator /= 2;
    }
  } else {
    key.kind = FacetVertexKind::ParentInterior;
    key.first = parent.source.source_parent_id;
    key.level = level;
    key.grid_i = vertex.i;
    key.grid_j = vertex.j;
  }
  return key;
}
FacetEdgeKey EdgeKey(const Vertex& a, const Vertex& b, const FacetVertexKey& ka, const FacetVertexKey& kb,
    unsigned arity, std::uint64_t parent_eid) noexcept {
  unsigned support[4]{}, count = 0;
  for (unsigned i = 0; i < arity; ++i)
    if (a.weights[i] > 0 || b.weights[i] > 0) support[count++] = i;
  FacetEdgeKey key;
  key.parent_boundary = count == 2 &&
      ((support[0] + 1) % arity == support[1] || (support[1] + 1) % arity == support[0]);
  key.parent_eid = key.parent_boundary ? 0 : parent_eid;
  const bool reverse = Less(kb, ka);
  key.endpoints[0] = reverse ? kb : ka;
  key.endpoints[1] = reverse ? ka : kb;
  return key;
}
} // namespace tlfea::contact::contact_facets
