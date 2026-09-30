// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../FixedContactFacetBinding.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::contact_facets {
struct Vertex { double weights[4]{}; unsigned i = 0, j = 0; };
struct Triangle { std::uint8_t vertices[3]{}; };
struct Layout {
  tl::util::ArenaRegion q4_vertices, t3_vertices, q4_triangles, t3_triangles;
  FixedContactFacetForecast forecast;
};
struct Templates {
  Vertex* q4_vertices = nullptr;
  Vertex* t3_vertices = nullptr;
  Triangle* q4_triangles = nullptr;
  Triangle* t3_triangles = nullptr;
};
FixedContactFacetReport MakeLayout(const SelfContactSurfaceBinding&, FixedContactFacetConfig,
    FixedContactFacetLimits, std::size_t implementation_bytes, Layout&) noexcept;
bool BuildTemplates(unsigned level, Templates&) noexcept;
FacetVertexKey VertexKey(const SelfContactSurfaceBinding&, const SelfContactSurfaceParent&,
    const Vertex&, unsigned level) noexcept;
FacetEdgeKey EdgeKey(const Vertex&, const Vertex&, const FacetVertexKey&, const FacetVertexKey&,
    unsigned arity, std::uint64_t parent_eid) noexcept;
Status MeasureApproximation(const SelfContactSurfaceParent&, unsigned level,
    const Vertex*, std::size_t count, VectorView, FacetApproximationBound*) noexcept;
} // namespace tlfea::contact::contact_facets

namespace tlfea::contact {
struct FixedContactFacetBinding::Impl {
  explicit Impl(const SelfContactSurfaceBinding& source) : surface(source) {}
  SelfContactSurfaceBinding surface;
  FixedContactFacetConfig config;
  FixedContactFacetForecast forecast;
  tl::util::HostArena arena;
  contact_facets::Templates templates;
};
} // namespace tlfea::contact
