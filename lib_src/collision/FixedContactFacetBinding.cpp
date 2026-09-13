// SPDX-License-Identifier: AGPL-3.0-or-later
#include "contact_facets/Storage.h"
#include "../solvers/NodalTrialIdentity.h"
#include <new>

namespace tlfea::contact {
FixedContactFacetPreflight FixedContactFacetBinding::Preflight(const SelfContactSurfaceBinding& source,
    FixedContactFacetConfig config, FixedContactFacetLimits limits) noexcept {
  contact_facets::Layout layout;
  const auto report = contact_facets::MakeLayout(source, config, limits, sizeof(Impl), layout);
  return {report, layout.forecast};
}
FixedContactFacetReport FixedContactFacetBinding::Initialize(const SelfContactSurfaceBinding& source,
    FixedContactFacetConfig config, FixedContactFacetLimits limits) noexcept try {
  using S = FixedContactFacetStatus;
  if (impl_) return {S::AlreadyInitialized, "Physical facet profile is immutable"};
  if (!source.OutputDisjoint(this, sizeof(*this))) return {S::InvalidInput, "Destination aliases source or source is absent"};
  contact_facets::Layout layout;
  const auto report = contact_facets::MakeLayout(source, config, limits, sizeof(Impl), layout);
  if (report.status != S::Ok) return report;
  auto next = std::make_shared<Impl>(source);
  if (!next->arena.Initialize(layout.forecast.template_arena_bytes)) return {S::ResourceLimit, "Template allocation failed"};
  auto& t = next->templates;
  t.q4_vertices = next->arena.Construct<contact_facets::Vertex>(layout.q4_vertices);
  t.t3_vertices = next->arena.Construct<contact_facets::Vertex>(layout.t3_vertices);
  t.q4_triangles = next->arena.Construct<contact_facets::Triangle>(layout.q4_triangles);
  t.t3_triangles = next->arena.Construct<contact_facets::Triangle>(layout.t3_triangles);
  if (!t.q4_vertices || !t.t3_vertices || !t.q4_triangles || !t.t3_triangles ||
      !contact_facets::BuildTemplates(config.level, t)) return {S::ResourceLimit, "Template construction rejected"};
  next->config = config;
  next->forecast = layout.forecast;
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {FixedContactFacetStatus::ResourceLimit, "Fixed facet storage allocation failed"};
}
const SelfContactSurfaceBinding* FixedContactFacetBinding::surface() const noexcept { return impl_ ? &impl_->surface : nullptr; }
FixedContactFacetConfig FixedContactFacetBinding::config() const noexcept { return impl_ ? impl_->config : FixedContactFacetConfig{}; }
FixedContactFacetForecast FixedContactFacetBinding::forecast() const noexcept { return impl_ ? impl_->forecast : FixedContactFacetForecast{}; }
bool FixedContactFacetBinding::SharesStorage(const FixedContactFacetBinding& other) const noexcept { return impl_ && impl_ == other.impl_; }
bool FixedContactFacetBinding::OutputDisjoint(const void* output, std::size_t bytes) const noexcept {
  return impl_ && tl::fea::trial_identity::Disjoint(output, bytes, this, sizeof(*this)) &&
      tl::fea::trial_identity::Disjoint(output, bytes, impl_.get(), sizeof(Impl)) &&
      tl::fea::trial_identity::Disjoint(output, bytes, impl_->arena.data(), impl_->arena.bytes()) &&
      impl_->surface.OutputDisjoint(output, bytes);
}
std::size_t FixedContactFacetBinding::facet_count(std::size_t parent) const noexcept {
  if (!impl_ || parent >= impl_->forecast.parents) return 0;
  return impl_->surface.parents()[parent].arity == 4 ? impl_->forecast.q4_template_facets : impl_->forecast.t3_template_facets;
}
FixedContactFacetReport FixedContactFacetBinding::Describe(std::size_t parent, unsigned local,
    FixedContactFacet* output) const noexcept {
  using S = FixedContactFacetStatus;
  if (!output || !OutputDisjoint(output, sizeof(*output))) return {S::InvalidInput, "Facet output aliases source or binding is absent"};
  if (local >= facet_count(parent)) return {S::OutOfRange, "Physical parent or facet index is out of range"};
  const auto& native = impl_->surface.parents()[parent];
  const auto* vertices = native.arity == 4 ? impl_->templates.q4_vertices : impl_->templates.t3_vertices;
  const auto& face = native.arity == 4 ? impl_->templates.q4_triangles[local] : impl_->templates.t3_triangles[local];
  FixedContactFacet next;
  next.source = native.source;
  next.law = native.law;
  next.material_points = native.material_points;
  next.source_instance_id = impl_->surface.physical()->domain()->source_instance_id();
  next.reference_half_thickness_m = native.reference_half_thickness_m;
  next.parent_index = parent;
  next.local_facet = local;
  next.level = impl_->config.level;
  for (unsigned v = 0; v < 3; ++v) {
    const auto& vertex = vertices[face.vertices[v]];
    next.vertices[v].count = native.arity;
    for (unsigned i = 0; i < native.arity; ++i) {
      next.vertices[v].nodes[i] = native.arity == 4 ? native.q4.nodes[i] : native.t3.nodes[i];
      next.vertices[v].weights[i] = vertex.weights[i];
    }
    next.vertex_keys[v] = contact_facets::VertexKey(impl_->surface, native, vertex, next.level);
  }
  for (unsigned e = 0; e < 3; ++e) {
    const unsigned end = (e + 1) % 3;
    next.edge_keys[e] = contact_facets::EdgeKey(vertices[face.vertices[e]], vertices[face.vertices[end]],
        next.vertex_keys[e], next.vertex_keys[end], native.arity, native.source.source_parent_id);
  }
  *output = next;
  return {};
}
Status FixedContactFacetBinding::Approximation(std::size_t parent, VectorView positions,
    FacetApproximationBound* output) const noexcept {
  if (!output || !positions.valid() || !OutputDisjoint(output, sizeof(*output))) return Status::kInvalidArgument;
  if (parent >= impl_->forecast.parents) return Status::kOutOfRange;
  if (positions.node_count != impl_->surface.physical()->domain()->node_count()) return Status::kInvalidArgument;
  const auto last = (positions.node_count - 1) * positions.node_stride + 2 * positions.component_stride;
  if (last >= SIZE_MAX / sizeof(double)) return Status::kInvalidArgument;
  const auto extent = (last + 1) * sizeof(double);
  if (!tl::fea::trial_identity::Disjoint(output, sizeof(*output), positions.data, extent)) return Status::kInvalidArgument;
  const auto& native = impl_->surface.parents()[parent];
  return contact_facets::MeasureApproximation(native, impl_->config.level,
      native.arity == 4 ? impl_->templates.q4_vertices : impl_->templates.t3_vertices,
      native.arity == 4 ? impl_->forecast.q4_template_vertices : impl_->forecast.t3_template_vertices,
      positions, output);
}
Status FixedContactFacetBinding::SummarizeApproximation(VectorView positions,
    FacetApproximationSummary* output) const noexcept {
  if (!output || !positions.valid() || !OutputDisjoint(output, sizeof(*output))) return Status::kInvalidArgument;
  if (positions.node_count != impl_->surface.physical()->domain()->node_count()) return Status::kInvalidArgument;
  const auto last = (positions.node_count - 1) * positions.node_stride + 2 * positions.component_stride;
  if (last >= SIZE_MAX / sizeof(double)) return Status::kInvalidArgument;
  const auto extent = (last + 1) * sizeof(double);
  if (!tl::fea::trial_identity::Disjoint(output, sizeof(*output), positions.data, extent)) return Status::kInvalidArgument;
  FacetApproximationSummary next;
  next.parents = impl_->forecast.parents;
  for (std::size_t parent = 0; parent < impl_->forecast.parents; ++parent) {
    const auto& native = impl_->surface.parents()[parent];
    FacetApproximationBound bound;
    const auto status = contact_facets::MeasureApproximation(native, impl_->config.level,
        native.arity == 4 ? impl_->templates.q4_vertices : impl_->templates.t3_vertices,
        native.arity == 4 ? impl_->forecast.q4_template_vertices : impl_->forecast.t3_template_vertices,
        positions, &bound);
    if (status != Status::kOk) return status;
    next.positive_bilinear_parents += bound.bilinear_error_upper_m > 0;
    next.positive_vertex_roundoff_parents += bound.vertex_roundoff_upper_m > 0;
    const auto maximum = [&](double value, double& current, std::size_t& witness) {
      if (value > current) { current = value; witness = parent; }
    };
    maximum(bound.bilinear_error_upper_m, next.maximum_bilinear_error_upper_m,
        next.maximum_bilinear_parent);
    maximum(bound.vertex_roundoff_upper_m, next.maximum_vertex_roundoff_upper_m,
        next.maximum_vertex_roundoff_parent);
    maximum(bound.total_error_upper_m, next.maximum_total_error_upper_m,
        next.maximum_total_parent);
  }
  *output = next;
  return Status::kOk;
}
} // namespace tlfea::contact
