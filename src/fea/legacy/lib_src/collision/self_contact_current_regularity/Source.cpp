// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include "../FixedContactFacetValues.h"

#include <cstring>

namespace tlfea::contact::current_regularity {
namespace {

using S = SelfContactCurrentRegularityStatus;

SelfContactCurrentRegularityReport Fail(
    const SelfContactCurrentRegularityForecast& forecast,
    const char* message, std::size_t parent = SIZE_MAX,
    std::size_t facet = SIZE_MAX) noexcept {
  return {S::IdentityMismatch, parent, facet,
          forecast.parents, forecast.facets, message};
}

bool Same(const tl::fea::ShellPlasticityParentInput& a,
          const tl::fea::ShellPlasticityParentInput& b) noexcept {
  return a.family == b.family && a.family_index == b.family_index &&
      a.source_parent_id == b.source_parent_id &&
      a.source_part_id == b.source_part_id &&
      a.material_id == b.material_id && a.section_id == b.section_id;
}

bool Same(const WeightedSurfacePoint& a,
          const WeightedSurfacePoint& b) noexcept {
  if (a.count != b.count) return false;
  for (unsigned i = 0; i < 4; ++i)
    if (a.nodes[i] != b.nodes[i] || a.weights[i] != b.weights[i])
      return false;
  return true;
}

void CopyTemplate(const FixedContactFacet& source,
                  FacetTemplate& destination) noexcept {
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    std::memcpy(destination.weights[vertex],
                source.vertices[vertex].weights,
                sizeof(destination.weights[vertex]));
}

bool SameTemplate(const FacetTemplate& expected,
                  const FixedContactFacet& source) noexcept {
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    if (std::memcmp(expected.weights[vertex],
                    source.vertices[vertex].weights,
                    sizeof(expected.weights[vertex])) != 0)
      return false;
  return true;
}

}  // namespace

SelfContactCurrentRegularityReport ValidateSource(
    const SelfContactActiveUseBinding& binding,
    const SelfContactCurrentRegularityForecast& forecast,
    Templates* templates,
    FixedContactFacetReadCursor* retained_reader) noexcept {
  const auto* fixed = binding.facets();
  if (!fixed || !fixed->surface() ||
      forecast.parents != binding.parents().size() ||
      forecast.facets != binding.facet_uses().size())
    return Fail(forecast, "Retained active-use source is absent");
  const auto& surface = *fixed->surface();
  if (surface.parents().size() != forecast.parents)
    return Fail(forecast,
        "Active-use parents do not cover the complete S0 surface");
  const auto fixed_forecast = fixed->forecast();
  if (templates &&
      (!templates->q4 || !templates->t3 ||
       templates->q4_count != fixed_forecast.q4_template_facets ||
       templates->t3_count != fixed_forecast.t3_template_facets))
    return Fail(forecast,
        "Current compact template storage differs from fixed authority");
  FixedContactFacetReadCursor local_reader;
  auto* reader =
      retained_reader ? retained_reader : &local_reader;
  if (!retained_reader &&
      reader->Initialize(*fixed).status != FixedContactFacetStatus::Ok)
    return Fail(forecast,
        "Fixed facet descriptor cursor could not authenticate its source");
  const auto source_instance_id =
      surface.physical()->domain()->source_instance_id();

  std::size_t facet_cursor = 0;
  std::uint64_t previous_eid = 0;
  bool have_q4_template = false;
  bool have_t3_template = false;
  for (std::size_t p = 0; p < forecast.parents; ++p) {
    const auto& parent = binding.parents()[p];
    if (!parent.source.source_parent_id ||
        (p && parent.source.source_parent_id <= previous_eid) ||
        parent.surface_parent >= surface.parents().size())
      return Fail(forecast,
          "Parents are not in strict deterministic source-EID order", p);
    previous_eid = parent.source.source_parent_id;
    const auto& native = surface.parents()[parent.surface_parent];
    if (!Same(parent.source, native.source) ||
        parent.arity != native.arity ||
        (parent.arity != 3 && parent.arity != 4) ||
        parent.level != fixed->config().level ||
        parent.facet_offset != facet_cursor ||
        parent.facet_count != fixed->facet_count(parent.surface_parent) ||
        !parent.facet_count ||
        facet_cursor > forecast.facets ||
        parent.facet_count > forecast.facets-facet_cursor)
      return Fail(forecast,
          "Parent identity/range differs from retained fixed facets", p);
    for (unsigned i = 0; i < parent.arity; ++i) {
      const auto node =
          native.arity == 4 ? native.q4.nodes[i] : native.t3.nodes[i];
      if (parent.nodes[i] != node)
        return Fail(forecast,
            "Parent native node order differs from retained S0", p);
    }

    for (std::uint32_t local = 0; local < parent.facet_count; ++local) {
      const auto index = facet_cursor + local;
      const auto described =
          reader->Describe(parent.surface_parent, local);
      if (described.report.status != FixedContactFacetStatus::Ok ||
          !described.facet)
        return Fail(forecast,
            "Fixed facet descriptor does not authenticate its parent",
            p, local);
      const auto& descriptor = *described.facet;
      if (
          descriptor.source_instance_id != source_instance_id ||
          descriptor.parent_index != parent.surface_parent ||
          descriptor.source.source_parent_id !=
              parent.source.source_parent_id ||
          !Same(descriptor.source, parent.source) ||
          descriptor.level != parent.level ||
          descriptor.local_facet != local)
        return Fail(forecast,
            "Fixed facet descriptor does not authenticate its parent",
            p, local);
      const auto& use = binding.facet_uses()[index];
      if (use.parent != p || use.local_facet != local)
        return Fail(forecast,
            "Active-use facet roster is not complete source order",
            p, local);
      for (unsigned slot = 0; slot < 3; ++slot) {
        const auto feature = use.vertex_features[slot];
        const auto vertex_use = use.vertex_uses[slot];
        const auto edge = use.edge_features[slot];
        const auto edge_use = use.edge_uses[slot];
        if (feature >= binding.vertices().size() ||
            vertex_use >= binding.vertex_uses().size() ||
            edge >= binding.edges().size() ||
            edge_use >= binding.edge_uses().size())
          return Fail(forecast,
              "Facet feature-use index is outside complete inventory",
              p, local);
        const auto& vu = binding.vertex_uses()[vertex_use];
        const auto& eu = binding.edge_uses()[edge_use];
        if (vu.parent != p || vu.feature != feature ||
            !SameFacetVertexKey(
                descriptor.vertex_keys[slot],
                binding.vertices()[feature].key) ||
            !SameFacetVertexKey(descriptor.vertex_keys[slot], vu.key) ||
            !Same(descriptor.vertices[slot], vu.point) ||
            eu.parent != p || eu.feature != edge ||
            !SameFacetEdgeKey(
                descriptor.edge_keys[slot], binding.edges()[edge].key) ||
            !SameFacetEdgeKey(descriptor.edge_keys[slot], eu.key))
          return Fail(forecast,
              "Facet weighted map/canonical feature identity differs",
              p, local);
        for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
          unsigned vertex = 3;
          for (unsigned candidate = 0; candidate < 3; ++candidate)
            if (SameFacetVertexKey(
                    descriptor.vertex_keys[candidate],
                    descriptor.edge_keys[slot].endpoints[endpoint])) {
              vertex = candidate;
              break;
            }
          if (vertex == 3 ||
              !Same(descriptor.vertices[vertex],
                    eu.endpoints[endpoint]))
            return Fail(forecast,
                "Facet edge endpoint map differs from canonical use",
                p, local);
        }
      }
      if (templates) {
        auto* retained =
            parent.arity == 4 ? templates->q4 : templates->t3;
        const bool have =
            parent.arity == 4 ? have_q4_template : have_t3_template;
        if (!have)
          CopyTemplate(descriptor, retained[local]);
        else if (!SameTemplate(retained[local], descriptor))
          return Fail(forecast,
              "Parent facet weights differ from immutable fixed template",
              p, local);
      }
    }
    if (parent.arity == 4)
      have_q4_template = true;
    else
      have_t3_template = true;
    facet_cursor += parent.facet_count;
  }
  if (facet_cursor != forecast.facets)
    return Fail(forecast,
        "Parent facet ranges do not cover every fixed facet");
  return {S::Ok, SIZE_MAX, SIZE_MAX,
          forecast.parents, forecast.facets, "OK"};
}

}  // namespace tlfea::contact::current_regularity
