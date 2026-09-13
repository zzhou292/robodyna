// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../FixedContactFacetValues.h"
#include "../SurfaceMaterialMeasure.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace tlfea::contact::active_use {
namespace {
using S = SelfContactActiveUseStatus;
SelfContactActiveUseReport Fail(S status, const char* message,
    std::size_t parent = SIZE_MAX, std::size_t feature = SIZE_MAX) noexcept {
  return {status, parent, feature, message};
}
bool Less(const FacetVertexKey& a, const FacetVertexKey& b) noexcept {
  if (a.source_instance_id != b.source_instance_id) return a.source_instance_id < b.source_instance_id;
  if (a.kind != b.kind) return a.kind < b.kind;
  if (a.first != b.first) return a.first < b.first;
  if (a.second != b.second) return a.second < b.second;
  if (a.numerator != b.numerator) return a.numerator < b.numerator;
  if (a.denominator != b.denominator) return a.denominator < b.denominator;
  if (a.level != b.level) return a.level < b.level;
  if (a.grid_i != b.grid_i) return a.grid_i < b.grid_i;
  return a.grid_j < b.grid_j;
}
bool Less(const FacetEdgeKey& a, const FacetEdgeKey& b) noexcept {
  if (Less(a.endpoints[0], b.endpoints[0])) return true;
  if (Less(b.endpoints[0], a.endpoints[0])) return false;
  if (Less(a.endpoints[1], b.endpoints[1])) return true;
  if (Less(b.endpoints[1], a.endpoints[1])) return false;
  if (a.parent_boundary != b.parent_boundary) return a.parent_boundary < b.parent_boundary;
  return a.parent_eid < b.parent_eid;
}
bool SamePoint(const WeightedSurfacePoint& a, const WeightedSurfacePoint& b) noexcept {
  if (a.count != b.count) return false;
  for (unsigned i = 0; i < 4; ++i)
    if (a.nodes[i] != b.nodes[i] || a.weights[i] != b.weights[i]) return false;
  return true;
}
bool Contains(const tl::constraints::tied_shell::cin::ActiveWitness& witness,
    std::uint32_t node) noexcept {
  for (const auto candidate : witness.nodes) if (candidate == node) return true;
  return false;
}
bool ValidWitness(const tl::constraints::tied_shell::cin::ActiveWitness& witness,
    const tl::constraints::tied_shell::CinAttachmentRow& row,
    std::size_t nodes) noexcept {
  namespace cin = tl::constraints::tied_shell::cin;
  if (!witness.source_element_id) return false;
  const bool triangle = witness.family == cin::WitnessFamily::ShellTriangle;
  if (!triangle && witness.family != cin::WitnessFamily::ShellQuad) return false;
  if (triangle && witness.nodes[2] != witness.nodes[3]) return false;
  const unsigned distinct = triangle ? 3 : 4;
  for (const auto node : witness.nodes) if (node >= nodes) return false;
  for (unsigned i = 0; i < distinct; ++i)
    for (unsigned j = i+1; j < distinct; ++j)
      if (witness.nodes[i] == witness.nodes[j]) return false;
  for (const auto node : row.master_domain_nodes) if (!Contains(witness, node)) return false;
  return true;
}
bool PrepareArea(const SelfContactSurfaceParent& source,
    const tl::fea::NodalNodeDomain& domain, SelfContactParentUse& output) noexcept {
  double xyz[12]{};
  for (unsigned i = 0; i < source.arity; ++i) {
    const auto node = source.arity == 4 ? source.q4.nodes[i] : source.t3.nodes[i];
    const auto value = domain.nodes()[node].position;
    xyz[3*i] = value.x; xyz[3*i+1] = value.y; xyz[3*i+2] = value.z;
    output.nodes[i] = node;
  }
  const VectorView positions{xyz, source.arity, 3, 1};
  if (source.arity == 4) {
    SurfaceQ4 parent;
    parent.feature_id = parent.parent_element_id = source.source.source_parent_id;
    for (unsigned i = 0; i < 4; ++i) parent.nodes[i] = i;
    Q4MaterialMeasure measure;
    if (PrepareQ4MaterialMeasure(positions, parent, &measure) != SurfaceMeasureStatus::Ok)
      return false;
    Q4CertifiedIntegral density;
    Q4IntegralInterval area;
    if (EvaluateQ4MaterialDensity(measure, 0, 0, &density) != SurfaceMeasureStatus::Ok ||
        !q4_bounds::Scale({density.lower, density.upper}, 4, &area) ||
        !q4_bounds::Certify(4*density.value, area, &output.reference_area_m2))
      return false;
    output.area_model = SelfContactReferenceAreaModel::Q4CenterAreaContactModel;
    return output.reference_area_m2.lower > 0;
  }
  SurfaceTriangle parent;
  parent.feature_id = parent.parent_element_id = source.source.source_parent_id;
  parent.interpolation = SurfaceInterpolation::kLinearTriangle;
  for (unsigned i = 0; i < 3; ++i) parent.nodes[i] = i;
  T3MaterialMeasure measure;
  if (PrepareT3MaterialMeasure(positions, parent, &measure) != SurfaceMeasureStatus::Ok ||
      !q4_bounds::Certify(.5*measure.density().value, measure.area_enclosure(),
          &output.reference_area_m2))
    return false;
  output.area_model = SelfContactReferenceAreaModel::T3CertifiedNativeArea;
  return output.reference_area_m2.lower > 0;
}
bool Dual(Q4CertifiedIntegral area, std::uint32_t valence, std::uint32_t facets,
    Q4CertifiedIntegral& dual, Q4CertifiedIntegral& directed) noexcept {
  if (!valence || !facets || facets > SIZE_MAX/3) return false;
  Q4IntegralInterval product, divided, half;
  const double factor = static_cast<double>(valence);
  const double divisor = static_cast<double>(3*std::size_t(facets));
  const double value = (area.value*factor)/divisor;
  if (!q4_bounds::Scale({area.lower, area.upper}, factor, &product) ||
      !q4_bounds::DividePositive(product, divisor, &divided) ||
      !q4_bounds::Certify(value, divided, &dual) ||
      !q4_bounds::DividePositive(divided, 2, &half) ||
      !q4_bounds::Certify(dual.value/2, half, &directed))
    return false;
  return dual.lower > 0 && directed.lower > 0;
}

// std::sort moves only these uint32 source ordinals. The payload permutation
// below is linear and each record is copied once per nontrivial cycle.
constexpr std::uint32_t VisitedOrder = std::uint32_t{1} << 31;
bool InvertOrder(std::uint32_t* order, std::size_t count) noexcept {
  if (!order || count >= VisitedOrder) return false;
  for (std::uint32_t start = 0; start < count; ++start) {
    if (order[start] & VisitedOrder) continue;
    std::uint32_t previous = start;
    std::uint32_t current = order[start];
    while (current != start) {
      if (current >= count || (order[current] & VisitedOrder)) return false;
      const auto next = order[current];
      order[current] = previous | VisitedOrder;
      previous = current;
      current = next;
    }
    order[start] = previous | VisitedOrder;
  }
  for (std::size_t i = 0; i < count; ++i) order[i] &= ~VisitedOrder;
  return true;
}
template<class T>
bool ApplyOrder(T* values, std::uint32_t* order, std::size_t count) noexcept {
  if (!values || !order) return false;
  for (std::size_t destination = 0; destination < count; ++destination) {
    if (order[destination] == destination) continue;
    T saved = values[destination];
    std::size_t current = destination;
    while (order[current] != destination) {
      const auto source = order[current];
      if (source >= count) return false;
      values[current] = values[source];
      order[current] = static_cast<std::uint32_t>(current);
      current = source;
    }
    values[current] = saved;
    order[current] = static_cast<std::uint32_t>(current);
  }
  return true;
}
}

SelfContactActiveUseReport Classify(const Inventory& inventory,
    const SelfContactActiveUseForecast& forecast, const WeightedSurfacePoint& point,
    SelfContactSupportClassification& output) noexcept {
  if (ValidateWeightedSurfacePoint(point, forecast.node_roles) != Status::kOk)
    return Fail(S::InvalidInput, "Weighted support map is invalid");
  SelfContactSupportClassification next;
  std::uint32_t common = UINT32_MAX;
  bool same_group = true;
  for (unsigned i = 0; i < point.count; ++i) {
    if (point.weights[i] == 0) continue;
    ++next.nonzero_slots;
    const auto& role = inventory.node_roles[point.nodes[i]];
    if (role.cin_secondary) next.status = SelfContactSupportStatus::UnsupportedCinSecondary;
    if (role.cin_master) ++next.cin_master_slots;
    if (role.rigid_group != UINT32_MAX) {
      ++next.rigid_slots;
      if (common == UINT32_MAX) common = role.rigid_group;
      else if (common != role.rigid_group) same_group = false;
    } else {
      same_group = false;
    }
  }
  if (next.status != SelfContactSupportStatus::UnsupportedCinSecondary) {
    if (next.rigid_slots == next.nonzero_slots && same_group) {
      next.status = SelfContactSupportStatus::CompleteRigidGroup;
      next.complete_rigid_group = common;
    } else if (next.rigid_slots) {
      next.status = SelfContactSupportStatus::AdmittedPartialOrMixedRigid;
    } else if (next.cin_master_slots) {
      next.status = SelfContactSupportStatus::AdmittedCinMaster;
    }
  }
  output = next;
  return {};
}

bool MapMatchesParent(const SelfContactParentUse& parent,
    const WeightedSurfacePoint& point) noexcept {
  if (point.count != parent.arity) return false;
  for (unsigned i = 0; i < parent.arity; ++i)
    if (point.nodes[i] != parent.nodes[i]) return false;
  return true;
}

bool ValidateActivity(const SelfContactActiveUseForecast& forecast,
    SelfContactActivityView activity) noexcept {
  if (!activity.base || !activity.current ||
      activity.parent_count != forecast.parents ||
      activity.parent_count > SIZE_MAX/sizeof(std::uint8_t))
    return false;
  const auto base = reinterpret_cast<std::uintptr_t>(activity.base);
  const auto current = reinterpret_cast<std::uintptr_t>(activity.current);
  if (activity.parent_count > UINTPTR_MAX-base ||
      activity.parent_count > UINTPTR_MAX-current)
    return false;
  for (std::size_t i = 0; i < activity.parent_count; ++i)
    if (activity.base[i] > 1 || activity.current[i] > activity.base[i]) return false;
  return true;
}

SelfContactActiveUseReport Build(const FixedContactFacetBinding& facets,
    SelfContactActiveUseSource source, const Layout& layout,
    BuildScratch scratch, Inventory& out) noexcept {
  const auto& forecast = layout.forecast;
  const auto& surface = *facets.surface();
  const auto& domain = *surface.physical()->domain();
  // Roles are immutable actual-domain facts. No PID, graph or geometry enters.
  if (source.rigid) {
    for (std::size_t group = 0; group < source.rigid->groups().size(); ++group) {
      const auto& row = source.rigid->groups()[group];
      if (group > UINT32_MAX || row.member_offset > source.rigid->members().size() ||
          row.member_count > source.rigid->members().size()-row.member_offset)
        return Fail(S::IdentityMismatch, "Rigid group ranges are invalid");
      for (std::size_t i = 0; i < row.member_count; ++i) {
        const auto node = source.rigid->members()[row.member_offset+i].domain_node;
        if (node >= forecast.node_roles || out.node_roles[node].rigid_group != UINT32_MAX)
          return Fail(S::IdentityMismatch, "Rigid membership is outside or duplicated in S0");
        out.node_roles[node].rigid_group = static_cast<std::uint32_t>(group);
      }
    }
  }
  if (forecast.cin_rows) {
    const auto rows = source.cin.model->rows();
    out.cin_rows = rows.data;
    std::size_t offset = 0;
    for (std::size_t r = 0; r < forecast.cin_rows; ++r) {
      const auto range = source.cin.ranges[r];
      if (range.offset != offset || !range.count || range.count > 4 ||
          range.count > forecast.cin_witnesses-offset)
        return Fail(S::IdentityMismatch, "CIN witness ranges are not a complete source-ordered roster", r);
      const auto& row = rows.data[r];
      if (row.secondary_domain_node >= forecast.node_roles ||
          out.node_roles[row.secondary_domain_node].cin_secondary)
        return Fail(S::IdentityMismatch, "CIN secondary role is outside or duplicated", r);
      out.node_roles[row.secondary_domain_node].cin_secondary = 1;
      for (const auto node : row.master_domain_nodes) {
        if (node >= forecast.node_roles)
          return Fail(S::IdentityMismatch, "CIN master role is outside S0", r);
        out.node_roles[node].cin_master = 1;
      }
      out.cin_ranges[r] = range;
      for (std::size_t w = offset; w < offset+range.count; ++w) {
        if (!ValidWitness(source.cin.witnesses[w], row, forecast.node_roles))
          return Fail(S::IdentityMismatch, "CIN witness does not authenticate its complete native support", r, w);
        out.cin_witnesses[w] = source.cin.witnesses[w];
      }
      offset += range.count;
    }
    if (offset != forecast.cin_witnesses)
      return Fail(S::IdentityMismatch, "CIN source has trailing witnesses");
  }

  for (std::size_t p = 0; p < forecast.parents; ++p) {
    const auto& source_parent = surface.parents()[p];
    auto& parent = out.parents[p];
    parent.surface_parent = p;
    parent.source = source_parent.source;
    parent.arity = source_parent.arity;
    parent.level = facets.config().level;
    parent.reference_half_thickness_m = source_parent.reference_half_thickness_m;
    if (!std::isfinite(parent.reference_half_thickness_m) ||
        !(parent.reference_half_thickness_m > 0) ||
        !PrepareArea(source_parent, domain, parent))
      return Fail(S::ReferenceFailure, "Certified parent reference area could not be prepared", p);
  }
  std::sort(out.parents, out.parents+forecast.parents,
      [](const auto& a, const auto& b) {
        return a.source.source_parent_id < b.source.source_parent_id;
      });
  std::size_t facet_offset = 0;
  for (std::size_t p = 0; p < forecast.parents; ++p) {
    auto& parent = out.parents[p];
    const auto count = facets.facet_count(parent.surface_parent);
    if (!count || facet_offset > UINT32_MAX || count > UINT32_MAX ||
        count > forecast.facets-facet_offset)
      return Fail(S::IdentityMismatch, "Parent facet range is invalid", p);
    parent.facet_offset = static_cast<std::uint32_t>(facet_offset);
    parent.facet_count = static_cast<std::uint32_t>(count);
    facet_offset += count;
  }
  if (facet_offset != forecast.facets)
    return Fail(S::IdentityMismatch, "Parent facet ranges do not cover the inventory");

  FixedContactFacet source_facet;
  FixedContactFacetReadCursor facet_reader;
  if (facet_reader.Initialize(facets, &source_facet).status !=
      FixedContactFacetStatus::Ok)
    return Fail(S::IdentityMismatch,
        "Fixed facet descriptor cursor could not be prepared");
  std::size_t vertex_cursor = 0, edge_cursor = 0;
  for (std::size_t p = 0; p < forecast.parents; ++p) {
    const auto& parent = out.parents[p];
    std::array<LocalVertex, MaxLocalVertices> local_vertices{};
    std::array<LocalEdge, MaxLocalEdges> local_edges{};
    std::size_t vertex_count = 0, edge_count = 0;
    for (std::uint32_t local = 0; local < parent.facet_count; ++local) {
      const auto report = facet_reader.Describe(parent.surface_parent, local);
      const auto& facet = source_facet;
      if (report.status != FixedContactFacetStatus::Ok ||
          facet.source.source_parent_id != parent.source.source_parent_id)
        return Fail(S::IdentityMismatch, "Fixed facet descriptor differs from its parent", p, local);
      auto& incidence = out.facets[std::size_t(parent.facet_offset)+local];
      incidence.parent = static_cast<std::uint32_t>(p);
      incidence.local_facet = local;
      for (unsigned slot = 0; slot < 3; ++slot) {
        std::size_t found = vertex_count;
        for (std::size_t i = 0; i < vertex_count; ++i)
          if (SameFacetVertexKey(local_vertices[i].key, facet.vertex_keys[slot])) { found = i; break; }
        if (found == vertex_count) {
          if (vertex_count == local_vertices.size())
            return Fail(S::IdentityMismatch, "Parent-local vertex template exceeds level-2 bound", p);
          local_vertices[found].key = facet.vertex_keys[slot];
          local_vertices[found].point = facet.vertices[slot];
          ++vertex_count;
        } else if (!SamePoint(local_vertices[found].point, facet.vertices[slot])) {
          return Fail(S::IdentityMismatch, "One parent-local vertex key has conflicting weighted maps", p);
        }
        ++local_vertices[found].valence;
        if (vertex_cursor+found > UINT32_MAX)
          return Fail(S::IdentityMismatch, "Facet vertex-use ordinal is unrepresentable", p, local);
        // Provisional source-order use ordinal; remapped after index sorting.
        incidence.vertex_uses[slot] =
            static_cast<std::uint32_t>(vertex_cursor+found);

        const auto& key = facet.edge_keys[slot];
        std::size_t edge_found = edge_count;
        for (std::size_t i = 0; i < edge_count; ++i)
          if (SameFacetEdgeKey(local_edges[i].key, key)) { edge_found = i; break; }
        WeightedSurfacePoint endpoints[2];
        for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
          bool matched = false;
          for (unsigned v = 0; v < 3; ++v)
            if (SameFacetVertexKey(key.endpoints[endpoint], facet.vertex_keys[v])) {
              endpoints[endpoint] = facet.vertices[v]; matched = true; break;
            }
          if (!matched)
            return Fail(S::IdentityMismatch, "Facet edge endpoint is absent from its triangle", p);
        }
        if (edge_found == edge_count) {
          if (edge_count == local_edges.size())
            return Fail(S::IdentityMismatch, "Parent-local edge template exceeds level-2 bound", p);
          local_edges[edge_found].key = key;
          local_edges[edge_found].endpoints[0] = endpoints[0];
          local_edges[edge_found].endpoints[1] = endpoints[1];
          ++edge_count;
        } else if (!SamePoint(local_edges[edge_found].endpoints[0], endpoints[0]) ||
                   !SamePoint(local_edges[edge_found].endpoints[1], endpoints[1])) {
          return Fail(S::IdentityMismatch, "One parent-local edge key has conflicting weighted maps", p);
        }
        ++local_edges[edge_found].valence;
        if (edge_cursor+edge_found > UINT32_MAX)
          return Fail(S::IdentityMismatch, "Facet edge-use ordinal is unrepresentable", p, local);
        incidence.edge_uses[slot] =
            static_cast<std::uint32_t>(edge_cursor+edge_found);
      }
    }
    if (vertex_count > forecast.vertex_uses ||
        vertex_cursor > forecast.vertex_uses-vertex_count ||
        edge_count > forecast.edge_uses ||
        edge_cursor > forecast.edge_uses-edge_count)
      return Fail(S::IdentityMismatch, "Parent-local feature count exceeds forecast", p);
    for (std::size_t i = 0; i < vertex_count; ++i) {
      auto& use = out.vertex_uses[vertex_cursor++];
      use.parent = static_cast<std::uint32_t>(p);
      use.key = local_vertices[i].key;
      use.point = local_vertices[i].point;
      use.facet_valence = local_vertices[i].valence;
      auto support = Classify(out, forecast, use.point, use.support);
      if (support.status != S::Ok) return support;
      if (!Dual(parent.reference_area_m2, use.facet_valence, parent.facet_count,
              use.dual_area_m2, use.directed_vf_area_m2))
        return Fail(S::Unrepresentable, "Directed vertex dual area is unrepresentable", p, i);
    }
    for (std::size_t i = 0; i < edge_count; ++i) {
      auto& use = out.edge_uses[edge_cursor++];
      use.parent = static_cast<std::uint32_t>(p);
      use.key = local_edges[i].key;
      use.endpoints[0] = local_edges[i].endpoints[0];
      use.endpoints[1] = local_edges[i].endpoints[1];
      use.facet_valence = local_edges[i].valence;
      for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
        auto support = Classify(out, forecast, use.endpoints[endpoint],
            use.endpoint_support[endpoint]);
        if (support.status != S::Ok) return support;
      }
    }
  }
  if (vertex_cursor != forecast.vertex_uses || edge_cursor != forecast.edge_uses)
    return Fail(S::IdentityMismatch, "Parent-local feature uses do not fill their exact forecast");

  if (!scratch.vertex_order || !scratch.edge_order ||
      forecast.vertex_uses >= VisitedOrder || forecast.edge_uses >= VisitedOrder)
    return Fail(S::ResourceLimit, "Canonical-order index range is unrepresentable");
  for (std::size_t i = 0; i < forecast.vertex_uses; ++i)
    scratch.vertex_order[i] = static_cast<std::uint32_t>(i);
  for (std::size_t i = 0; i < forecast.edge_uses; ++i)
    scratch.edge_order[i] = static_cast<std::uint32_t>(i);
  std::sort(scratch.vertex_order,
      scratch.vertex_order+forecast.vertex_uses,
      [&](std::uint32_t ai, std::uint32_t bi) {
        const auto& a = out.vertex_uses[ai];
        const auto& b = out.vertex_uses[bi];
        if (Less(a.key, b.key)) return true;
        if (Less(b.key, a.key)) return false;
        return out.parents[a.parent].source.source_parent_id <
            out.parents[b.parent].source.source_parent_id;
      });
  std::sort(scratch.edge_order,
      scratch.edge_order+forecast.edge_uses,
      [&](std::uint32_t ai, std::uint32_t bi) {
        const auto& a = out.edge_uses[ai];
        const auto& b = out.edge_uses[bi];
        if (Less(a.key, b.key)) return true;
        if (Less(b.key, a.key)) return false;
        return out.parents[a.parent].source.source_parent_id <
            out.parents[b.parent].source.source_parent_id;
      });
  // Convert destination->source to source->destination long enough to remap
  // the facet incidences captured during the only descriptor traversal.
  if (!InvertOrder(scratch.vertex_order, forecast.vertex_uses) ||
      !InvertOrder(scratch.edge_order, forecast.edge_uses))
    return Fail(S::IdentityMismatch, "Canonical-order index is not a permutation");
  for (std::size_t f = 0; f < forecast.facets; ++f) {
    auto& facet = out.facets[f];
    for (unsigned slot = 0; slot < 3; ++slot) {
      if (facet.vertex_uses[slot] >= forecast.vertex_uses ||
          facet.edge_uses[slot] >= forecast.edge_uses)
        return Fail(S::IdentityMismatch,
            "Facet source-order feature use is absent", facet.parent,
            facet.local_facet);
      facet.vertex_uses[slot] =
          scratch.vertex_order[facet.vertex_uses[slot]];
      facet.edge_uses[slot] =
          scratch.edge_order[facet.edge_uses[slot]];
    }
  }
  if (!InvertOrder(scratch.vertex_order, forecast.vertex_uses) ||
      !InvertOrder(scratch.edge_order, forecast.edge_uses) ||
      !ApplyOrder(out.vertex_uses, scratch.vertex_order,
          forecast.vertex_uses) ||
      !ApplyOrder(out.edge_uses, scratch.edge_order, forecast.edge_uses))
    return Fail(S::IdentityMismatch,
        "Canonical feature-use payload permutation failed");

  std::size_t vertex_feature = 0;
  for (std::size_t i = 0; i < forecast.vertex_uses;) {
    const auto key = out.vertex_uses[i].key;
    const auto begin = i;
    while (i < forecast.vertex_uses &&
        SameFacetVertexKey(key, out.vertex_uses[i].key)) ++i;
    if (vertex_feature >= forecast.vertices || begin > UINT32_MAX ||
        i-begin > UINT32_MAX)
      return Fail(S::IdentityMismatch, "Canonical vertex count differs from forecast");
    auto& feature = out.vertices[vertex_feature];
    feature.key = key;
    feature.use_offset = static_cast<std::uint32_t>(begin);
    feature.use_count = static_cast<std::uint32_t>(i-begin);
    for (std::size_t u = begin; u < i; ++u)
      out.vertex_uses[u].feature = static_cast<std::uint32_t>(vertex_feature);
    ++vertex_feature;
  }
  if (vertex_feature != forecast.vertices)
    return Fail(S::IdentityMismatch, "Canonical vertex deduplication differs from exact topology");
  std::size_t edge_feature = 0;
  for (std::size_t i = 0; i < forecast.edge_uses;) {
    const auto key = out.edge_uses[i].key;
    const auto begin = i;
    while (i < forecast.edge_uses &&
        SameFacetEdgeKey(key, out.edge_uses[i].key)) ++i;
    if (edge_feature >= forecast.edges || begin > UINT32_MAX ||
        i-begin > UINT32_MAX)
      return Fail(S::IdentityMismatch, "Canonical edge count differs from forecast");
    auto& feature = out.edges[edge_feature];
    feature.key = key;
    feature.use_offset = static_cast<std::uint32_t>(begin);
    feature.use_count = static_cast<std::uint32_t>(i-begin);
    for (std::size_t u = begin; u < i; ++u)
      out.edge_uses[u].feature = static_cast<std::uint32_t>(edge_feature);
    ++edge_feature;
  }
  if (edge_feature != forecast.edges)
    return Fail(S::IdentityMismatch, "Canonical edge deduplication differs from exact topology");

  for (std::size_t e = 0; e < forecast.edges; ++e)
    out.edges[e].vertices[0] = out.edges[e].vertices[1] = UINT32_MAX;
  for (std::size_t f = 0; f < forecast.facets; ++f) {
    auto& facet = out.facets[f];
    for (unsigned slot = 0; slot < 3; ++slot) {
      if (facet.vertex_uses[slot] >= forecast.vertex_uses ||
          facet.edge_uses[slot] >= forecast.edge_uses)
        return Fail(S::IdentityMismatch, "Facet feature use is absent",
            facet.parent, facet.local_facet);
      const auto& vertex = out.vertex_uses[facet.vertex_uses[slot]];
      const auto& edge = out.edge_uses[facet.edge_uses[slot]];
      if (vertex.parent != facet.parent || edge.parent != facet.parent ||
          vertex.feature >= forecast.vertices || edge.feature >= forecast.edges)
        return Fail(S::IdentityMismatch, "Facet feature use is absent",
            facet.parent, facet.local_facet);
      facet.vertex_features[slot] = vertex.feature;
      facet.edge_features[slot] = edge.feature;
    }
    // Every canonical edge endpoint is authenticated directly by a facet that
    // uses it. Repeated incidences must resolve to the identical vertex feature.
    for (unsigned slot = 0; slot < 3; ++slot) {
      auto& edge = out.edges[facet.edge_features[slot]];
      for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
        std::uint32_t vertex = UINT32_MAX;
        for (unsigned candidate = 0; candidate < 3; ++candidate) {
          const auto feature = facet.vertex_features[candidate];
          if (SameFacetVertexKey(edge.key.endpoints[endpoint],
                  out.vertices[feature].key)) {
            vertex = feature;
            break;
          }
        }
        if (vertex == UINT32_MAX)
          return Fail(S::IdentityMismatch,
              "Canonical edge endpoint vertex is absent", facet.parent,
              facet.edge_features[slot]);
        if (edge.vertices[endpoint] == UINT32_MAX)
          edge.vertices[endpoint] = vertex;
        else if (edge.vertices[endpoint] != vertex)
          return Fail(S::IdentityMismatch,
              "Canonical edge endpoint vertex conflicts", facet.parent,
              facet.edge_features[slot]);
      }
    }
  }
  for (std::size_t e = 0; e < forecast.edges; ++e)
    if (out.edges[e].vertices[0] == UINT32_MAX ||
        out.edges[e].vertices[1] == UINT32_MAX)
      return Fail(S::IdentityMismatch,
          "Canonical edge endpoint vertex is absent", SIZE_MAX, e);
  return {};
}
} // namespace tlfea::contact::active_use
