// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include "../FixedContactFacetValues.h"

#include <algorithm>
#include <cmath>

namespace tlfea::contact::self_contact_transaction {
namespace {

using S = SelfContactTransactionStatus;

SelfContactTransactionReport Failure(
    S status, const char* message,
    std::size_t candidate = SIZE_MAX,
    std::size_t pair = SIZE_MAX) noexcept {
  SelfContactTransactionReport result;
  result.status = status;
  result.candidate = candidate;
  result.pair = pair;
  result.message = message;
  return result;
}

SelfContactTransactionReport FeatureFailure(
    SelfContactTransactionReport report,
    const FixedTriangleFeatureCandidate& feature,
    std::size_t candidate) noexcept {
  report.candidate = candidate;
  report.offending_motion[0].facet = feature.triangles[0];
  report.offending_motion[1].facet = feature.triangles[1];
  report.offending_feature_distance_m = feature.distance_m;
  report.offending_edge_parameters[0] =
      feature.edge_parameters[0];
  report.offending_edge_parameters[1] =
      feature.edge_parameters[1];
  return report;
}

FixedTriangleKey DescriptorKey(
    const FixedContactFacet& value) noexcept {
  return {value.source_instance_id, value.source.source_parent_id,
          value.level, value.local_facet};
}

bool DescriptorLess(const FixedContactFacet& a,
                    const FixedContactFacet& b) noexcept {
  return fixed_triangle_features::Compare(
             DescriptorKey(a), DescriptorKey(b)) < 0;
}

std::size_t TriangleIndex(
    const FixedContactFacet* descriptors,
    const std::uint32_t* order, std::size_t count,
    const FixedTriangleKey& key) noexcept {
  std::size_t lower = 0, upper = count;
  while (lower < upper) {
    const auto middle = lower + (upper - lower) / 2;
    const auto index = order[middle];
    if (fixed_triangle_features::Compare(
            DescriptorKey(descriptors[index]), key) < 0)
      lower = middle + 1;
    else
      upper = middle;
  }
  return lower < count &&
          fixed_triangle_features::Compare(
              DescriptorKey(descriptors[order[lower]]), key) == 0
      ? order[lower] : SIZE_MAX;
}

bool AddRigidGroup(std::uint32_t group,
                   MotionSupport* output) noexcept {
  for (unsigned i = 0; i < output->rigid_group_count; ++i)
    if (output->rigid_groups[i] == group) return true;
  if (output->rigid_group_count == 4) return false;
  output->rigid_groups[output->rigid_group_count++] = group;
  return true;
}

void SortRigidGroups(MotionSupport* output) noexcept {
  std::sort(output->rigid_groups,
            output->rigid_groups + output->rigid_group_count);
}

bool PointMotion(const WeightedSurfacePoint& point,
                 const SelfContactSupportClassification& support,
                 const std::uint32_t* node_rigid_groups,
                 std::size_t nodes, MotionSupport* output) noexcept {
  if (!node_rigid_groups || !output ||
      ValidateWeightedSurfacePoint(point, nodes) != Status::kOk)
    return false;
  MotionSupport next;
  std::size_t nonzero = 0, rigid = 0;
  std::uint32_t common = UINT32_MAX;
  bool same = true;
  for (unsigned slot = 0; slot < point.count; ++slot) {
    if (point.weights[slot] == 0) continue;
    ++nonzero;
    const auto group = node_rigid_groups[point.nodes[slot]];
    if (group == UINT32_MAX) {
      same = false;
      continue;
    }
    ++rigid;
    if (!AddRigidGroup(group, &next)) return false;
    if (common == UINT32_MAX) common = group;
    else if (common != group) same = false;
  }
  if (nonzero != support.nonzero_slots ||
      rigid != support.rigid_slots)
    return false;
  if (!rigid) {
    if (support.status ==
            SelfContactSupportStatus::CompleteRigidGroup ||
        support.status ==
            SelfContactSupportStatus::AdmittedPartialOrMixedRigid)
      return false;
    next.motion = SelfContactFacetMotion::LinearNodalV1;
  } else if (rigid == nonzero && same) {
    if (support.status !=
            SelfContactSupportStatus::UnsupportedCinSecondary &&
        (support.status !=
             SelfContactSupportStatus::CompleteRigidGroup ||
         support.complete_rigid_group != common))
      return false;
    next.motion = SelfContactFacetMotion::CompleteRigidGroup;
    next.complete_rigid_group = common;
  } else {
    if (support.status !=
            SelfContactSupportStatus::UnsupportedCinSecondary &&
        support.status !=
            SelfContactSupportStatus::AdmittedPartialOrMixedRigid)
      return false;
    next.motion = SelfContactFacetMotion::PartialOrMixedRigid;
  }
  SortRigidGroups(&next);
  *output = next;
  return true;
}

bool FacetMotion(const SelfContactActiveUseBinding& active_use,
                 std::size_t facet,
                 const std::uint32_t* node_rigid_groups,
                 std::size_t nodes, MotionSupport* output) noexcept {
  if (!output || facet >= active_use.facet_uses().size()) return false;
  const auto& use = active_use.facet_uses()[facet];
  MotionSupport next;
  next.parent = use.parent;
  bool all_linear = true, all_complete = true;
  std::uint32_t common = UINT32_MAX;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    if (use.vertex_uses[vertex] >= active_use.vertex_uses().size())
      return false;
    const auto& value =
        active_use.vertex_uses()[use.vertex_uses[vertex]];
    if (value.parent != use.parent) return false;
    MotionSupport point;
    if (!PointMotion(value.point, value.support,
                     node_rigid_groups, nodes, &point))
      return false;
    all_linear = all_linear &&
        point.motion == SelfContactFacetMotion::LinearNodalV1;
    all_complete = all_complete &&
        point.motion == SelfContactFacetMotion::CompleteRigidGroup;
    if (point.motion == SelfContactFacetMotion::CompleteRigidGroup) {
      if (common == UINT32_MAX)
        common = point.complete_rigid_group;
      else if (common != point.complete_rigid_group)
        all_complete = false;
    }
    for (unsigned group = 0;
         group < point.rigid_group_count; ++group)
      if (!AddRigidGroup(point.rigid_groups[group], &next))
        return false;
  }
  if (all_linear) {
    next.motion = SelfContactFacetMotion::LinearNodalV1;
  } else if (all_complete && common != UINT32_MAX) {
    next.motion = SelfContactFacetMotion::CompleteRigidGroup;
    next.complete_rigid_group = common;
  } else {
    next.motion = SelfContactFacetMotion::PartialOrMixedRigid;
  }
  SortRigidGroups(&next);
  *output = next;
  return true;
}

bool ParentMotion(const SelfContactParentUse& parent,
                  const std::uint32_t* node_rigid_groups,
                  std::size_t nodes, std::uint32_t parent_index,
                  MotionSupport* output) noexcept {
  if (!node_rigid_groups || !output ||
      (parent.arity != 3 && parent.arity != 4))
    return false;
  MotionSupport next;
  next.parent = parent_index;
  bool all_rigid = true;
  std::uint32_t common = UINT32_MAX;
  for (unsigned slot = 0; slot < parent.arity; ++slot) {
    if (parent.nodes[slot] >= nodes) return false;
    const auto group = node_rigid_groups[parent.nodes[slot]];
    if (group == UINT32_MAX) {
      all_rigid = false;
      continue;
    }
    if (!AddRigidGroup(group, &next)) return false;
    if (common == UINT32_MAX) common = group;
    else if (common != group) all_rigid = false;
  }
  if (!next.rigid_group_count) {
    next.motion = SelfContactFacetMotion::LinearNodalV1;
  } else if (all_rigid && next.rigid_group_count == 1) {
    next.motion = SelfContactFacetMotion::CompleteRigidGroup;
    next.complete_rigid_group = common;
  } else {
    next.motion = SelfContactFacetMotion::PartialOrMixedRigid;
  }
  SortRigidGroups(&next);
  *output = next;
  return true;
}

bool AllowedExclusion(SelfContactPairStatus status) noexcept {
  return status == SelfContactPairStatus::ExcludedLocalIncidence ||
      status == SelfContactPairStatus::ExcludedSameRigidGroup ||
      status == SelfContactPairStatus::ExcludedRegularOwnParent;
}

SelfContactTransactionReport VertexFaceEvent(
    const SelfContactActiveUseBinding& active_use,
    const SelfContactCurrentRegularity& regularity,
    const SelfContactCurrentRegularityReceipt& regularity_receipt,
    const FixedTriangleFeatureCandidate& feature,
    const FixedContactFacet* descriptors,
    const std::uint32_t* triangle_order, std::size_t facet_count,
    SelfContactActivityView activity, std::uint64_t source_order,
    SelfContactForceEvent* output,
    AcceptedEventCertificate* certificate,
    bool* admitted) noexcept {
  *admitted = false;
  const bool first_vertex = feature.local_features[0] < 3;
  if (first_vertex == (feature.local_features[1] < 3))
    return Failure(S::IdentityMismatch,
        "Directed VF provenance does not identify one vertex and one face");
  const unsigned vertex_side = first_vertex ? 0 : 1;
  const unsigned target_side = 1 - vertex_side;
  if (feature.local_features[target_side] != 3)
    return Failure(S::IdentityMismatch,
        "Directed VF target provenance is not a face");
  const auto vertex_facet = TriangleIndex(
      descriptors, triangle_order, facet_count,
      feature.triangles[vertex_side]);
  const auto target_facet = TriangleIndex(
      descriptors, triangle_order, facet_count,
      feature.triangles[target_side]);
  if (vertex_facet == SIZE_MAX || target_facet == SIZE_MAX)
    return Failure(S::IdentityMismatch,
        "Directed VF provenance is absent from the complete facet inventory");
  const auto facets = active_use.facet_uses();
  const unsigned local_vertex = feature.local_features[vertex_side];
  if (vertex_facet >= facets.size() || target_facet >= facets.size() ||
      local_vertex >= 3)
    return Failure(S::IdentityMismatch,
        "Directed VF active-use facet provenance is invalid");
  const auto vertex_use =
      facets[vertex_facet].vertex_uses[local_vertex];
  if (vertex_use >= active_use.vertex_uses().size() ||
      fixed_triangle_features::Compare(
          active_use.vertex_uses()[vertex_use].key,
          feature.key.vertex_face.vertex) != 0)
    return Failure(S::IdentityMismatch,
        "Directed VF canonical vertex differs from its active use");

  WeightedSurfacePoint face_point;
  const auto nodes = active_use.facets()->surface()->physical()->
      domain()->node_count();
  if (ComposeFacetPoint(descriptors[target_facet],
                        feature.face_weights,
                        static_cast<std::uint32_t>(nodes),
                        &face_point) != Status::kOk)
    return Failure(S::DiscoveryFailure,
        "Directed VF face weights cannot be composed exactly");
  SelfContactPairClassification classification;
  const auto classified = active_use.ClassifyVertexFace(
      vertex_use, target_facet, face_point, activity,
      &classification);
  if (classified.status != SelfContactActiveUseStatus::Ok)
    return Failure(S::IdentityMismatch, classified.message);
  if (classification.status ==
      SelfContactPairStatus::SameParentNeedsCurrentRegularity) {
    SelfContactPairClassification excluded;
    const auto resolved = regularity.ExcludeCertifiedOwnParent(
        classification, regularity_receipt, &excluded);
    if (resolved.status != SelfContactCurrentRegularityStatus::Ok) {
      auto report = Failure(S::RegularityFailure, resolved.message);
      report.regularity_status = resolved.status;
      return report;
    }
    classification = excluded;
  }
  if (classification.status == SelfContactPairStatus::InactiveParent)
    return {};
  if (AllowedExclusion(classification.status)) return {};
  if (classification.status !=
      SelfContactPairStatus::AdmittedVertexFace)
    return Failure(S::CandidateRejected,
        "Directed VF support is unresolved or unsupported by policy");

  *admitted = true;
  if (!output) return {};
  SelfContactForceEvent event;
  event.feature = feature.key;
  event.source_order = source_order;
  event.vertex_use = static_cast<std::uint32_t>(vertex_use);
  event.facet_use = static_cast<std::uint32_t>(target_facet);
  event.endpoints[0] = active_use.vertex_uses()[vertex_use].point;
  event.endpoints[1] = face_point;
  event.classification = classification;
  *output = event;
  certificate->kind = AcceptedEventCertificateKind::VertexFace;
  certificate->event = event;
  certificate->discovery = feature;
  certificate->vertex_facet =
      static_cast<std::uint32_t>(vertex_facet);
  certificate->target_facet =
      static_cast<std::uint32_t>(target_facet);
  return {};
}

bool EdgePoint(
    const SelfContactActiveUseBinding& active_use,
    const FixedContactFacet* descriptors,
    const std::uint32_t* triangle_order, std::size_t facet_count,
    const FixedTriangleFeatureCandidate& feature,
    unsigned canonical_edge, WeightedSurfacePoint* point,
    std::size_t* edge_use, std::size_t* edge_facet) noexcept {
  const auto& key = feature.key.edge_edge.edges[canonical_edge];
  const auto facets = active_use.facet_uses();
  for (unsigned side = 0; side < 2; ++side) {
    const auto triangle = TriangleIndex(
        descriptors, triangle_order, facet_count,
        feature.triangles[side]);
    if (triangle == SIZE_MAX || triangle >= facets.size()) return false;
    for (unsigned local = 0; local < 3; ++local) {
      if (fixed_triangle_features::Compare(
              descriptors[triangle].edge_keys[local], key) != 0)
        continue;
      if (feature.local_features[side] != local) return false;
      double weights[3]{};
      const double parameter = feature.edge_parameters[canonical_edge];
      if (!std::isfinite(parameter) ||
          parameter < 0 || parameter > 1)
        return false;
      bool first = false, second = false;
      for (unsigned vertex = 0; vertex < 3; ++vertex) {
        if (fixed_triangle_features::Compare(
                descriptors[triangle].vertex_keys[vertex],
                key.endpoints[0]) == 0) {
          weights[vertex] = 1 - parameter;
          first = true;
        }
        if (fixed_triangle_features::Compare(
                descriptors[triangle].vertex_keys[vertex],
                key.endpoints[1]) == 0) {
          weights[vertex] = parameter;
          second = true;
        }
      }
      const auto nodes = active_use.facets()->surface()->physical()->
          domain()->node_count();
      if (!first || !second ||
          ComposeFacetPoint(descriptors[triangle], weights,
                            static_cast<std::uint32_t>(nodes),
                            point) != Status::kOk)
        return false;
      *edge_use = facets[triangle].edge_uses[local];
      *edge_facet = triangle;
      return *edge_use < active_use.edge_uses().size();
    }
  }
  return false;
}

SelfContactTransactionReport EdgeEdgeEvent(
    const SelfContactActiveUseBinding& active_use,
    const SelfContactCurrentRegularity& regularity,
    const SelfContactCurrentRegularityReceipt& regularity_receipt,
    const FixedTriangleFeatureCandidate& feature,
    const FixedContactFacet* descriptors,
    const std::uint32_t* triangle_order, std::size_t facet_count,
    SelfContactActivityView activity, std::uint64_t source_order,
    SelfContactForceEvent* output,
    AcceptedEventCertificate* certificate,
    bool* admitted) noexcept {
  *admitted = false;
  WeightedSurfacePoint points[2];
  std::size_t edge_uses[2]{};
  std::size_t edge_facets[2]{};
  if (!EdgePoint(active_use, descriptors, triangle_order, facet_count,
                 feature, 0, points, edge_uses, edge_facets) ||
      !EdgePoint(active_use, descriptors, triangle_order, facet_count,
                 feature, 1, points + 1, edge_uses + 1, edge_facets + 1))
    return Failure(S::IdentityMismatch,
        "EE provenance cannot be mapped to exact active uses");
  SelfContactPairClassification classification;
  const bool strict_interior =
      feature.edge_parameters[0] > 0 &&
      feature.edge_parameters[0] < 1 &&
      feature.edge_parameters[1] > 0 &&
      feature.edge_parameters[1] < 1;
  const auto edge_case =
      feature.distance_m == 0
          ? SelfContactEdgeEdgeCase::ZeroDistance
          : (strict_interior
                 ? SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum
                 : SelfContactEdgeEdgeCase::
                       BoundaryVertexEdgeMinimum);
  const auto classified = active_use.ClassifyEdgeEdge(
      edge_uses[0], points[0], edge_uses[1], points[1],
      edge_case, activity, &classification);
  if (classified.status != SelfContactActiveUseStatus::Ok)
    return Failure(S::IdentityMismatch, classified.message);
  if (classification.status ==
      SelfContactPairStatus::SameParentNeedsCurrentRegularity) {
    SelfContactPairClassification excluded;
    const auto resolved = regularity.ExcludeCertifiedOwnParent(
        classification, regularity_receipt, &excluded);
    if (resolved.status != SelfContactCurrentRegularityStatus::Ok) {
      auto report = Failure(S::RegularityFailure, resolved.message);
      report.regularity_status = resolved.status;
      return report;
    }
    classification = excluded;
  }
  if (!std::isfinite(feature.distance_m) ||
      feature.distance_m < 0)
    return Failure(S::DiscoveryFailure,
        "EE distance is invalid");
  const double gap =
      (feature.distance_m -
       classification.reference_half_thickness_m[0]) -
      classification.reference_half_thickness_m[1];
  if (!std::isfinite(gap))
    return Failure(S::DiscoveryFailure,
        "EE represented gap is unrepresentable");
  if (gap > 0 || AllowedExclusion(classification.status))
    return {};
  if (classification.status == SelfContactPairStatus::InactiveParent)
    return {};
  if (classification.status != SelfContactPairStatus::AdmittedEdgeEdge)
    return Failure(S::CandidateRejected,
        "Nonlocal EE contact lacks strict authenticated edge-point area");

  *admitted = true;
  if (!output) return {};
  if (edge_uses[0] > UINT32_MAX || edge_uses[1] > UINT32_MAX ||
      edge_facets[0] > UINT32_MAX || edge_facets[1] > UINT32_MAX)
    return Failure(S::ResourceLimit,
        "EE active-use provenance is unrepresentable");
  SelfContactForceEvent event;
  event.feature = feature.key;
  event.source_order = source_order;
  event.edge_use[0] = static_cast<std::uint32_t>(edge_uses[0]);
  event.edge_use[1] = static_cast<std::uint32_t>(edge_uses[1]);
  event.endpoints[0] = points[0];
  event.endpoints[1] = points[1];
  event.classification = classification;
  *output = event;
  certificate->kind = AcceptedEventCertificateKind::EdgeEdge;
  certificate->event = event;
  certificate->discovery = feature;
  certificate->edge_facet[0] =
      static_cast<std::uint32_t>(edge_facets[0]);
  certificate->edge_facet[1] =
      static_cast<std::uint32_t>(edge_facets[1]);
  return {};
}

bool ExactBoundaryVertexFace(
    const FixedTriangleFeatureCandidate& vertex_face,
    const FixedTriangleFeatureCandidate& edge_edge) noexcept {
  if (vertex_face.key.kind !=
          FixedTriangleCandidateKind::VertexFace ||
      edge_edge.key.kind != FixedTriangleCandidateKind::EdgeEdge ||
      !ExactFacetPair(vertex_face, edge_edge))
    return false;
  const auto& vf = vertex_face.key.vertex_face;
  for (unsigned endpoint_edge = 0;
       endpoint_edge < 2; ++endpoint_edge) {
    const auto parameter =
        edge_edge.edge_parameters[endpoint_edge];
    if (parameter != 0 && parameter != 1) continue;
    const auto& vertex =
        edge_edge.key.edge_edge.edges[endpoint_edge].
            endpoints[parameter == 1];
    if (fixed_triangle_features::Compare(
            vf.vertex, vertex) != 0)
      continue;
    const auto other = 1 - endpoint_edge;
    if (vf.target.kind == FixedTriangleStratumKind::Edge &&
        fixed_triangle_features::Compare(
            vf.target.edge,
            edge_edge.key.edge_edge.edges[other]) == 0)
      return true;
    const auto other_parameter =
        edge_edge.edge_parameters[other];
    if ((other_parameter == 0 || other_parameter == 1) &&
        vf.target.kind == FixedTriangleStratumKind::Vertex &&
        fixed_triangle_features::Compare(
            vf.target.vertex,
            edge_edge.key.edge_edge.edges[other].
                endpoints[other_parameter == 1]) == 0)
      return true;
  }
  return false;
}

SelfContactTransactionReport CoveredByExactBoundaryVertexFace(
    const SelfContactActiveUseBinding& active_use,
    const SelfContactCurrentRegularity& regularity,
    const SelfContactCurrentRegularityReceipt& regularity_receipt,
    FixedTriangleFeatureView features,
    const FixedTriangleFeatureCandidate& edge_edge,
    const FixedContactFacet* descriptors,
    const std::uint32_t* triangle_order, std::size_t facet_count,
    SelfContactActivityView activity, bool* covered) noexcept {
  *covered = false;
  for (std::size_t i = 0; i < features.count; ++i) {
    const auto& candidate = features.data[i];
    if (!ExactBoundaryVertexFace(candidate, edge_edge))
      continue;
    bool admitted = false;
    const auto report = VertexFaceEvent(
        active_use, regularity, regularity_receipt, candidate,
        descriptors, triangle_order, facet_count, activity,
        0, nullptr, nullptr, &admitted);
    if (report.status != S::Ok) return report;
    if (admitted) {
      *covered = true;
      return {};
    }
  }
  return {};
}

bool SameEdgeEdgeCertificateIdentity(
    const FixedTriangleFeatureCandidate& a,
    const FixedTriangleFeatureCandidate& b) noexcept {
  if (a.key.kind != FixedTriangleCandidateKind::EdgeEdge ||
      b.key.kind != FixedTriangleCandidateKind::EdgeEdge ||
      !self_contact_transaction::Same(a.key, b.key) ||
      !self_contact_transaction::Same(a.triangles[0], b.triangles[0]) ||
      !self_contact_transaction::Same(a.triangles[1], b.triangles[1]) ||
      a.local_features[0] != b.local_features[0] ||
      a.local_features[1] != b.local_features[1])
    return false;
  return true;
}

}  // namespace

SelfContactTransactionReport InitializeStaticPipeline(
    const SelfContactActiveUseBinding& active_use, Buffers buffers,
    std::size_t nodes, std::size_t surface_parents,
    std::size_t facets) noexcept {
  if (!nodes || !buffers.node_rigid_groups ||
      !buffers.surface_to_active ||
      !buffers.parent_facet_offsets || !buffers.facet_descriptors ||
      !buffers.parent_motion || !buffers.facet_motion ||
      !buffers.triangle_order || !buffers.vertex_identity_order ||
      !buffers.edge_identity_order)
    return Failure(S::InvalidInput,
        "Static transaction pipeline storage is incomplete");
  std::fill_n(buffers.node_rigid_groups, nodes, UINT32_MAX);
  if (const auto* rigid = active_use.rigid()) {
    const auto groups = rigid->groups();
    const auto members = rigid->members();
    for (std::size_t group = 0; group < groups.size(); ++group) {
      const auto& value = groups[group];
      if (group > UINT32_MAX ||
          value.member_offset > members.size() ||
          value.member_count >
              members.size() - value.member_offset)
        return Failure(S::IdentityMismatch,
            "Actual rigid binding group range is invalid", group);
      for (std::size_t local = 0;
           local < value.member_count; ++local) {
        const auto node =
            members[value.member_offset + local].domain_node;
        if (node >= nodes ||
            buffers.node_rigid_groups[node] != UINT32_MAX)
          return Failure(S::IdentityMismatch,
              "Actual rigid binding membership is outside or duplicated",
              group, local);
        buffers.node_rigid_groups[node] =
            static_cast<std::uint32_t>(group);
      }
    }
  }
  std::fill_n(buffers.surface_to_active, surface_parents, UINT32_MAX);
  const auto parents = active_use.parents();
  if (!parents.size() || active_use.facet_uses().size() != facets)
    return Failure(S::IdentityMismatch,
        "Active-use parent/facet inventory is incomplete");
  const auto* facet_binding = active_use.facets();
  if (!facet_binding)
    return Failure(S::IdentityMismatch,
        "Active-use facet binding is absent");
  FixedContactFacetReadCursor facet_reader;
  const auto initialized = facet_reader.Initialize(*facet_binding);
  if (initialized.status != FixedContactFacetStatus::Ok)
    return Failure(S::DiscoveryFailure, initialized.message);
  std::size_t next_facet = 0;
  for (std::size_t parent = 0; parent < parents.size(); ++parent) {
    const auto& value = parents[parent];
    if (value.surface_parent >= surface_parents ||
        buffers.surface_to_active[value.surface_parent] != UINT32_MAX ||
        value.facet_offset != next_facet ||
        value.facet_count > facets - next_facet)
      return Failure(S::IdentityMismatch,
          "Surface-parent to active-use map is not complete and one-to-one",
          parent);
    buffers.surface_to_active[value.surface_parent] =
        static_cast<std::uint32_t>(parent);
    buffers.parent_facet_offsets[parent] =
        static_cast<std::uint32_t>(next_facet);
    if (!ParentMotion(
            value, buffers.node_rigid_groups, nodes,
            static_cast<std::uint32_t>(parent),
            buffers.parent_motion + parent))
      return Failure(S::IdentityMismatch,
          "Parent rigid-motion support differs from actual binding",
          parent);
    for (std::uint32_t local = 0; local < value.facet_count; ++local) {
      const auto global = next_facet + local;
      const auto described =
          facet_reader.Describe(value.surface_parent, local);
      if (described.report.status != FixedContactFacetStatus::Ok ||
          !described.facet)
        return Failure(
            S::DiscoveryFailure, described.report.message, global);
      buffers.facet_descriptors[global] = *described.facet;
      const auto& use = active_use.facet_uses()[global];
      if (use.parent != parent || use.local_facet != local ||
          buffers.facet_descriptors[global].source.source_parent_id !=
              value.source.source_parent_id)
        return Failure(S::IdentityMismatch,
            "Facet descriptor differs from active-use incidence", global);
      if (!FacetMotion(
              active_use, global, buffers.node_rigid_groups,
              nodes, buffers.facet_motion + global) ||
          buffers.facet_motion[global].parent != parent)
        return Failure(S::IdentityMismatch,
            "Facet rigid-motion support differs from authenticated active use",
            parent, global);
      buffers.triangle_order[global] =
          static_cast<std::uint32_t>(global);
      for (unsigned local_vertex = 0; local_vertex < 3; ++local_vertex) {
        const auto encoded =
            static_cast<std::uint32_t>(3 * global + local_vertex);
        buffers.vertex_identity_order[3 * global + local_vertex] =
            encoded;
        buffers.edge_identity_order[3 * global + local_vertex] =
            encoded;
      }
    }
    next_facet += value.facet_count;
  }
  buffers.parent_facet_offsets[parents.size()] =
      static_cast<std::uint32_t>(next_facet);
  if (next_facet != facets)
    return Failure(S::IdentityMismatch,
        "Parent facet offsets do not cover the complete inventory");
  for (std::size_t surface = 0; surface < surface_parents; ++surface)
    if (buffers.surface_to_active[surface] == UINT32_MAX)
      return Failure(S::IdentityMismatch,
          "An S0 broadphase parent has no selected active use", surface);
  std::sort(buffers.triangle_order,
            buffers.triangle_order + facets,
            [&](std::uint32_t a, std::uint32_t b) {
              return DescriptorLess(buffers.facet_descriptors[a],
                                    buffers.facet_descriptors[b]);
            });
  for (std::size_t i = 1; i < facets; ++i)
    if (!DescriptorLess(
            buffers.facet_descriptors[buffers.triangle_order[i - 1]],
            buffers.facet_descriptors[buffers.triangle_order[i]]))
      return Failure(S::IdentityMismatch,
          "Complete facet identities are not unique", i);
  std::sort(
      buffers.vertex_identity_order,
      buffers.vertex_identity_order + 3 * facets,
      [&](std::uint32_t a, std::uint32_t b) {
        return fixed_triangle_features::Compare(
            buffers.facet_descriptors[a / 3].vertex_keys[a % 3],
            buffers.facet_descriptors[b / 3].vertex_keys[b % 3]) < 0;
      });
  std::sort(
      buffers.edge_identity_order,
      buffers.edge_identity_order + 3 * facets,
      [&](std::uint32_t a, std::uint32_t b) {
        return fixed_triangle_features::Compare(
            buffers.facet_descriptors[a / 3].edge_keys[a % 3],
            buffers.facet_descriptors[b / 3].edge_keys[b % 3]) < 0;
      });
  return {};
}

SelfContactTransactionReport FilterSameRigidFacetPairs(
    const MotionSupport* motion, std::size_t facets,
    FixedTrianglePair* pairs, std::size_t* pair_count) noexcept {
  if (!motion || !facets || !pairs || !pair_count)
    return Failure(S::InvalidInput,
        "Rigid facet-pair filter storage is incomplete");
  std::size_t write = 0;
  for (std::size_t pair = 0; pair < *pair_count; ++pair) {
    const auto value = pairs[pair];
    if (value.first >= facets || value.second >= facets ||
        value.first == value.second)
      return Failure(S::IdentityMismatch,
          "Rigid facet-pair filter received an invalid exact pair",
          SIZE_MAX, pair);
    const auto action = ClassifyCandidatePairMotion(
        motion[value.first], {}, motion[value.second], {});
    if (action == PairMotionAction::ExcludedSameRigidGroup)
      continue;
    pairs[write++] = value;
  }
  *pair_count = write;
  return {};
}

bool CompleteRegularity(
    const SelfContactActiveUseBinding& active_use,
    const SelfContactCurrentRegularityReceipt& receipt,
    SelfContactCurrentRegularityView view,
    SelfContactActivityView activity) noexcept {
  const auto parents = active_use.parents();
  const auto source_instance_id =
      active_use.facets()->surface()->physical()->
          domain()->source_instance_id();
  if (!receipt.prepared() || !view.complete ||
      !activity.base || !activity.current ||
      activity.parent_count != parents.size() ||
      view.count != parents.size() ||
      view.summary.generation != receipt.generation() ||
      view.summary.parents != parents.size())
    return false;

  std::size_t active = 0;
  std::size_t removing = 0;
  std::size_t skipped = 0;
  std::size_t certified = 0;
  std::size_t facets_evaluated = 0;
  for (std::size_t parent = 0; parent < parents.size(); ++parent) {
    if (activity.base[parent] > 1 ||
        activity.current[parent] > activity.base[parent])
      return false;
    const auto& expected = parents[parent];
    const auto& result = view.data[parent];
    if (result.source_instance_id != source_instance_id ||
        result.source_eid != expected.source.source_parent_id ||
        result.binding_parent != parent ||
        result.surface_parent != expected.surface_parent ||
        result.arity != expected.arity ||
        result.level != expected.level ||
        result.facet_count != expected.facet_count)
      return false;
    if (!activity.base[parent]) {
      ++skipped;
      if (result.state !=
              SelfContactCurrentParentState::LongInactiveSkipped ||
          result.chart !=
              SelfContactCurrentChartStatus::SkippedLongInactive ||
          result.geometry_evaluated || result.facets_evaluated)
        return false;
      continue;
    }
    ++certified;
    active += activity.current[parent] != 0;
    removing += activity.current[parent] == 0;
    facets_evaluated += expected.facet_count;
    const auto expected_state = activity.current[parent]
        ? SelfContactCurrentParentState::Active
        : SelfContactCurrentParentState::Removing;
    if (result.state != expected_state ||
        result.chart ==
            SelfContactCurrentChartStatus::SkippedLongInactive ||
        !result.geometry_evaluated ||
        result.facets_evaluated != result.facet_count)
      return false;
  }
  return view.summary.certified_parents == certified &&
      view.summary.active_parents == active &&
      view.summary.removing_parents == removing &&
      view.summary.skipped_parents == skipped &&
      view.summary.facets_evaluated == facets_evaluated;
}

SelfContactTransactionReport EvaluateCompleteTriangles(
    const FixedContactFacet* descriptors, std::size_t count,
    VectorView positions, CurrentFixedTriangle* output) noexcept {
  if (!descriptors || !count || !output)
    return Failure(S::InvalidInput,
        "Complete facet evaluation storage is absent");
  for (std::size_t facet = 0; facet < count; ++facet)
    if (EvaluateCurrentFixedTriangle(
            descriptors[facet], positions, output + facet) !=
        Status::kOk)
      return Failure(S::DiscoveryFailure,
          "Actual owner facet geometry cannot be represented", facet);
  return {};
}

SelfContactTransactionReport ReadBroadphase(
    const SelfContactBroadphase& broadphase, cudaStream_t stream,
    SelfContactPairKey* host_keys, std::size_t broadphase_capacity,
    std::size_t* broadphase_count) noexcept {
  if (!stream || !host_keys || !broadphase_count)
    return Failure(S::InvalidInput,
        "Broadphase readback storage is incomplete");
  const auto pairs = broadphase.pairs();
  if (!pairs.complete || pairs.count > broadphase_capacity ||
      pairs.count > SIZE_MAX ||
      (pairs.count && !pairs.device_keys) ||
      (!pairs.count && pairs.device_keys))
    return Failure(S::BroadphaseFailure,
        "Broadphase did not publish one complete bounded pair set");
  if (pairs.count) {
    const auto copied = cudaMemcpyAsync(
        host_keys, pairs.device_keys,
        static_cast<std::size_t>(pairs.count) * sizeof(*host_keys),
        cudaMemcpyDeviceToHost, stream);
    if (copied != cudaSuccess) {
      auto report = Failure(S::BroadphaseFailure,
                            cudaGetErrorString(copied));
      report.broadphase_status =
          SelfContactBroadphaseStatus::DeviceFailure;
      return report;
    }
    const auto synchronized = cudaStreamSynchronize(stream);
    if (synchronized != cudaSuccess) {
      auto report = Failure(S::BroadphaseFailure,
                            cudaGetErrorString(synchronized));
      report.broadphase_status =
          SelfContactBroadphaseStatus::DeviceFailure;
      return report;
    }
  }
  *broadphase_count = static_cast<std::size_t>(pairs.count);
  return {};
}

SelfContactTransactionReport BuildAcceptedEvents(
    const SelfContactActiveUseBinding& active_use,
    const SelfContactCurrentRegularity& regularity,
    const SelfContactCurrentRegularityReceipt& regularity_receipt,
    FixedTriangleFeatureView features,
    FixedTriangleIntersectionView intersections,
    const FixedContactFacet* descriptors,
    const std::uint32_t* triangle_order, std::size_t facet_count,
    SelfContactActivityView activity,
    SelfContactForceEvent* events,
    AcceptedEventCertificate* certificates,
    std::size_t capacity, std::size_t* count) noexcept {
  if (!features.complete || !intersections.complete ||
      (features.count && !features.data) ||
      (intersections.count && !intersections.data) ||
      !descriptors || !triangle_order || !events || !certificates ||
      !capacity || !count)
    return Failure(S::InvalidInput,
        "Accepted discovery publication or event storage is incomplete");
  for (std::size_t i = 0; i < intersections.count; ++i)
    if (RequiresIntersectionAdmission(intersections.data[i]))
      return Failure(S::CandidateRejected,
          "Accepted nonlocal triangle intersection is rejected", i);

  std::size_t required = 0;
  for (std::size_t feature = 0; feature < features.count; ++feature) {
    const auto& value = features.data[feature];
    bool admitted = false;
    auto checked =
        value.key.kind == FixedTriangleCandidateKind::VertexFace
            ? VertexFaceEvent(
                  active_use, regularity, regularity_receipt, value,
                  descriptors, triangle_order, facet_count, activity,
                  required, nullptr, nullptr, &admitted)
            : EdgeEdgeEvent(
                  active_use, regularity, regularity_receipt, value,
                  descriptors, triangle_order, facet_count, activity,
                  required, nullptr, nullptr, &admitted);
    if (value.key.kind == FixedTriangleCandidateKind::EdgeEdge &&
        checked.status == S::CandidateRejected) {
      bool covered = false;
      const auto coverage = CoveredByExactBoundaryVertexFace(
          active_use, regularity, regularity_receipt, features, value,
          descriptors, triangle_order, facet_count, activity, &covered);
      if (coverage.status != S::Ok) return coverage;
      if (covered) continue;
    }
    if (checked.status != S::Ok)
      return FeatureFailure(checked, value, feature);
    required += admitted;
  }
  if (required > capacity)
    return Failure(S::ResourceLimit,
        "Complete accepted VF+EE event set exceeds its exact capacity",
        required);

  std::size_t written = 0;
  for (std::size_t feature = 0; feature < features.count; ++feature) {
    const auto& value = features.data[feature];
    bool admitted = false;
    auto checked =
        value.key.kind == FixedTriangleCandidateKind::VertexFace
            ? VertexFaceEvent(
                  active_use, regularity, regularity_receipt, value,
                  descriptors, triangle_order, facet_count, activity,
                  written, events + written, certificates + written,
                  &admitted)
            : EdgeEdgeEvent(
                  active_use, regularity, regularity_receipt, value,
                  descriptors, triangle_order, facet_count, activity,
                  written, events + written, certificates + written,
                  &admitted);
    if (value.key.kind == FixedTriangleCandidateKind::EdgeEdge &&
        checked.status == S::CandidateRejected) {
      bool covered = false;
      const auto coverage = CoveredByExactBoundaryVertexFace(
          active_use, regularity, regularity_receipt, features, value,
          descriptors, triangle_order, facet_count, activity, &covered);
      if (coverage.status != S::Ok) return coverage;
      if (covered) continue;
    }
    if (checked.status != S::Ok)
      return FeatureFailure(checked, value, feature);
    written += admitted;
  }
  if (written != required)
    return Failure(S::IdentityMismatch,
        "Accepted event count changed between count and write");
  *count = written;
  return {};
}

SelfContactTransactionReport ValidateCandidateEdgePolicy(
    const SelfContactActiveUseBinding& active_use,
    const SelfContactCurrentRegularity& regularity,
    const SelfContactCurrentRegularityReceipt& regularity_receipt,
    FixedTriangleFeatureView features,
    const FixedContactFacet* descriptors,
    const std::uint32_t* triangle_order, std::size_t facet_count,
    SelfContactActivityView activity,
    const AcceptedEventCertificate* accepted,
    std::size_t accepted_count) noexcept {
  if (!features.complete || (features.count && !features.data) ||
      !descriptors || !triangle_order ||
      (accepted_count && !accepted))
    return Failure(S::InvalidInput,
        "Candidate feature publication is incomplete");
  for (std::size_t feature = 0; feature < features.count; ++feature) {
    if (features.data[feature].key.kind !=
        FixedTriangleCandidateKind::EdgeEdge)
      continue;
    bool admitted = false;
    const auto checked = EdgeEdgeEvent(
        active_use, regularity, regularity_receipt,
        features.data[feature], descriptors, triangle_order,
        facet_count, activity, 0, nullptr, nullptr, &admitted);
    if (checked.status != S::Ok) {
      if (checked.status == S::CandidateRejected) {
        bool covered = false;
        const auto coverage = CoveredByExactBoundaryVertexFace(
            active_use, regularity, regularity_receipt, features,
            features.data[feature], descriptors, triangle_order,
            facet_count, activity, &covered);
        if (coverage.status != S::Ok) return coverage;
        if (covered) continue;
      }
      return FeatureFailure(
          checked, features.data[feature], feature);
    }
    (void)admitted;
  }
  return {};
}

}  // namespace tlfea::contact::self_contact_transaction
