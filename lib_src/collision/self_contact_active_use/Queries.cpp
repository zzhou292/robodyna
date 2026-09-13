// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace tlfea::contact::active_use {
namespace {
using S = SelfContactActiveUseStatus;
SelfContactActiveUseReport Fail(const char* message, std::size_t parent = SIZE_MAX,
    std::size_t feature = SIZE_MAX) noexcept {
  return {S::InvalidInput, parent, feature, message};
}
bool RowContains(const tl::constraints::tied_shell::CinAttachmentRow& row,
    std::uint32_t node) noexcept {
  for (const auto master : row.master_domain_nodes) if (master == node) return true;
  return false;
}
bool WitnessContains(const tl::constraints::tied_shell::cin::ActiveWitness& witness,
    std::uint32_t node) noexcept {
  for (const auto candidate : witness.nodes) if (candidate == node) return true;
  return false;
}
bool WitnessAuthenticates(const Inventory& inventory,
    const SelfContactActiveUseForecast& forecast, std::size_t row_index,
    const SelfContactParentUse& parent) noexcept {
  namespace cin = tl::constraints::tied_shell::cin;
  const auto range = inventory.cin_ranges[row_index];
  for (std::size_t i = range.offset; i < std::size_t(range.offset)+range.count; ++i) {
    if (i >= forecast.cin_witnesses) return false;
    const auto& witness = inventory.cin_witnesses[i];
    const bool triangle = parent.arity == 3;
    if (witness.source_element_id != parent.source.source_parent_id ||
        witness.native_parent_index != parent.source.family_index ||
        witness.family != (triangle ? cin::WitnessFamily::ShellTriangle :
            cin::WitnessFamily::ShellQuad))
      continue;
    bool complete = true;
    for (unsigned n = 0; n < parent.arity; ++n)
      complete = complete && WitnessContains(witness, parent.nodes[n]);
    if (complete) return true;
  }
  return false;
}
SelfContactTiedStatus DirectionalTied(const Inventory& inventory,
    const SelfContactActiveUseForecast& forecast,
    const WeightedSurfacePoint& secondary,
    const SelfContactParentUse& master_parent,
    const WeightedSurfacePoint& master) noexcept {
  if (!forecast.cin_rows) return SelfContactTiedStatus::NotRelated;
  const auto rows = inventory.cin_rows; // Copied ranges authenticate retained model row order.
  bool any_local = false, complete = true;
  for (unsigned slot = 0; slot < secondary.count; ++slot) {
    if (secondary.weights[slot] == 0) continue;
    const auto node = secondary.nodes[slot];
    bool matched = false, local = false;
    for (std::size_t r = 0; r < forecast.cin_rows; ++r) {
      const auto& row = rows[r];
      if (row.secondary_domain_node != node ||
          row.master_source.element_id != master_parent.source.source_parent_id)
        continue;
      local = true;
      bool contains = true;
      for (unsigned m = 0; m < master.count; ++m)
        if (master.weights[m] != 0 && !RowContains(row, master.nodes[m]))
          contains = false;
      if (contains && WitnessAuthenticates(inventory, forecast, r, master_parent))
        matched = true;
    }
    any_local = any_local || local;
    complete = complete && matched;
  }
  if (!any_local) return SelfContactTiedStatus::NotRelated;
  return complete ? SelfContactTiedStatus::CompleteLocalSupportNeedsRuntimeActivity :
      SelfContactTiedStatus::PartialOrUnauthenticatedLocalSupportNotExcluded;
}
}

SelfContactTiedStatus TiedStatus(const Inventory& inventory,
    const SelfContactActiveUseForecast& forecast,
    const SelfContactParentUse& first_parent, const WeightedSurfacePoint& first,
    const SelfContactParentUse& second_parent, const WeightedSurfacePoint& second) noexcept {
  const auto a = DirectionalTied(inventory, forecast, first,
      second_parent, second);
  const auto b = DirectionalTied(inventory, forecast, second,
      first_parent, first);
  if (a == SelfContactTiedStatus::CompleteLocalSupportNeedsRuntimeActivity ||
      b == SelfContactTiedStatus::CompleteLocalSupportNeedsRuntimeActivity)
    return SelfContactTiedStatus::CompleteLocalSupportNeedsRuntimeActivity;
  if (a == SelfContactTiedStatus::PartialOrUnauthenticatedLocalSupportNotExcluded ||
      b == SelfContactTiedStatus::PartialOrUnauthenticatedLocalSupportNotExcluded)
    return SelfContactTiedStatus::PartialOrUnauthenticatedLocalSupportNotExcluded;
  return SelfContactTiedStatus::NotRelated;
}
} // namespace tlfea::contact::active_use

namespace tlfea::contact {
namespace {
using S = SelfContactActiveUseStatus;
SelfContactActiveUseReport Invalid(const char* message) noexcept {
  return {S::InvalidInput, SIZE_MAX, SIZE_MAX, message};
}
bool ActivityOutputDisjoint(const void* output, std::size_t bytes,
    SelfContactActivityView activity) noexcept {
  using tl::fea::trial_identity::Disjoint;
  return Disjoint(output, bytes, activity.base, activity.parent_count) &&
      Disjoint(output, bytes, activity.current, activity.parent_count);
}
SelfContactPairStatus CommonStatus(SelfContactPairClassification& next,
    bool edge_edge) noexcept {
  const auto unsupported = SelfContactSupportStatus::UnsupportedCinSecondary;
  if (!next.active[0] || !next.active[1]) return SelfContactPairStatus::InactiveParent;
  if (next.local_incidence) {
    next.excluded = true;
    return SelfContactPairStatus::ExcludedLocalIncidence;
  }
  if (next.parent[0] == next.parent[1])
    return SelfContactPairStatus::SameParentNeedsCurrentRegularity;
  if (next.endpoint_support[0].status == unsupported ||
      next.endpoint_support[1].status == unsupported)
    return SelfContactPairStatus::UnsupportedCinSecondary;
  if (next.endpoint_support[0].status == SelfContactSupportStatus::CompleteRigidGroup &&
      next.endpoint_support[1].status == SelfContactSupportStatus::CompleteRigidGroup &&
      next.endpoint_support[0].complete_rigid_group ==
          next.endpoint_support[1].complete_rigid_group) {
    next.excluded = true;
    return SelfContactPairStatus::ExcludedSameRigidGroup;
  }
  if (next.tied == SelfContactTiedStatus::CompleteLocalSupportNeedsRuntimeActivity)
    return SelfContactPairStatus::UnresolvedTiedSupportNotExcluded;
  if (!edge_edge) return SelfContactPairStatus::AdmittedVertexFace;
  return SelfContactPairStatus::UnadmittedEdgeEdgeForceArea;
}
}

SelfContactActiveUseReport SelfContactActiveUseBinding::ResolveVertexUse(
    std::size_t index, SelfContactActivityView activity,
    SelfContactResolvedVertexUse* output) const noexcept {
  if (!output || !OutputDisjoint(output, sizeof(*output)) ||
      !active_use::ValidateActivity(impl_ ? impl_->forecast : SelfContactActiveUseForecast{},
          activity) ||
      !ActivityOutputDisjoint(output, sizeof(*output), activity))
    return Invalid("Vertex-use activity/output is invalid or aliases source");
  if (index >= impl_->forecast.vertex_uses)
    return {S::InvalidInput, SIZE_MAX, index, "Vertex-use index is out of range"};
  const auto& use = impl_->inventory.vertex_uses[index];
  const auto& parent = impl_->inventory.parents[use.parent];
  SelfContactResolvedVertexUse next;
  next.use = use;
  next.active = activity.current[use.parent] != 0;
  if (next.active) {
    next.reference_half_thickness_m = parent.reference_half_thickness_m;
    next.reference_area_m2 = parent.reference_area_m2;
    next.dual_area_m2 = use.dual_area_m2;
    next.directed_vf_area_m2 = use.directed_vf_area_m2;
  }
  *output = next;
  return {};
}
SelfContactActiveUseReport SelfContactActiveUseBinding::ResolveEdgeUse(
    std::size_t index, SelfContactActivityView activity,
    SelfContactResolvedEdgeUse* output) const noexcept {
  if (!output || !OutputDisjoint(output, sizeof(*output)) ||
      !active_use::ValidateActivity(impl_ ? impl_->forecast : SelfContactActiveUseForecast{},
          activity) ||
      !ActivityOutputDisjoint(output, sizeof(*output), activity))
    return Invalid("Edge-use activity/output is invalid or aliases source");
  if (index >= impl_->forecast.edge_uses)
    return {S::InvalidInput, SIZE_MAX, index, "Edge-use index is out of range"};
  const auto& use = impl_->inventory.edge_uses[index];
  const auto& parent = impl_->inventory.parents[use.parent];
  SelfContactResolvedEdgeUse next;
  next.use = use;
  next.active = activity.current[use.parent] != 0;
  if (next.active) {
    next.reference_half_thickness_m = parent.reference_half_thickness_m;
    next.reference_area_m2 = parent.reference_area_m2;
  }
  *output = next;
  return {};
}

SelfContactActiveUseReport SelfContactActiveUseBinding::ClassifyVertexFace(
    std::size_t vertex_index, std::size_t facet_index,
    const WeightedSurfacePoint& face_point, SelfContactActivityView activity,
    SelfContactPairClassification* output) const noexcept {
  using tl::fea::trial_identity::Disjoint;
  if (!output || !OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), &face_point, sizeof(face_point)) ||
      !active_use::ValidateActivity(impl_ ? impl_->forecast : SelfContactActiveUseForecast{},
          activity) ||
      !ActivityOutputDisjoint(output, sizeof(*output), activity))
    return Invalid("VF inputs/output/activity are invalid or alias");
  if (vertex_index >= impl_->forecast.vertex_uses ||
      facet_index >= impl_->forecast.facets)
    return {S::InvalidInput, SIZE_MAX, SIZE_MAX, "VF feature-use index is out of range"};
  const auto& vertex = impl_->inventory.vertex_uses[vertex_index];
  const auto& facet = impl_->inventory.facets[facet_index];
  const auto& first_parent = impl_->inventory.parents[vertex.parent];
  const auto& second_parent = impl_->inventory.parents[facet.parent];
  if (!active_use::MapMatchesParent(second_parent, face_point) ||
      ValidateWeightedSurfacePoint(face_point, impl_->forecast.node_roles) != Status::kOk)
    return {S::InvalidInput, facet.parent, facet_index,
        "VF face map does not match its parent-local original slots"};
  SelfContactPairClassification next;
  next.binding_identity = impl_.get();
  next.kind = SelfContactPairKind::VertexFace;
  next.parent[0] = vertex.parent; next.parent[1] = facet.parent;
  next.feature[0] = vertex.feature;
  next.feature[1] = static_cast<std::uint32_t>(facet_index);
  next.active[0] = activity.current[vertex.parent] != 0;
  next.active[1] = activity.current[facet.parent] != 0;
  if (next.active[0]) next.reference_half_thickness_m[0] =
      first_parent.reference_half_thickness_m;
  if (next.active[1]) next.reference_half_thickness_m[1] =
      second_parent.reference_half_thickness_m;
  next.endpoint_support[0] = vertex.support;
  auto report = active_use::Classify(impl_->inventory, impl_->forecast, face_point,
      next.endpoint_support[1]);
  if (report.status != S::Ok) return report;
  for (const auto feature : facet.vertex_features)
    if (feature == vertex.feature) next.local_incidence = true;
  next.tied = active_use::TiedStatus(impl_->inventory, impl_->forecast,
      first_parent, vertex.point, second_parent, face_point);
  if (next.active[0] && next.active[1])
    next.candidate_directed_area_m2 = vertex.directed_vf_area_m2;
  next.status = CommonStatus(next, false);
  if (next.status == SelfContactPairStatus::AdmittedVertexFace)
    next.admitted_force_area_m2 = next.candidate_directed_area_m2;
  *output = next;
  return {};
}

SelfContactActiveUseReport SelfContactActiveUseBinding::ClassifyEdgeEdge(
    std::size_t first_index, const WeightedSurfacePoint& first_point,
    std::size_t second_index, const WeightedSurfacePoint& second_point,
    SelfContactEdgeEdgeCase edge_case, SelfContactActivityView activity,
    SelfContactPairClassification* output) const noexcept {
  using tl::fea::trial_identity::Disjoint;
  if (!output || !OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), &first_point, sizeof(first_point)) ||
      !Disjoint(output, sizeof(*output), &second_point, sizeof(second_point)) ||
      !active_use::ValidateActivity(impl_ ? impl_->forecast : SelfContactActiveUseForecast{},
          activity) ||
      !ActivityOutputDisjoint(output, sizeof(*output), activity))
    return Invalid("EE inputs/output/activity are invalid or alias");
  if (first_index >= impl_->forecast.edge_uses ||
      second_index >= impl_->forecast.edge_uses)
    return {S::InvalidInput, SIZE_MAX, SIZE_MAX, "EE feature-use index is out of range"};
  const auto& first = impl_->inventory.edge_uses[first_index];
  const auto& second = impl_->inventory.edge_uses[second_index];
  const auto& first_parent = impl_->inventory.parents[first.parent];
  const auto& second_parent = impl_->inventory.parents[second.parent];
  if (!active_use::MapMatchesParent(first_parent, first_point) ||
      !active_use::MapMatchesParent(second_parent, second_point) ||
      ValidateWeightedSurfacePoint(first_point, impl_->forecast.node_roles) != Status::kOk ||
      ValidateWeightedSurfacePoint(second_point, impl_->forecast.node_roles) != Status::kOk)
    return {S::InvalidInput, SIZE_MAX, SIZE_MAX,
        "EE point map does not match its parent-local original slots"};
  SelfContactPairClassification next;
  next.binding_identity = impl_.get();
  next.kind = SelfContactPairKind::EdgeEdge;
  next.edge_edge_case = edge_case;
  next.parent[0] = first.parent; next.parent[1] = second.parent;
  next.feature[0] = first.feature; next.feature[1] = second.feature;
  next.active[0] = activity.current[first.parent] != 0;
  next.active[1] = activity.current[second.parent] != 0;
  if (next.active[0]) next.reference_half_thickness_m[0] =
      first_parent.reference_half_thickness_m;
  if (next.active[1]) next.reference_half_thickness_m[1] =
      second_parent.reference_half_thickness_m;
  auto report = active_use::Classify(impl_->inventory, impl_->forecast,
      first_point, next.endpoint_support[0]);
  if (report.status != S::Ok) return report;
  report = active_use::Classify(impl_->inventory, impl_->forecast,
      second_point, next.endpoint_support[1]);
  if (report.status != S::Ok) return report;
  const auto& a = impl_->inventory.edges[first.feature];
  const auto& b = impl_->inventory.edges[second.feature];
  for (const auto av : a.vertices) for (const auto bv : b.vertices)
    if (av == bv) next.local_incidence = true;
  next.tied = active_use::TiedStatus(impl_->inventory, impl_->forecast,
      first_parent, first_point, second_parent, second_point);
  next.status = CommonStatus(next, true);
  *output = next;
  return {};
}
} // namespace tlfea::contact
