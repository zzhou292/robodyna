// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../Q4ContactBounds.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <cmath>
#include <cstring>

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
bool HasCinSecondary(const Inventory& inventory,
    const SelfContactActiveUseForecast& forecast,
    const WeightedSurfacePoint& point) noexcept {
  for (unsigned slot = 0; slot < point.count; ++slot)
    if (point.weights[slot] != 0 &&
        point.nodes[slot] < forecast.node_roles &&
        inventory.node_roles[point.nodes[slot]].cin_secondary)
      return true;
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
    const auto indexed = inventory.cin_node_rows[node];
    for (std::size_t i = indexed.offset;
         i < std::size_t(indexed.offset)+indexed.count; ++i) {
      const auto r = inventory.cin_row_indices[i];
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
  const bool first_secondary =
      HasCinSecondary(inventory, forecast, first);
  const bool second_secondary =
      HasCinSecondary(inventory, forecast, second);
  if (!first_secondary && !second_secondary)
    return SelfContactTiedStatus::NotRelated;
  const auto a = first_secondary
      ? DirectionalTied(inventory, forecast, first,
            second_parent, second)
      : SelfContactTiedStatus::NotRelated;
  const auto b = second_secondary
      ? DirectionalTied(inventory, forecast, second,
            first_parent, first)
      : SelfContactTiedStatus::NotRelated;
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
bool SameBits(double a, double b) noexcept {
  std::uint64_t aa = 0, bb = 0;
  std::memcpy(&aa, &a, sizeof(aa));
  std::memcpy(&bb, &b, sizeof(bb));
  return aa == bb;
}
bool SamePoint(const WeightedSurfacePoint& a,
    const WeightedSurfacePoint& b) noexcept {
  if (a.count != b.count) return false;
  for (unsigned slot = 0; slot < 4; ++slot)
    if (a.nodes[slot] != b.nodes[slot] ||
        !SameBits(a.weights[slot], b.weights[slot]))
      return false;
  return true;
}
bool EdgeParameter(const SelfContactFacetEdgeUse& edge,
    const WeightedSurfacePoint& point, double* output) noexcept {
  if (!output || point.count != edge.endpoints[0].count ||
      point.count != edge.endpoints[1].count)
    return false;
  for (unsigned slot = 0; slot < point.count; ++slot)
    if (point.nodes[slot] != edge.endpoints[0].nodes[slot] ||
        point.nodes[slot] != edge.endpoints[1].nodes[slot])
      return false;
  if (SamePoint(point, edge.endpoints[0])) {
    *output = 0;
    return true;
  }
  if (SamePoint(point, edge.endpoints[1])) {
    *output = 1;
    return true;
  }
  for (unsigned pivot = 0; pivot < point.count; ++pivot) {
    const double denominator =
        edge.endpoints[1].weights[pivot] -
        edge.endpoints[0].weights[pivot];
    if (denominator == 0) continue;
    const double parameter =
        (point.weights[pivot] - edge.endpoints[0].weights[pivot]) /
        denominator;
    if (!std::isfinite(parameter) || parameter < 0 || parameter > 1)
      continue;
    Q4IntegralInterval complement;
    if (!q4_bounds::Difference(1, parameter, &complement))
      continue;
    bool authenticated = true;
    for (unsigned slot = 0; slot < point.count; ++slot) {
      Q4IntegralInterval first, second, represented;
      if (!q4_bounds::MultiplyPositive(
              {edge.endpoints[0].weights[slot],
               edge.endpoints[0].weights[slot]},
              complement, &first) ||
          !q4_bounds::MultiplyPositive(
              {edge.endpoints[1].weights[slot],
               edge.endpoints[1].weights[slot]},
              {parameter, parameter}, &second) ||
          !q4_bounds::Add(first, second, &represented) ||
          point.weights[slot] < represented.lower ||
          point.weights[slot] > represented.upper) {
        authenticated = false;
        break;
      }
    }
    if (authenticated) {
      *output = parameter;
      return true;
    }
  }
  return false;
}
bool ValidArea(Q4CertifiedIntegral area) noexcept {
  Q4CertifiedIntegral checked;
  return std::isfinite(area.error) && area.error >= 0 &&
      area.value > 0 && area.lower > 0 &&
      q4_bounds::Certify(
          area.value, {area.lower, area.upper}, &checked) &&
      checked.error <= area.error;
}
bool InterpolateArea(const SelfContactFacetEdgeUse& edge,
    double parameter, Q4CertifiedIntegral* output) noexcept {
  if (!output || !ValidArea(edge.directed_endpoint_dual_area_m2[0]) ||
      !ValidArea(edge.directed_endpoint_dual_area_m2[1]) ||
      !std::isfinite(parameter) || parameter < 0 || parameter > 1)
    return false;
  if (parameter == 0) {
    *output = edge.directed_endpoint_dual_area_m2[0];
    return true;
  }
  if (parameter == 1) {
    *output = edge.directed_endpoint_dual_area_m2[1];
    return true;
  }
  const double complement = 1 - parameter;
  Q4IntegralInterval complement_interval, first, second, sum;
  const double value =
      complement * edge.directed_endpoint_dual_area_m2[0].value +
      parameter * edge.directed_endpoint_dual_area_m2[1].value;
  return q4_bounds::Difference(1, parameter, &complement_interval) &&
      q4_bounds::MultiplyPositive(
             {edge.directed_endpoint_dual_area_m2[0].lower,
              edge.directed_endpoint_dual_area_m2[0].upper},
             complement_interval, &first) &&
      q4_bounds::MultiplyPositive(
          {edge.directed_endpoint_dual_area_m2[1].lower,
           edge.directed_endpoint_dual_area_m2[1].upper},
          {parameter, parameter}, &second) &&
      q4_bounds::Add(first, second, &sum) &&
      q4_bounds::Certify(value, sum, output) &&
      output->lower > 0;
}
bool AddArea(Q4CertifiedIntegral a, Q4CertifiedIntegral b,
    Q4CertifiedIntegral* output) noexcept {
  Q4IntegralInterval sum;
  return ValidArea(a) && ValidArea(b) &&
      q4_bounds::Add({a.lower, a.upper}, {b.lower, b.upper}, &sum) &&
      q4_bounds::Certify(a.value + b.value, sum, output) &&
      output->lower > 0;
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
    next.directed_endpoint_dual_area_m2[0] =
        use.directed_endpoint_dual_area_m2[0];
    next.directed_endpoint_dual_area_m2[1] =
        use.directed_endpoint_dual_area_m2[1];
  }
  *output = next;
  return {};
}

bool self_contact_transaction::ActiveUseQueryAccess::ValidateActivity(
    const SelfContactActiveUseBinding& binding,
    SelfContactActivityView activity) noexcept {
  return active_use::ValidateActivity(
      binding.impl_ ? binding.impl_->forecast :
          SelfContactActiveUseForecast{},
      activity);
}

SelfContactActiveUseReport
self_contact_transaction::ActiveUseQueryAccess::ClassifyVertexFace(
    const SelfContactActiveUseBinding& binding,
    std::size_t vertex_index, std::size_t facet_index,
    const WeightedSurfacePoint& face_point, SelfContactActivityView activity,
    SelfContactPairClassification* output) noexcept {
  if (!binding.impl_ || !output)
    return Invalid("VF inputs/output/activity are invalid or alias");
  if (vertex_index >= binding.impl_->forecast.vertex_uses ||
      facet_index >= binding.impl_->forecast.facets)
    return {S::InvalidInput, SIZE_MAX, SIZE_MAX, "VF feature-use index is out of range"};
  const auto& vertex =
      binding.impl_->inventory.vertex_uses[vertex_index];
  const auto& facet =
      binding.impl_->inventory.facets[facet_index];
  const auto& first_parent =
      binding.impl_->inventory.parents[vertex.parent];
  const auto& second_parent =
      binding.impl_->inventory.parents[facet.parent];
  if (!active_use::MapMatchesParent(second_parent, face_point) ||
      ValidateWeightedSurfacePoint(
          face_point, binding.impl_->forecast.node_roles) !=
          Status::kOk)
    return {S::InvalidInput, facet.parent, facet_index,
        "VF face map does not match its parent-local original slots"};
  SelfContactPairClassification next;
  next.binding_identity = binding.impl_.get();
  next.activity_base_identity = activity.base;
  next.activity_current_identity = activity.current;
  next.activity_parent_count = activity.parent_count;
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
  auto report = active_use::Classify(
      binding.impl_->inventory, binding.impl_->forecast,
      face_point, next.endpoint_support[1]);
  if (report.status != S::Ok) return report;
  for (const auto feature : facet.vertex_features)
    if (feature == vertex.feature) next.local_incidence = true;
  next.tied = active_use::TiedStatus(
      binding.impl_->inventory, binding.impl_->forecast,
      first_parent, vertex.point, second_parent, face_point);
  if (next.active[0] && next.active[1])
    next.candidate_directed_area_m2 = vertex.directed_vf_area_m2;
  next.status = CommonStatus(next, false);
  if (next.status == SelfContactPairStatus::AdmittedVertexFace)
    next.admitted_force_area_m2 = next.candidate_directed_area_m2;
  *output = next;
  return {};
}

SelfContactActiveUseReport SelfContactActiveUseBinding::ClassifyVertexFace(
    std::size_t vertex_index, std::size_t facet_index,
    const WeightedSurfacePoint& face_point,
    SelfContactActivityView activity,
    SelfContactPairClassification* output) const noexcept {
  using tl::fea::trial_identity::Disjoint;
  if (!output || !OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output),
          &face_point, sizeof(face_point)) ||
      !self_contact_transaction::ActiveUseQueryAccess::
          ValidateActivity(*this, activity) ||
      !ActivityOutputDisjoint(
          output, sizeof(*output), activity))
    return Invalid("VF inputs/output/activity are invalid or alias");
  return self_contact_transaction::ActiveUseQueryAccess::
      ClassifyVertexFace(*this, vertex_index, facet_index,
          face_point, activity, output);
}

SelfContactActiveUseReport
self_contact_transaction::ActiveUseQueryAccess::ClassifyEdgeEdge(
    const SelfContactActiveUseBinding& binding,
    std::size_t first_index, const WeightedSurfacePoint& first_point,
    std::size_t second_index, const WeightedSurfacePoint& second_point,
    SelfContactEdgeEdgeCase edge_case, SelfContactActivityView activity,
    SelfContactPairClassification* output) noexcept {
  if (!binding.impl_ || !output)
    return Invalid("EE inputs/output/activity are invalid or alias");
  if (first_index >= binding.impl_->forecast.edge_uses ||
      second_index >= binding.impl_->forecast.edge_uses)
    return {S::InvalidInput, SIZE_MAX, SIZE_MAX, "EE feature-use index is out of range"};
  const auto& first =
      binding.impl_->inventory.edge_uses[first_index];
  const auto& second =
      binding.impl_->inventory.edge_uses[second_index];
  const auto& first_parent =
      binding.impl_->inventory.parents[first.parent];
  const auto& second_parent =
      binding.impl_->inventory.parents[second.parent];
  double edge_parameter[2]{};
  if (!active_use::MapMatchesParent(first_parent, first_point) ||
      !active_use::MapMatchesParent(second_parent, second_point) ||
      ValidateWeightedSurfacePoint(
          first_point, binding.impl_->forecast.node_roles) !=
          Status::kOk ||
      ValidateWeightedSurfacePoint(
          second_point, binding.impl_->forecast.node_roles) !=
          Status::kOk ||
      !EdgeParameter(first, first_point, edge_parameter) ||
      !EdgeParameter(second, second_point, edge_parameter + 1))
    return {S::InvalidInput, SIZE_MAX, SIZE_MAX,
        "EE point is not an exact authenticated point on its edge use"};
  SelfContactPairClassification next;
  next.binding_identity = binding.impl_.get();
  next.activity_base_identity = activity.base;
  next.activity_current_identity = activity.current;
  next.activity_parent_count = activity.parent_count;
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
  auto report = SelfContactActiveUseReport{};
  if (SamePoint(first_point, first.endpoints[0]))
    next.endpoint_support[0] = first.endpoint_support[0];
  else if (SamePoint(first_point, first.endpoints[1]))
    next.endpoint_support[0] = first.endpoint_support[1];
  else
    report = active_use::Classify(
        binding.impl_->inventory, binding.impl_->forecast,
        first_point, next.endpoint_support[0]);
  if (report.status != S::Ok) return report;
  if (SamePoint(second_point, second.endpoints[0]))
    next.endpoint_support[1] = second.endpoint_support[0];
  else if (SamePoint(second_point, second.endpoints[1]))
    next.endpoint_support[1] = second.endpoint_support[1];
  else
    report = active_use::Classify(
        binding.impl_->inventory, binding.impl_->forecast,
        second_point, next.endpoint_support[1]);
  if (report.status != S::Ok) return report;
  const auto& a = binding.impl_->inventory.edges[first.feature];
  const auto& b = binding.impl_->inventory.edges[second.feature];
  for (const auto av : a.vertices) for (const auto bv : b.vertices)
    if (av == bv) next.local_incidence = true;
  next.tied = active_use::TiedStatus(
      binding.impl_->inventory, binding.impl_->forecast,
      first_parent, first_point, second_parent, second_point);
  next.status = CommonStatus(next, true);
  const bool area_case =
      edge_case == SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum ||
      edge_case == SelfContactEdgeEdgeCase::BoundaryVertexEdgeMinimum ||
      edge_case == SelfContactEdgeEdgeCase::ZeroDistance;
  if (next.status == SelfContactPairStatus::UnadmittedEdgeEdgeForceArea &&
      area_case) {
    Q4CertifiedIntegral edge_point_area[2];
    if (!InterpolateArea(first, edge_parameter[0], edge_point_area) ||
        !InterpolateArea(second, edge_parameter[1], edge_point_area + 1) ||
        !AddArea(edge_point_area[0], edge_point_area[1],
                 &next.candidate_directed_area_m2))
      return {S::Unrepresentable, SIZE_MAX, SIZE_MAX,
          "Symmetric directed edge-point area is unrepresentable"};
    next.admitted_force_area_m2 = next.candidate_directed_area_m2;
    next.status = SelfContactPairStatus::AdmittedEdgeEdge;
  }
  *output = next;
  return {};
}

SelfContactActiveUseReport SelfContactActiveUseBinding::ClassifyEdgeEdge(
    std::size_t first_index,
    const WeightedSurfacePoint& first_point,
    std::size_t second_index,
    const WeightedSurfacePoint& second_point,
    SelfContactEdgeEdgeCase edge_case,
    SelfContactActivityView activity,
    SelfContactPairClassification* output) const noexcept {
  using tl::fea::trial_identity::Disjoint;
  if (!output || !OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output),
          &first_point, sizeof(first_point)) ||
      !Disjoint(output, sizeof(*output),
          &second_point, sizeof(second_point)) ||
      !self_contact_transaction::ActiveUseQueryAccess::
          ValidateActivity(*this, activity) ||
      !ActivityOutputDisjoint(
          output, sizeof(*output), activity))
    return Invalid("EE inputs/output/activity are invalid or alias");
  return self_contact_transaction::ActiveUseQueryAccess::
      ClassifyEdgeEdge(*this, first_index, first_point,
          second_index, second_point, edge_case, activity,
          output);
}
} // namespace tlfea::contact
