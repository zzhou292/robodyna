// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "FinalizedCoverageLedger.h"
#include "CandidateExclusions.h"
#include "CrossingBatch.h"
#include "TranslatedLocal.h"
#include "SortedIntersections.h"
#include "QualificationRanges.h"

#include "lib_src/solvers/NodalTrialIdentity.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tlfea::contact {
namespace {

using S = SelfContactTransactionStatus;
namespace fe = tl::fea;
namespace sct = self_contact_transaction;

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

RepresentedTrianglePathKey PathKey(
    const FixedTriangleKey& key) noexcept {
  return {key.source_instance_id, key.parent_eid,
          key.level, key.local_facet};
}

RepresentedIntervalPairKey PairKey(
    RepresentedTrianglePathKey a,
    RepresentedTrianglePathKey b) noexcept {
  if (sct::Compare(b, a) < 0) std::swap(a, b);
  return {{a, b}};
}

SelfContactTransactionReport ValidatePreparedIntersections(
    FixedTriangleIntersectionView intersections,
    const RepresentedIntervalPairKey* pairs,
    std::size_t pair_count) noexcept {
  if (!intersections.complete ||
      (intersections.count && !intersections.data) ||
      (pair_count && !pairs))
    return Failure(S::DiscoveryFailure,
        "Prepared intersection publication is incomplete");
  for (std::size_t intersection = 0;
       intersection < intersections.count; ++intersection) {
    const auto& value = intersections.data[intersection];
    if (!RequiresIntersectionAdmission(value)) continue;
    if (!pair_count)
      return Failure(S::IdentityMismatch,
          "Prepared nonlocal intersection has no candidate pair");
    const auto key = PairKey(
        PathKey(value.triangles[0]), PathKey(value.triangles[1]));
    const auto* found = std::lower_bound(
        pairs, pairs + pair_count, key,
        [](const auto& first, const auto& second) {
          return sct::Compare(first, second) < 0;
        });
    if (found == pairs + pair_count || sct::Compare(*found, key) != 0)
      return Failure(S::IdentityMismatch,
          "Prepared nonlocal intersection belongs to a foreign candidate pair");
    // An exact endpoint intersection already disproves the candidate. Do not
    // replace this witness with an inconclusive whole-interval proof result.
    return Failure(S::CandidateRejected,
        "Nonlocal current triangle intersection is rejected", SIZE_MAX,
        static_cast<std::size_t>(found - pairs));
  }
  return {};
}

void MakePath(const CurrentFixedTriangle& base,
              const CurrentFixedTriangle& current,
              const sct::MotionSupport& motion,
              RepresentedTrianglePath* output) noexcept {
  RepresentedTrianglePath next;
  next.key = PathKey(current.key);
  next.motion =
      motion.certified_affine
      ? RepresentedMotion::LinearNodalV1
      : RepresentedMotion::RigidArc;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    next.vertices[vertex].key = current.vertex_keys[vertex];
    next.vertices[vertex].endpoint[0] = base.vertices[vertex];
    next.vertices[vertex].endpoint[1] = current.vertices[vertex];
    next.edge_keys[vertex] = current.edge_keys[vertex];
  }
  *output = next;
}

double Down(double value) noexcept {
  return std::nextafter(
      value, -std::numeric_limits<double>::infinity());
}

double Up(double value) noexcept {
  return std::nextafter(
      value, std::numeric_limits<double>::infinity());
}

double Component(Vec3 value, unsigned component) noexcept {
  return component == 0 ? value.x :
      (component == 1 ? value.y : value.z);
}

double Component(tl::math::Vec3 value,
                 unsigned component) noexcept {
  return component == 0 ? value.x :
      (component == 1 ? value.y : value.z);
}

void SetComponent(Vec3* value, unsigned component,
                  double next) noexcept {
  if (component == 0) value->x = next;
  else if (component == 1) value->y = next;
  else value->z = next;
}

bool NodeSweepBounds(
    std::uint32_t node, VectorView base, VectorView current,
    const std::uint32_t* node_rigid_groups,
    const fe::NodalRigidGroupSnapshot* accepted_groups,
    const fe::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count,
    fe::NodalRigidMemberTrajectory rigid_trajectory,
    double duration,
    double kick_duration,
    SelfContactSweptParentBounds* output) noexcept {
  if (!output || !node_rigid_groups ||
      node >= base.node_count || node >= current.node_count)
    return false;
  const auto first = base.at(node);
  const auto second = current.at(node);
  const auto group = node_rigid_groups[node];
  SelfContactSweptParentBounds next;
  if (group == UINT32_MAX) {
    for (unsigned component = 0; component < 3; ++component) {
      const auto a = Component(first, component);
      const auto b = Component(second, component);
      SetComponent(&next.lower, component, std::min(a, b));
      SetComponent(&next.upper, component, std::max(a, b));
    }
    *output = next;
    return IsFinite(next.lower) && IsFinite(next.upper);
  }
  if (group >= group_count || !accepted_groups ||
      !prepared_groups || !(duration > 0) ||
      !(kick_duration > 0) ||
      !std::isfinite(duration) ||
      !std::isfinite(kick_duration) ||
      duration > 2 * kick_duration)
    return false;
  return sct::BuildRigidMemberSweepBounds(
      first, second, accepted_groups[group],
      prepared_groups[group], rigid_trajectory,
      duration, output) ==
      sct::RigidMemberSweepStatus::Ok;
}

bool PointSweepBounds(
    const WeightedSurfacePoint& point, VectorView base,
    VectorView current, const std::uint32_t* node_rigid_groups,
    const fe::NodalRigidGroupSnapshot* accepted_groups,
    const fe::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count,
    fe::NodalRigidMemberTrajectory rigid_trajectory,
    double duration,
    double kick_duration,
    SelfContactSweptParentBounds* output) noexcept {
  if (!output ||
      ValidateWeightedSurfacePoint(point, base.node_count) != Status::kOk)
    return false;
  SelfContactSweptParentBounds next;
  for (unsigned component = 0; component < 3; ++component) {
    double lower = 0, upper = 0;
    for (unsigned slot = 0; slot < point.count; ++slot) {
      if (point.weights[slot] == 0) continue;
      SelfContactSweptParentBounds node;
      if (!NodeSweepBounds(
              point.nodes[slot], base, current,
              node_rigid_groups, accepted_groups, prepared_groups,
              group_count, rigid_trajectory,
              duration, kick_duration, &node))
        return false;
      const double weight = point.weights[slot];
      lower = Down(lower + Down(
          weight * Component(node.lower, component)));
      upper = Up(upper + Up(
          weight * Component(node.upper, component)));
    }
    SetComponent(&next.lower, component, lower);
    SetComponent(&next.upper, component, upper);
  }
  *output = next;
  return IsFinite(next.lower) && IsFinite(next.upper);
}

SelfContactTransactionReport BuildSweptBounds(
    const SelfContactActiveUseBinding& active_use,
    const FixedContactFacet* descriptors, std::size_t facets,
    const sct::MotionSupport* parent_motion,
    sct::MotionSupport* facet_motion,
    sct::FacetQuadraticCoefficients* facet_quadratic,
    const std::uint32_t* node_rigid_groups,
    const fe::NodalRigidGroupSnapshot* accepted_groups,
    const fe::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count, VectorView base, VectorView current,
    fe::NodalRigidMemberTrajectory rigid_trajectory,
    double duration, double kick_duration,
    SelfContactSweptParentBounds* facet_bounds,
    SelfContactSweptParentBounds* parent_bounds,
    std::size_t surface_parents) noexcept {
  if (!descriptors || !facets || !parent_motion || !facet_motion ||
      !facet_quadratic ||
      !node_rigid_groups ||
      !facet_bounds || !parent_bounds || !surface_parents)
    return Failure(S::InvalidInput,
        "Rigid swept-bound storage is incomplete");
  const double infinity = std::numeric_limits<double>::infinity();
  for (std::size_t parent = 0; parent < surface_parents; ++parent)
    parent_bounds[parent] = {{infinity, infinity, infinity},
                             {-infinity, -infinity, -infinity}};
  for (std::size_t facet = 0; facet < facets; ++facet) {
    const auto& use = active_use.facet_uses()[facet];
    if (use.parent >= active_use.parents().size())
      return Failure(S::IdentityMismatch,
          "Swept facet has no active-use parent", facet);
    SelfContactSweptParentBounds next{
        {infinity, infinity, infinity},
        {-infinity, -infinity, -infinity}};
    bool certified_affine = false;
    const auto affine_status =
        sct::BuildRigidFacetQuadraticCoefficients(
        descriptors[facet], base, current, node_rigid_groups,
        accepted_groups, prepared_groups, group_count,
        rigid_trajectory, duration, facet_quadratic + facet,
        &certified_affine);
    facet_motion[facet].certified_affine =
        affine_status == sct::RigidMemberSweepStatus::Ok &&
        certified_affine;
    if (facet_motion[facet].motion ==
            SelfContactFacetMotion::LinearNodalV1 &&
        !facet_motion[facet].certified_affine)
      return Failure(S::IdentityMismatch,
          "Ordinary facet failed affine motion authentication", facet);
    bool bounded = true;
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      SelfContactSweptParentBounds point;
      if (!PointSweepBounds(
              descriptors[facet].vertices[vertex],
              base, current, node_rigid_groups,
              accepted_groups, prepared_groups, group_count,
              rigid_trajectory, duration, kick_duration, &point)) {
        bounded = false;
        break;
      }
      for (unsigned component = 0; component < 3; ++component) {
        SetComponent(&next.lower, component, std::min(
            Component(next.lower, component),
            Component(point.lower, component)));
        SetComponent(&next.upper, component, std::max(
            Component(next.upper, component),
            Component(point.upper, component)));
      }
    }
    const double thickness =
        active_use.parents()[use.parent].reference_half_thickness_m;
    if (!std::isfinite(thickness) || !(thickness > 0))
      return Failure(S::IdentityMismatch,
          "Swept facet thickness is invalid", facet);
    if (bounded) {
      for (unsigned component = 0; component < 3; ++component) {
        SetComponent(&next.lower, component,
            Down(Component(next.lower, component) - thickness));
        SetComponent(&next.upper, component,
            Up(Component(next.upper, component) + thickness));
      }
    } else {
      const double maximum = std::numeric_limits<double>::max();
      next = {{-maximum, -maximum, -maximum},
              {maximum, maximum, maximum}};
    }
    if (!IsFinite(next.lower) || !IsFinite(next.upper))
      return Failure(S::UnsupportedMotion,
          "Outward rigid swept facet bound overflowed", facet);
    facet_bounds[facet] = next;
    const auto surface = active_use.parents()[use.parent].surface_parent;
    if (surface >= surface_parents)
      return Failure(S::IdentityMismatch,
          "Swept facet surface-parent identity is invalid", facet);
    for (unsigned component = 0; component < 3; ++component) {
      SetComponent(&parent_bounds[surface].lower, component, std::min(
          Component(parent_bounds[surface].lower, component),
          Component(next.lower, component)));
      SetComponent(&parent_bounds[surface].upper, component, std::max(
          Component(parent_bounds[surface].upper, component),
          Component(next.upper, component)));
    }
  }
  // Preserve the existing ordinary LinearNodalV1 broadphase box exactly:
  // endpoint corner union followed by one directed thickness inflation.
  for (std::size_t parent = 0;
       parent < active_use.parents().size(); ++parent) {
    if (parent_motion[parent].motion !=
        SelfContactFacetMotion::LinearNodalV1)
      continue;
    const auto& value = active_use.parents()[parent];
    if ((value.arity != 3 && value.arity != 4) ||
        value.surface_parent >= surface_parents)
      return Failure(S::IdentityMismatch,
          "Linear swept parent identity is invalid", parent);
    SelfContactSweptParentBounds next{
        {infinity, infinity, infinity},
        {-infinity, -infinity, -infinity}};
    for (unsigned slot = 0; slot < value.arity; ++slot) {
      if (value.nodes[slot] >= base.node_count)
        return Failure(S::IdentityMismatch,
            "Linear swept parent node is invalid", parent);
      const auto first = base.at(value.nodes[slot]);
      const auto second = current.at(value.nodes[slot]);
      for (unsigned component = 0; component < 3; ++component) {
        SetComponent(&next.lower, component, std::min(
            Component(next.lower, component),
            std::min(Component(first, component),
                     Component(second, component))));
        SetComponent(&next.upper, component, std::max(
            Component(next.upper, component),
            std::max(Component(first, component),
                     Component(second, component))));
      }
    }
    for (unsigned component = 0; component < 3; ++component) {
      SetComponent(&next.lower, component,
          Down(Component(next.lower, component) -
               value.reference_half_thickness_m));
      SetComponent(&next.upper, component,
          Up(Component(next.upper, component) +
             value.reference_half_thickness_m));
    }
    parent_bounds[value.surface_parent] = next;
  }
  for (std::size_t parent = 0; parent < surface_parents; ++parent)
    if (!IsFinite(parent_bounds[parent].lower) ||
        !IsFinite(parent_bounds[parent].upper))
      return Failure(S::IdentityMismatch,
          "Complete swept parent roster has an empty bound", parent);
  return {};
}

bool SameRigidSnapshotIdentity(
    const fe::RigidBindingGroup& binding,
    const fe::NodalRigidGroupSnapshot& snapshot) noexcept {
  return binding.source_kind == snapshot.source_kind &&
      binding.source_id == snapshot.source_group_id &&
      binding.source_node_set_id == snapshot.source_node_set_id;
}

void DescribeMotionFailure(
    const SelfContactActiveUseBinding& active_use,
    const CurrentFixedTriangle* triangles,
    const sct::MotionSupport* motion,
    const sct::FacetQuadraticCoefficients* coefficients,
    const SelfContactSweptParentBounds* swept_bounds,
    FixedTrianglePair pair,
    SelfContactTransactionReport* report) noexcept {
  const auto* rigid = active_use.rigid();
  const std::uint32_t facets[2]{pair.first, pair.second};
  for (unsigned side = 0; side < 2; ++side) {
    const auto facet = facets[side];
    auto& output = report->offending_motion[side];
    output.facet = triangles[facet].key;
    output.active_parent = motion[facet].parent;
    output.motion = motion[facet].motion;
    if (coefficients && coefficients[facet].complete)
      for (unsigned vertex = 0; vertex < 3; ++vertex)
        for (unsigned component = 0; component < 3; ++component) {
          SetComponent(
              &report->offending_quadratic_lower[side][vertex],
              component,
              coefficients[facet].q[vertex][component].lower);
          SetComponent(
              &report->offending_quadratic_upper[side][vertex],
              component,
              coefficients[facet].q[vertex][component].upper);
        }
    if (swept_bounds)
      report->offending_swept_bounds[side] = swept_bounds[facet];
    if (motion[facet].parent < active_use.parents().size())
      report->offending_half_thickness_m[side] =
          active_use.parents()[motion[facet].parent].
              reference_half_thickness_m;
    output.rigid_group_count = motion[facet].rigid_group_count;
    for (unsigned group = 0;
         group < motion[facet].rigid_group_count; ++group) {
      const auto index = motion[facet].rigid_groups[group];
      auto& identity = output.rigid_groups[group];
      identity.binding_group = index;
      if (rigid && index < rigid->groups().size()) {
        const auto& value = rigid->groups()[index];
        identity.source_kind = value.source_kind;
        identity.source_group_id = value.source_id;
        identity.source_node_set_id = value.source_node_set_id;
      }
    }
  }
}

bool PairPresent(const RepresentedIntervalPairKey* pairs,
                 std::size_t count,
                 const RepresentedIntervalPairKey& key) noexcept {
  std::size_t lower = 0, upper = count;
  while (lower < upper) {
    const auto middle = lower + (upper - lower) / 2;
    if (sct::Compare(pairs[middle], key) < 0)
      lower = middle + 1;
    else
      upper = middle;
  }
  return lower < count && sct::Compare(pairs[lower], key) == 0;
}

sct::LinearResidualSeparationResult
ResidualLinearCertificate(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    double first_half_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    double second_half_thickness,
    FixedTriangleFeatureTaskMask mask,
    FixedTriangleFeatureView features,
    FixedTriangleIntersectionView intersections) noexcept {
  auto result = sct::CertifyLinearResidualSeparation(
      first_base, first_prepared, first_half_thickness,
      second_base, second_prepared, second_half_thickness,
      features, intersections);
  if (result.status !=
          sct::LinearResidualSeparationStatus::
              IncompleteFeatureRoster ||
      mask.local_tasks)
    return result;

  // Publication deduplication may retain an identical canonical feature from
  // another pair. Re-evaluate this exact nonlocal pair into bounded stack
  // storage only when that prevents a complete 15-task distance proof.
  FixedTriangleFeatureCandidate local_features[15];
  fixed_triangle_features::PairFeatureResult feature_result;
  if (fixed_triangle_features::EvaluatePairFeaturesOnce(
          first_prepared, second_prepared,
          local_features, 15, &feature_result) !=
          FixedTriangleDiscoveryStatus::Ok ||
      feature_result.feature_count != 15)
    return result;
  FixedTriangleIntersection local_intersection;
  bool intersects = false;
  if (fixed_triangle_features::ClassifyPairIntersection(
          first_prepared, second_prepared,
          &local_intersection, &intersects) !=
      FixedTriangleDiscoveryStatus::Ok)
    return result;
  return sct::CertifyLinearResidualSeparation(
      first_base, first_prepared, first_half_thickness,
      second_base, second_prepared, second_half_thickness,
      {local_features, 15, true},
      {intersects ? &local_intersection : nullptr,
       intersects ? 1u : 0u, true});
}

sct::PersistentLinearContactResult
PersistentLinearCertificate(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    double first_half_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    double second_half_thickness,
    FixedTriangleFeatureTaskMask mask,
    FixedTriangleFeatureView features,
    const sct::AcceptedEventCertificate* accepted,
    std::size_t accepted_count) noexcept {
  auto result = sct::CertifyPersistentLinearContact(
      first_base, first_prepared, first_half_thickness,
      second_base, second_prepared, second_half_thickness,
      features, accepted, accepted_count);
  if (result.status ==
          sct::PersistentLinearContactStatus::CertifiedContact ||
      mask.local_tasks)
    return result;

  FixedTriangleFeatureCandidate local_features[15];
  fixed_triangle_features::PairFeatureResult feature_result;
  if (fixed_triangle_features::EvaluatePairFeaturesOnce(
          first_prepared, second_prepared,
          local_features, 15, &feature_result) !=
          FixedTriangleDiscoveryStatus::Ok ||
      feature_result.feature_count != 15)
    return result;
  return sct::CertifyPersistentLinearContact(
      first_base, first_prepared, first_half_thickness,
      second_base, second_prepared, second_half_thickness,
      {local_features, 15, true}, accepted, accepted_count);
}

sct::LinearResidualSeparationResult
QuadraticResidualCertificate(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    const sct::FacetQuadraticCoefficients& first_quadratic,
    double first_half_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    const sct::FacetQuadraticCoefficients& second_quadratic,
    double second_half_thickness, double duration,
    FixedTriangleFeatureTaskMask mask,
    FixedTriangleFeatureView features,
    FixedTriangleIntersectionView intersections) noexcept {
  auto result = sct::CertifyQuadraticResidualSeparation(
      first_base, first_prepared, first_quadratic,
      first_half_thickness,
      second_base, second_prepared, second_quadratic,
      second_half_thickness, duration, features, intersections);
  if (result.status !=
          sct::LinearResidualSeparationStatus::
              IncompleteFeatureRoster ||
      mask.local_tasks)
    return result;
  FixedTriangleFeatureCandidate local_features[15];
  fixed_triangle_features::PairFeatureResult feature_result;
  if (fixed_triangle_features::EvaluatePairFeaturesOnce(
          first_prepared, second_prepared,
          local_features, 15, &feature_result) !=
          FixedTriangleDiscoveryStatus::Ok ||
      feature_result.feature_count != 15)
    return result;
  FixedTriangleIntersection local_intersection;
  bool intersects = false;
  if (fixed_triangle_features::ClassifyPairIntersection(
          first_prepared, second_prepared,
          &local_intersection, &intersects) !=
      FixedTriangleDiscoveryStatus::Ok)
    return result;
  return sct::CertifyQuadraticResidualSeparation(
      first_base, first_prepared, first_quadratic,
      first_half_thickness,
      second_base, second_prepared, second_quadratic,
      second_half_thickness, duration,
      {local_features, 15, true},
      {intersects ? &local_intersection : nullptr,
       intersects ? 1u : 0u, true});
}


}  // namespace

SelfContactTransactionReport SelfContactTransaction::SealCandidate(
    fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::ShellPhysicalDiagnostics& physical_diagnostics,
    const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    SelfContactTransactionReceipt* output) {
  return SealCandidateImpl(owner, token, physical_diagnostics, prepared,
                           assembly, output, nullptr);
}

SelfContactTransactionReport SelfContactTransaction::SealCandidateImpl(
    fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::ShellPhysicalDiagnostics& physical_diagnostics,
    const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    SelfContactTransactionReceipt* output,
    const sct::CandidateFailureObserver* observer) {
  if (!impl_)
    return Failure(S::NotInitialized,
        "Self-contact transaction is not initialized");
  auto& state = *impl_;
  sct::DiagnosticAttempt diagnostics(state.diagnostics.candidate,
      state.config.enable_diagnostics, prepared.owner_id,
      prepared.kinematics.base_epoch, prepared.attempt, state.diagnostic_clock);
  using Stage = SelfContactDiagnosticStage;
  using fe::trial_identity::Disjoint;
  const auto& force_diagnostics = assembly.force_.diagnostics();
  const bool same_assembly =
      assembly.valid() && assembly.transaction_ == this &&
      state.force.Authenticates(assembly.force_) &&
      assembly.owner_ == &owner &&
      assembly.active_use_identity_ == state.active_use.identity() &&
      assembly.source_id_ == state.config.source_id &&
      assembly.configuration_id_ ==
          state.config.force.configuration_id &&
      assembly.qualification_id_ ==
          state.config.force.qualification_id &&
      assembly.owner_id_ == state.owner_id &&
      assembly.base_epoch_ == state.base_epoch &&
      assembly.attempt_ == state.attempt &&
      assembly.broadphase_pairs_ ==
          state.accepted_broadphase_pair_count &&
      assembly.facet_pairs_ ==
          state.accepted_facet_pair_count &&
      assembly.discovered_features_ ==
          state.accepted_feature_observation_count &&
      assembly.potential_tasks_ ==
          state.accepted_potential_task_count &&
      assembly.local_masked_tasks_ ==
          state.accepted_local_masked_task_count &&
      assembly.exact_executed_tasks_ ==
          state.accepted_exact_executed_task_count &&
      force_diagnostics.owner_id == state.owner_id &&
      force_diagnostics.base_epoch == state.base_epoch &&
      force_diagnostics.attempt == state.attempt &&
      force_diagnostics.event_count == state.accepted_event_count &&
      force_diagnostics.configuration_id ==
          state.config.force.configuration_id &&
      force_diagnostics.qualification_id ==
          state.config.force.qualification_id &&
      force_diagnostics.active_use_identity == state.active_use.identity() &&
      assembly.activity_.valid();
  if (&owner != state.owner || !output ||
      state.phase != Impl::Phase::AssemblyRecorded ||
      !same_assembly ||
      !state.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &physical_diagnostics,
                sizeof(physical_diagnostics)) ||
      !Disjoint(output, sizeof(*output), &prepared, sizeof(prepared)) ||
      !Disjoint(output, sizeof(*output), &assembly, sizeof(assembly)))
    return state.Fail(Failure(S::InvalidInput,
        "Candidate transaction receipt or output is invalid"));

  const auto node_count =
      state.active_use.facets()->surface()->physical()->
          domain()->node_count();
  fe::NodalStamp base_stamp;
  const fe::NodalSnapshotBuffer base_output{
      state.buffers.accepted_positions,
      state.buffers.accepted_velocities, node_count};
  auto owner_report = owner.CopyAccepted(base_output, &base_stamp);
  if (owner_report.status != fe::NodalStatus::Ok) {
    auto report = Failure(S::OwnerFailure, owner_report.message);
    report.owner_status = owner_report.status;
    return state.Fail(report);
  }
  fe::NodalPreparedView authentic;
  const fe::NodalSnapshotBuffer prepared_output{
      state.buffers.prepared_positions,
      state.buffers.prepared_velocities, node_count};
  owner_report =
      owner.CopyPrepared(token, prepared_output, &authentic);
  if (owner_report.status != fe::NodalStatus::Ok) {
    auto report = Failure(S::OwnerFailure, owner_report.message);
    report.owner_status = owner_report.status;
    return state.Fail(report);
  }
  if (!fe::trial_identity::SamePrepared(prepared, authentic) ||
      base_stamp.owner_id != state.owner_id ||
      base_stamp.epoch != state.base_epoch ||
      authentic.owner_id != state.owner_id ||
      authentic.kinematics.base_epoch != state.base_epoch ||
      authentic.attempt != state.attempt ||
      authentic.stream != state.stream) {
    return state.Fail(Failure(S::IdentityMismatch,
        "Candidate owner/token/view differs from accepted assembly"));
  }

  diagnostics.Authenticate();
  if (state.rigid_group_count) {
    const auto* rigid = state.active_use.rigid();
    if (!rigid ||
        rigid->groups().size() != state.rigid_group_count ||
        authentic.temporal_scheme !=
            fe::NodalTemporalScheme::StaggeredHalfKickStart)
      return state.Fail(Failure(S::IdentityMismatch,
          "Candidate rigid owner scope differs from active use"));
    fe::NodalStamp rigid_base_stamp;
    owner_report = owner.CopyAcceptedRigidGroups(
        {state.buffers.accepted_rigid_groups,
         state.rigid_group_count},
        &rigid_base_stamp);
    if (owner_report.status != fe::NodalStatus::Ok) {
      auto report = Failure(S::OwnerFailure, owner_report.message);
      report.owner_status = owner_report.status;
      return state.Fail(report);
    }
    fe::NodalPreparedView rigid_prepared;
    owner_report = owner.CopyPreparedRigidGroups(
        token,
        {state.buffers.prepared_rigid_groups,
         state.rigid_group_count},
        &rigid_prepared);
    if (owner_report.status != fe::NodalStatus::Ok) {
      auto report = Failure(S::OwnerFailure, owner_report.message);
      report.owner_status = owner_report.status;
      return state.Fail(report);
    }
    if (!fe::trial_identity::SameStamp(
            base_stamp, rigid_base_stamp) ||
        !fe::trial_identity::SamePrepared(
            authentic, rigid_prepared))
      return state.Fail(Failure(S::IdentityMismatch,
          "Rigid snapshots differ from candidate owner identity"));
    for (std::size_t group = 0;
         group < state.rigid_group_count; ++group)
      if (!SameRigidSnapshotIdentity(
              rigid->groups()[group],
              state.buffers.accepted_rigid_groups[group]) ||
          !SameRigidSnapshotIdentity(
              rigid->groups()[group],
              state.buffers.prepared_rigid_groups[group]))
        return state.Fail(Failure(S::IdentityMismatch,
            "Rigid snapshot group identity differs from actual binding",
            group));
  }

  SelfContactPreparedActivityReceipt activity_receipt;
  const auto captured = state.physical_activity.CapturePrepared(
      owner, token, physical_diagnostics, prepared,
      assembly.activity_, &activity_receipt);
  if (captured.status != SelfContactPhysicalActivityStatus::Ok) {
    auto report = Failure(S::ActivityFailure, captured.message);
    report.activity_status = captured.status;
    report.publication_status = captured.publication_status;
    report.owner_status = captured.owner_status;
    report.candidate = captured.parent;
    return state.Fail(report);
  }
  const auto activity = activity_receipt.activity();
  if (!activity.base || !activity.current ||
      activity.parent_count != state.active_use.parents().size())
    return state.Fail(Failure(S::ActivityFailure,
        "Prepared physical activity receipt is incomplete"));
  const VectorView current_positions{
      state.buffers.prepared_positions,
      static_cast<std::uint32_t>(node_count), 3, 1};
  const VectorView base_positions{
      state.buffers.accepted_positions,
      static_cast<std::uint32_t>(node_count), 3, 1};
  SelfContactCurrentRegularityReceipt regularity_receipt;
  const auto regularity = state.regularity.Certify(
      current_positions, activity, &regularity_receipt);
  if (regularity.status !=
      SelfContactCurrentRegularityStatus::Ok) {
    auto report =
        Failure(S::RegularityFailure, regularity.message);
    report.regularity_status = regularity.status;
    report.candidate = regularity.parent;
    return state.Fail(report);
  }
  const auto regularity_results = state.regularity.results();
  if (!sct::CompleteRegularity(
          state.active_use, regularity_receipt,
          regularity_results, activity))
    return state.Fail(Failure(S::RegularityFailure,
        "Current regularity publication is incomplete or unresolved"));

  const auto parents = state.active_use.parents();
  const auto triangles = state.facet_count;
  auto evaluated = sct::EvaluateCompleteTriangles(
      state.buffers.facet_descriptors, triangles,
      current_positions, state.buffers.prepared_triangles);
  if (evaluated.status != S::Ok) return state.Fail(evaluated);
  evaluated = sct::ValidateCompleteTriangleIdentities(
      state.buffers.prepared_triangles, triangles,
      state.buffers.vertex_identity_order,
      state.buffers.edge_identity_order);
  if (evaluated.status != S::Ok) return state.Fail(evaluated);
  for (std::size_t triangle = 0; triangle < triangles; ++triangle) {
    if (!sct::Same(state.buffers.accepted_triangles[triangle].key,
                   state.buffers.prepared_triangles[triangle].key))
      return state.Fail(Failure(S::IdentityMismatch,
          "Accepted and prepared facet identities differ", triangle));
  }
  const double duration = authentic.proposed_time - authentic.base_time;
  auto bounded = BuildSweptBounds(
      state.active_use, state.buffers.facet_descriptors, triangles,
      state.buffers.parent_motion,
      state.buffers.facet_motion,
      state.buffers.facet_quadratic,
      state.buffers.node_rigid_groups,
      state.buffers.accepted_rigid_groups,
      state.buffers.prepared_rigid_groups,
      state.rigid_group_count, base_positions, current_positions,
      authentic.rigid_member_trajectory,
      duration, authentic.kick_dt,
      state.buffers.swept_facet_bounds,
      state.buffers.swept_parent_bounds,
      state.surface_parent_count);
  if (bounded.status != S::Ok) return state.Fail(bounded);

  const auto broadphase = state.broadphase.Evaluate(
      {{}, {}, SelfContactBoundsMotion::ConservativeSweptParentBounds,
       state.config.broadphase_axis,
       state.buffers.swept_parent_bounds,
       state.surface_parent_count}, state.stream);
  if (broadphase.status != SelfContactBroadphaseStatus::Ok) {
    auto report = Failure(S::BroadphaseFailure, broadphase.message);
    report.broadphase_status = broadphase.status;
    report.candidate =
        broadphase.status == SelfContactBroadphaseStatus::PairCapacity
            ? static_cast<std::size_t>(broadphase.required_pairs)
            : static_cast<std::size_t>(broadphase.parent);
    return state.Fail(report);
  }
  auto expanded = sct::ReadBroadphase(
      state.broadphase, state.stream,
      state.buffers.broadphase_pairs,
      state.storage_forecast.broadphase_pair_capacity,
      &state.candidate_broadphase_pair_count);
  if (expanded.status != S::Ok) return state.Fail(expanded);
  auto streamed = state.candidate_source.Begin(
      state.buffers.broadphase_pairs,
      state.candidate_broadphase_pair_count,
      state.buffers.surface_to_active, state.surface_parent_count,
      state.buffers.parent_facet_offsets, parents.size(), activity);
  if (streamed.status != S::Ok) return state.Fail(streamed);
  // Finalization sorted this transaction-owned ledger by immutable feature,
  // then physical ownership. Same-assembly checks above authenticate its epoch
  // and attempt. This synchronous borrow is never accessed after discard
  // and never retained for another attempt.
  if (state.facet_filters) {
    diagnostics.Stage(Stage::Filtering);
    const auto filters = state.facet_filters->CandidateScene(
        state.buffers.accepted_triangles, state.buffers.prepared_triangles,
        state.buffers.facet_motion, state.buffers.swept_facet_bounds);
    if (filters.status != self_contact_filters::Status::Ok)
      return state.Fail(sct::FacetFilterFailure(filters));
  }
  const sct::FinalizedCoverageLedger coverage_ledger(
      state.buffers.accepted_certificates, state.accepted_event_count);
  SelfContactCandidatePolicySummary summary;
  std::size_t policy_outcomes = 0;
  std::size_t crossing_work = 0;
  std::size_t nonlinear_work = 0;
  std::size_t potential_tasks = 0;
  std::size_t local_masked_tasks = 0;
  std::size_t exact_executed_tasks = 0;
  bool retain_detailed = true;
  for (;;) {
    const FixedTrianglePair* pairs = nullptr;
    std::size_t streamed_pair_count = 0;
    diagnostics.Stage(Stage::Filtering);
    streamed = state.candidate_source.Next(
        &pairs, &streamed_pair_count);
    if (streamed.status != S::Ok) return state.Fail(streamed);
    if (!streamed_pair_count) break;
    if (state.facet_filters) {
      const auto query = state.facet_filters->BeginCandidateChunk(pairs, streamed_pair_count);
      if (query.status != self_contact_filters::Status::Ok) {
        auto report = sct::FacetFilterFailure(query);
        report.filter_scope = SelfContactFacetFilterFailureScope::CandidateChunkBeforeFold;
        report.filter_chunk_begin = state.candidate_facet_pair_count;
        report.filter_chunk_pairs = streamed_pair_count;
        return state.Fail(report);
      }
    }

    std::size_t pair_count = 0;
    std::size_t chunk_nonlinear_work = 0;
    for (std::size_t pair = 0;
         pair < streamed_pair_count; ++pair) {
      const auto value = pairs[pair];
      const auto key = PairKey(
          PathKey(state.buffers.prepared_triangles[value.first].key),
          PathKey(state.buffers.prepared_triangles[value.second].key));
      auto action = sct::ClassifyCandidatePairMotion(
          state.buffers.facet_motion[value.first],
          state.buffers.swept_facet_bounds[value.first],
          state.buffers.facet_motion[value.second],
          state.buffers.swept_facet_bounds[value.second]);
      state.buffers.chunk_nonlinear_results[pair] = {};
      if (action == sct::PairMotionAction::UnsupportedRigidArc) {
        const auto first_parent =
            state.buffers.facet_motion[value.first].parent;
        const auto second_parent =
            state.buffers.facet_motion[value.second].parent;
        if (first_parent >= parents.size() ||
            second_parent >= parents.size())
          return state.Fail(Failure(
              S::IdentityMismatch,
              "Nonlinear candidate facet has no active parent",
              SIZE_MAX, state.candidate_facet_pair_count + pair));
        const auto chunk_remaining =
            state.storage_forecast.nonlinear_subdivision_work_per_chunk >
                    chunk_nonlinear_work
                ? state.storage_forecast.
                      nonlinear_subdivision_work_per_chunk -
                      chunk_nonlinear_work
                : 0;
        const auto complete_remaining =
            state.storage_forecast.
                        complete_nonlinear_subdivision_work_capacity >
                    nonlinear_work
                ? state.storage_forecast.
                      complete_nonlinear_subdivision_work_capacity -
                      nonlinear_work
                : 0;
        const auto allowed = std::min({
            state.storage_forecast.
                nonlinear_subdivision_work_per_pair,
            chunk_remaining, complete_remaining});
        auto nonlinear = allowed
            ? sct::CertifyQuadraticFacetSeparation(
                  state.buffers.accepted_triangles[value.first],
                  state.buffers.prepared_triangles[value.first],
                  state.buffers.facet_quadratic[value.first],
                  parents[first_parent].reference_half_thickness_m,
                  state.buffers.accepted_triangles[value.second],
                  state.buffers.prepared_triangles[value.second],
                  state.buffers.facet_quadratic[value.second],
                  parents[second_parent].reference_half_thickness_m,
                  duration, allowed,
                  state.storage_forecast.nonlinear_subdivision_depth)
            : sct::NonlinearSeparationResult{
                  sct::NonlinearSeparationStatus::WorkExhausted, 0, 0};
        state.buffers.chunk_nonlinear_results[pair] = nonlinear;
        ++summary.nonlinear_subdivision_pairs;
        if (nonlinear.work > SIZE_MAX - chunk_nonlinear_work ||
            nonlinear.work > SIZE_MAX - nonlinear_work ||
            nonlinear.work > SIZE_MAX -
                summary.nonlinear_subdivision_work)
          return state.Fail(Failure(
              S::ResourceLimit,
              "Nonlinear subdivision work accounting overflowed",
              value.first,
              state.candidate_facet_pair_count + pair));
        chunk_nonlinear_work += nonlinear.work;
        nonlinear_work += nonlinear.work;
        summary.nonlinear_subdivision_work += nonlinear.work;
        if (nonlinear.status ==
            sct::NonlinearSeparationStatus::CertifiedSeparated) {
          action = sct::PairMotionAction::CertifiedRigidArcSeparation;
          ++summary.motion_certified_nonlinear_separated;
        } else {
          ++summary.nonlinear_subdivision_unresolved;
          if (nonlinear.status ==
              sct::NonlinearSeparationStatus::WorkExhausted)
            ++summary.nonlinear_subdivision_work_exhausted;
          if (nonlinear.status ==
              sct::NonlinearSeparationStatus::DepthExhausted)
            ++summary.nonlinear_subdivision_depth_exhausted;
        }
      }
      if (action == sct::PairMotionAction::LinearNodalV1) {
        const auto first_parent =
            state.buffers.facet_motion[value.first].parent;
        const auto second_parent =
            state.buffers.facet_motion[value.second].parent;
        if (first_parent >= parents.size() ||
            second_parent >= parents.size())
          return state.Fail(Failure(
              S::IdentityMismatch,
              "Candidate facet prism has no active parent",
              SIZE_MAX, state.candidate_facet_pair_count + pair));
        bool valid = false;
        sct::FacetPrismSeparationAxis separated_axis =
            sct::FacetPrismSeparationAxis::None;
        self_contact_filters::Report filter_report;
        const bool separated = sct::OptionalFacetPrism(
            state.facet_filters.get(), pair, [&]() noexcept {
              return sct::CertifiedLinearFacetPrismSeparation(
                state.buffers.accepted_triangles[value.first],
                state.buffers.prepared_triangles[value.first],
                parents[first_parent].reference_half_thickness_m,
                state.buffers.accepted_triangles[value.second],
                state.buffers.prepared_triangles[value.second],
                parents[second_parent].reference_half_thickness_m,
                sct::FacetPrismAxisLimit::VertexVertex,
                &separated_axis, &valid);
            }, &separated_axis, &valid, &filter_report);
        if (filter_report.status != self_contact_filters::Status::Ok)
          return state.Fail(sct::FacetFilterFailure(filter_report));
        if (separated) {
          action = sct::PairMotionAction::CertifiedLinearSeparation;
          ++summary.axis_certified_linear_separated;
          switch (separated_axis) {
            case sct::FacetPrismSeparationAxis::EdgeCross:
              ++summary.edge_axis_certified_linear_separated;
              break;
            case sct::FacetPrismSeparationAxis::VertexEdge:
              ++summary.vertex_edge_axis_separated;
              break;
            case sct::FacetPrismSeparationAxis::VertexVertex:
              ++summary.vertex_vertex_axis_separated;
              break;
            case sct::FacetPrismSeparationAxis::FaceNormal:
            case sct::FacetPrismSeparationAxis::None:
              break;
          }
        }
        if (!valid)
          return state.Fail(Failure(
              S::IdentityMismatch,
              "Candidate facet prism certificate input is invalid",
              SIZE_MAX, state.candidate_facet_pair_count + pair));
      }
      state.buffers.chunk_raw_canonical_pairs[pair] = key;
      state.buffers.chunk_motion_actions[pair] = action;
      if (pair && sct::Compare(
              state.buffers.chunk_raw_canonical_pairs[pair - 1],
              key) >= 0)
        return state.Fail(Failure(S::IdentityMismatch,
            "Candidate chunk is not strict immutable pair order",
            SIZE_MAX, state.candidate_facet_pair_count + pair));
      if (action == sct::PairMotionAction::ExcludedSameRigidGroup) {
        ++summary.motion_excluded_same_rigid_group;
        continue;
      }
      if (action == sct::PairMotionAction::CertifiedLinearSeparation) {
        ++summary.motion_certified_linear_separated;
        continue;
      }
      state.buffers.facet_pair_chunk[pair_count] = value;
      state.buffers.chunk_canonical_pairs[pair_count] = key;
      ++pair_count;
    }
    if (pair_count >
        SIZE_MAX - summary.exact_crossing_pairs)
      return state.Fail(Failure(
          S::ResourceLimit,
          "Candidate exact crossing pair count overflowed"));
    summary.exact_crossing_pairs += pair_count;

    diagnostics.Stage(Stage::Policy);
    std::size_t validated_count = 0;
    if (pair_count) {
      auto masked = sct::BuildLocalFeatureTaskMasks(
          state.buffers.facet_descriptors, triangles,
          state.buffers.facet_pair_chunk, pair_count,
          state.buffers.chunk_feature_task_masks,
          state.storage_forecast.feature_task_mask_capacity);
      if (masked.status != S::Ok)
        return state.Fail(masked);
      diagnostics.Stage(Stage::Discovery);
      const auto discovery = state.candidate_discovery.DiscoverMasked(
          state.buffers.prepared_triangles, triangles,
          state.buffers.facet_pair_chunk, pair_count,
          state.buffers.chunk_feature_task_masks);
      diagnostics.Discovery(discovery, state.candidate_discovery.diagnostics());
      if (discovery.status != FixedTriangleDiscoveryStatus::Ok) {
        auto report = Failure(
            S::DiscoveryFailure, discovery.message,
            SIZE_MAX, discovery.input_pair);
        report.discovery_status = discovery.status;
        report.discovery_task = discovery.input_task;
        report.discovery_reason = discovery.arithmetic_reason;
        return state.Fail(report);
      }
      diagnostics.Stage(Stage::Policy);
      if (discovery.potential_tasks >
              SIZE_MAX - potential_tasks ||
          discovery.local_masked_tasks >
              SIZE_MAX - local_masked_tasks ||
          discovery.exact_executed_tasks >
              SIZE_MAX - exact_executed_tasks)
        return state.Fail(Failure(
            S::ResourceLimit,
            "Candidate feature task diagnostics overflowed"));
      potential_tasks += discovery.potential_tasks;
      local_masked_tasks += discovery.local_masked_tasks;
      exact_executed_tasks += discovery.exact_executed_tasks;
      const auto intersections =
          state.candidate_discovery.intersections();
      const auto features =
          state.candidate_discovery.features();
      auto intersection_policy = ValidatePreparedIntersections(
          intersections, state.buffers.chunk_canonical_pairs, pair_count);
      if (intersection_policy.status != S::Ok) {
        if (intersection_policy.pair < pair_count) {
          DescribeMotionFailure(
              state.active_use, state.buffers.prepared_triangles,
              state.buffers.facet_motion, state.buffers.facet_quadratic,
              state.buffers.swept_facet_bounds,
              state.buffers.facet_pair_chunk[intersection_policy.pair],
              &intersection_policy);
          if (observer)
            sct::QualificationAccess::ObserveCandidateFailure(
                *this, observer, intersection_policy,
                state.buffers.facet_pair_chunk[intersection_policy.pair],
                base_stamp, authentic, assembly, activity_receipt);
        }
        return state.Fail(intersection_policy);
      }
      // Successful native publication and prepared-intersection policy above
      // authenticate this lexical cohort. No Discover occurs before it dies.
      const sct::SortedIntersections sorted_intersections(state.candidate_discovery);
      diagnostics.Stage(Stage::Residual);
      std::size_t crossing_pair_count = 0;
      std::size_t raw_pair = 0;
      for (std::size_t pair = 0; pair < pair_count; ++pair) {
        while (raw_pair < streamed_pair_count &&
               (state.buffers.chunk_motion_actions[raw_pair] ==
                    sct::PairMotionAction::ExcludedSameRigidGroup ||
                state.buffers.chunk_motion_actions[raw_pair] ==
                    sct::PairMotionAction::CertifiedLinearSeparation))
          ++raw_pair;
        if (raw_pair >= streamed_pair_count)
          return state.Fail(Failure(
              S::IdentityMismatch,
              "Candidate motion roster ended before exact geometry",
              SIZE_MAX, state.candidate_facet_pair_count + raw_pair));
        const auto action =
            state.buffers.chunk_motion_actions[raw_pair];
        const auto facet_pair = state.buffers.facet_pair_chunk[pair];
        if (action == sct::PairMotionAction::LinearNodalV1) {
          const auto first_parent =
              state.buffers.facet_motion[facet_pair.first].parent;
          const auto second_parent =
              state.buffers.facet_motion[facet_pair.second].parent;
          if (first_parent >= parents.size() ||
              second_parent >= parents.size())
            return state.Fail(Failure(
                S::IdentityMismatch,
                "Residual linear certificate has no active parent",
                SIZE_MAX,
                state.candidate_facet_pair_count + raw_pair));
          const auto residual =
              ResidualLinearCertificate(
                  state.buffers.accepted_triangles[
                      facet_pair.first],
                  state.buffers.prepared_triangles[
                      facet_pair.first],
                  parents[first_parent].
                      reference_half_thickness_m,
                  state.buffers.accepted_triangles[
                      facet_pair.second],
                  state.buffers.prepared_triangles[
                      facet_pair.second],
                  parents[second_parent].
                      reference_half_thickness_m,
                  state.buffers.chunk_feature_task_masks[pair],
                  features, intersections);
          if (residual.status ==
              sct::LinearResidualSeparationStatus::InvalidInput)
            return state.Fail(Failure(
                S::IdentityMismatch,
                "Residual linear certificate input is invalid",
                SIZE_MAX,
                state.candidate_facet_pair_count + raw_pair));
          if (residual.status ==
              sct::LinearResidualSeparationStatus::
                  CertifiedSeparated) {
            state.buffers.chunk_motion_actions[raw_pair] =
                sct::PairMotionAction::
                    CertifiedResidualLinearSeparation;
            auto& local_result =
                state.buffers.chunk_crossings[pair];
            local_result = {};
            local_result.key =
                state.buffers.chunk_canonical_pairs[pair];
            local_result.classification =
                RepresentedIntervalClassification::
                    CertifiedSeparated;
            local_result.reason =
                RepresentedIntervalReason::None;
            local_result.work = 1;
            ++raw_pair;
            continue;
          }
          // Thickness persistence alone does not certify continuous geometry.
          // Every remaining pair passes the represented/policy geometry path.
        }
        if (action ==
            sct::PairMotionAction::CertifiedRigidArcSeparation) {
          auto& local_result =
              state.buffers.chunk_crossings[pair];
          local_result = {};
          local_result.key =
              state.buffers.chunk_canonical_pairs[pair];
          local_result.classification =
              RepresentedIntervalClassification::CertifiedSeparated;
          local_result.reason = RepresentedIntervalReason::None;
          local_result.work =
              state.buffers.
                  chunk_nonlinear_results[raw_pair].work;
          ++raw_pair;
          continue;
        }
        if (action == sct::PairMotionAction::UnsupportedRigidArc) {
          const auto first_parent =
              state.buffers.facet_motion[facet_pair.first].parent;
          const auto second_parent =
              state.buffers.facet_motion[facet_pair.second].parent;
          const auto quadratic_residual =
              first_parent < parents.size() &&
                      second_parent < parents.size()
                  ? QuadraticResidualCertificate(
                        state.buffers.accepted_triangles[
                            facet_pair.first],
                        state.buffers.prepared_triangles[
                            facet_pair.first],
                        state.buffers.facet_quadratic[
                            facet_pair.first],
                        parents[first_parent].
                            reference_half_thickness_m,
                        state.buffers.accepted_triangles[
                            facet_pair.second],
                        state.buffers.prepared_triangles[
                            facet_pair.second],
                        state.buffers.facet_quadratic[
                            facet_pair.second],
                        parents[second_parent].
                            reference_half_thickness_m,
                        duration,
                        state.buffers.
                            chunk_feature_task_masks[pair],
                        features, intersections)
                  : sct::LinearResidualSeparationResult{};
          if (quadratic_residual.status ==
              sct::LinearResidualSeparationStatus::
                  InvalidInput) {
            return state.Fail(Failure(
                S::IdentityMismatch,
                "Quadratic residual certificate input is invalid",
                SIZE_MAX,
                state.candidate_facet_pair_count + raw_pair));
          }
          if (quadratic_residual.status ==
              sct::LinearResidualSeparationStatus::
                  CertifiedSeparated) {
            state.buffers.chunk_motion_actions[raw_pair] =
                sct::PairMotionAction::
                    CertifiedQuadraticResidualSeparation;
            auto& local_result =
                state.buffers.chunk_crossings[pair];
            local_result = {};
            local_result.key =
                state.buffers.chunk_canonical_pairs[pair];
            local_result.classification =
                RepresentedIntervalClassification::
                    CertifiedSeparated;
            local_result.reason =
                RepresentedIntervalReason::None;
            local_result.work = 1;
            ++raw_pair;
            continue;
          }
          // Accepted thickness coverage is authenticated below together with
          // continuous geometric safety, never as a stand-alone shortcut.
          sct::CandidateExclusions exclusion_context{
              state.active_use,
              state.buffers.accepted_triangles[facet_pair.first],
              state.buffers.accepted_triangles[facet_pair.second],
              state.buffers.chunk_feature_task_masks[pair],
              state.buffers.facet_descriptors, state.buffers.triangle_order,
              triangles, activity, state.candidate_facet_pair_count + raw_pair};
          auto exclusion_source = exclusion_context.source();
          const auto prior_nonlinear =
              state.buffers.chunk_nonlinear_results[raw_pair];
          const auto pair_remaining =
              state.storage_forecast.
                          nonlinear_subdivision_work_per_pair >
                      prior_nonlinear.work
                  ? state.storage_forecast.
                            nonlinear_subdivision_work_per_pair -
                        prior_nonlinear.work
                  : 0;
          const auto chunk_remaining =
              state.storage_forecast.
                          nonlinear_subdivision_work_per_chunk >
                      chunk_nonlinear_work
                  ? state.storage_forecast.
                            nonlinear_subdivision_work_per_chunk -
                        chunk_nonlinear_work
                  : 0;
          const auto complete_remaining =
              state.storage_forecast.
                          complete_nonlinear_subdivision_work_capacity >
                      nonlinear_work
                  ? state.storage_forecast.
                            complete_nonlinear_subdivision_work_capacity -
                        nonlinear_work
                  : 0;
          const auto coverage_allowed = std::min({
              pair_remaining, chunk_remaining, complete_remaining});
          sct::NonlinearSeparationResult coverage;
          if (coverage_allowed) {
            coverage = sct::CertifyQuadraticFacetPolicyCoverage(
                state.buffers.accepted_triangles[
                    facet_pair.first],
                state.buffers.prepared_triangles[
                    facet_pair.first],
                state.buffers.facet_quadratic[
                    facet_pair.first],
                parents[first_parent].
                    reference_half_thickness_m,
                state.buffers.accepted_triangles[
                    facet_pair.second],
                state.buffers.prepared_triangles[
                    facet_pair.second],
                state.buffers.facet_quadratic[
                    facet_pair.second],
                parents[second_parent].
                    reference_half_thickness_m,
                duration,
                coverage_ledger,
                nullptr, 0, coverage_allowed,
                state.storage_forecast.nonlinear_subdivision_depth,
                &exclusion_source);
          } else {
            coverage.status =
                sct::NonlinearSeparationStatus::WorkExhausted;
            coverage.work_exhausted = true;
          }
          if (exclusion_source.report.status != S::Ok)
            return state.Fail(exclusion_source.report);
          if (coverage.status ==
              sct::NonlinearSeparationStatus::InvalidInput)
            return state.Fail(Failure(
                S::IdentityMismatch,
                "Quadratic ledger coverage input is invalid",
                SIZE_MAX,
                state.candidate_facet_pair_count + raw_pair));
          if (coverage.work > SIZE_MAX - chunk_nonlinear_work ||
              coverage.work > SIZE_MAX - nonlinear_work ||
              coverage.work > SIZE_MAX -
                  summary.nonlinear_subdivision_work)
            return state.Fail(Failure(
                S::ResourceLimit,
                "Quadratic ledger coverage work accounting overflowed",
                facet_pair.first,
                state.candidate_facet_pair_count + raw_pair));
          chunk_nonlinear_work += coverage.work;
          nonlinear_work += coverage.work;
          summary.nonlinear_subdivision_work += coverage.work;
          if (coverage.work > SIZE_MAX - prior_nonlinear.work)
            return state.Fail(Failure(
                S::ResourceLimit,
                "Quadratic pair work accounting overflowed",
                facet_pair.first,
                state.candidate_facet_pair_count + raw_pair));
          coverage.work += prior_nonlinear.work;
          coverage.deepest = std::max(
              coverage.deepest, prior_nonlinear.deepest);
          coverage.work_exhausted =
              coverage.work_exhausted ||
              prior_nonlinear.status ==
                  sct::NonlinearSeparationStatus::WorkExhausted;
          coverage.depth_exhausted =
              coverage.depth_exhausted ||
              prior_nonlinear.status ==
                  sct::NonlinearSeparationStatus::DepthExhausted;
          state.buffers.chunk_nonlinear_results[raw_pair] =
              coverage;
          const bool accepted_coverage =
              coverage.status ==
                  sct::NonlinearSeparationStatus::
                      CertifiedAcceptedCoverage;
          const bool exact_exclusion =
              coverage.status ==
                  sct::NonlinearSeparationStatus::
                      CertifiedExactExclusion;
          const bool local_topology =
              coverage.status ==
                  sct::NonlinearSeparationStatus::CertifiedLocalIntersection;
          if (accepted_coverage || exact_exclusion || local_topology) {
            if (!summary.nonlinear_subdivision_unresolved)
              return state.Fail(Failure(
                  S::IdentityMismatch,
                  "Quadratic ledger coverage resolved no prior pair",
                  facet_pair.first,
                  state.candidate_facet_pair_count + raw_pair));
            --summary.nonlinear_subdivision_unresolved;
            if (prior_nonlinear.status ==
                    sct::NonlinearSeparationStatus::
                        WorkExhausted &&
                summary.nonlinear_subdivision_work_exhausted)
              --summary.nonlinear_subdivision_work_exhausted;
            if (prior_nonlinear.status ==
                    sct::NonlinearSeparationStatus::
                        DepthExhausted &&
                summary.nonlinear_subdivision_depth_exhausted)
              --summary.nonlinear_subdivision_depth_exhausted;
            summary.
                motion_certified_nonlinear_accepted_coverage +=
                    accepted_coverage;
            summary.
                motion_certified_nonlinear_exact_exclusion +=
                    exact_exclusion || local_topology;
            state.buffers.chunk_motion_actions[raw_pair] =
                accepted_coverage
                    ? sct::PairMotionAction::
                          CertifiedQuadraticAcceptedCoverage
                    : local_topology
                        ? sct::PairMotionAction::CertifiedQuadraticLocalIntersection
                        : sct::PairMotionAction::CertifiedQuadraticExactExclusion;
            auto& local_result =
                state.buffers.chunk_crossings[pair];
            local_result = {};
            local_result.key =
                state.buffers.chunk_canonical_pairs[pair];
            local_result.classification =
                accepted_coverage || local_topology
                    ? RepresentedIntervalClassification::
                          CertifiedCrossingContact
                    : RepresentedIntervalClassification::
                          CertifiedExactExclusion;
            local_result.reason =
                RepresentedIntervalReason::None;
            if (accepted_coverage) {
              local_result.feature = coverage.feature;
              local_result.geometry =
                  RepresentedIntersectionGeometry::
                      PersistentAcceptedLedgerCoverage;
              local_result.accepted_event =
                  coverage.accepted_certificate;
            } else if (local_topology) {
              local_result.feature.kind =
                  RepresentedFeatureKind::TriangleIntersection;
              local_result.geometry =
                  RepresentedIntersectionGeometry::CertifiedLocalTopology;
            }
            local_result.witness_time_numerator = 0;
            local_result.witness_time_depth = 0;
            local_result.work = coverage.work;
            ++raw_pair;
            continue;
          }
          auto report = Failure(
              S::UnsupportedMotion,
              coverage.status ==
                      sct::NonlinearSeparationStatus::
                          MissingAcceptedOwner
                  ? "Quadratic contact cell has no exact accepted VF/EE owner"
                  : coverage.status ==
                            sct::NonlinearSeparationStatus::
                                OwnerAmbiguity
                        ? "Quadratic contact cell has ambiguous accepted ownership"
                        : coverage.status ==
                                  sct::NonlinearSeparationStatus::
                                      PossibleGeometricCrossing
                              ? "Quadratic contact coverage cannot exclude a true geometric crossing"
                              : "Quadratic subdivision and ledger coverage remain unresolved",
              facet_pair.first,
              state.candidate_facet_pair_count + raw_pair);
          report.crossing_reason =
              RepresentedIntervalReason::UnsupportedMotion;
          const auto nonlinear =
              state.buffers.chunk_nonlinear_results[raw_pair];
          report.nonlinear_subdivision_work = nonlinear.work;
          report.nonlinear_subdivision_depth = nonlinear.deepest;
          report.nonlinear_subdivision_work_exhausted =
              nonlinear.work_exhausted ||
              nonlinear.status ==
                  sct::NonlinearSeparationStatus::WorkExhausted;
          report.nonlinear_subdivision_depth_exhausted =
              nonlinear.depth_exhausted ||
              nonlinear.status ==
                  sct::NonlinearSeparationStatus::DepthExhausted;
          DescribeMotionFailure(
              state.active_use, state.buffers.prepared_triangles,
              state.buffers.facet_motion,
              state.buffers.facet_quadratic,
              state.buffers.swept_facet_bounds,
              facet_pair, &report);
          if (observer)
            sct::QualificationAccess::ObserveCandidateFailure(
                *this, observer, report, facet_pair, base_stamp, authentic,
                assembly, activity_receipt, &nonlinear);
          return state.Fail(report);
        }
        MakePath(
            state.buffers.accepted_triangles[facet_pair.first],
            state.buffers.prepared_triangles[facet_pair.first],
            state.buffers.facet_motion[facet_pair.first],
            state.buffers.chunk_paths + 2 * crossing_pair_count);
        MakePath(
            state.buffers.accepted_triangles[facet_pair.second],
            state.buffers.prepared_triangles[facet_pair.second],
            state.buffers.facet_motion[facet_pair.second],
            state.buffers.chunk_paths + 2 * crossing_pair_count + 1);
        state.buffers.chunk_represented_pairs[crossing_pair_count] = {
            static_cast<std::uint32_t>(2 * crossing_pair_count),
            static_cast<std::uint32_t>(2 * crossing_pair_count + 1)};
        ++crossing_pair_count;
        ++raw_pair;
      }
      diagnostics.Stage(Stage::Policy);
      auto edge_policy = sct::ValidateCandidateEdgePolicy(
          state.active_use, state.regularity, regularity_receipt,
          state.candidate_discovery.features(),
          state.buffers.facet_descriptors,
          state.buffers.triangle_order, triangles, activity,
          state.buffers.accepted_certificates,
          state.accepted_event_count);
      if (edge_policy.status != S::Ok) {
        if (observer)
          sct::QualificationAccess::ObserveCandidateFeatureFailure(
              *this, observer, edge_policy, base_stamp, authentic,
              assembly, activity_receipt);
        return state.Fail(edge_policy);
      }
      diagnostics.Stage(Stage::NativeCrossing);
      const auto execution = state.crossing.Certify(
          state.buffers.chunk_paths, 2 * crossing_pair_count,
          state.buffers.chunk_represented_pairs,
          crossing_pair_count,
          state.storage_forecast.crossing_batch_pair_capacity,
          state.buffers.chunk_raw_crossings,
          state.storage_forecast.raw_crossing_result_capacity);
      const auto& crossing = execution.native;
      diagnostics.Crossing(crossing, crossing_pair_count,
          state.storage_forecast.crossing_batch_pair_capacity);
      diagnostics.CrossingDevice(execution.device);
      if (crossing.status != RepresentedIntervalStatus::Ok) {
        auto report = Failure(
            S::CrossingFailure, crossing.message,
            crossing.native_called ? crossing.native_report.input_path
                                   : SIZE_MAX,
            crossing.input_pair);
        report.crossing_status = crossing.status;
        report.crossing_diagnostics = sct::CrossingBatchDiagnostics(crossing);
        sct::DescribeCrossingDeviceFailure(execution.device, report);
        // Native ordinals belong to a compact raw subbatch. Recover the
        // original facet pair before revoking the live activity authority.
        if (crossing.input_pair < crossing_pair_count) {
          const auto input = state.buffers.chunk_represented_pairs[
              crossing.input_pair];
          if (input.first >= 2 * crossing_pair_count ||
              input.second >= 2 * crossing_pair_count)
            return state.Fail(report);
          const auto key = PairKey(state.buffers.chunk_paths[input.first].key,
                                   state.buffers.chunk_paths[input.second].key);
          const auto* begin = state.buffers.chunk_canonical_pairs;
          const auto* found = std::lower_bound(begin, begin + pair_count, key,
              [](const auto& a, const auto& b) {
                return sct::Compare(a, b) < 0;
              });
          if (found != begin + pair_count && sct::Compare(*found, key) == 0) {
            const auto facet_pair = state.buffers.facet_pair_chunk[found - begin];
            const auto* raw_begin = state.buffers.chunk_raw_canonical_pairs;
            const auto* raw_found = std::lower_bound(
                raw_begin, raw_begin + streamed_pair_count, key,
                [](const auto& a, const auto& b) {
                  return sct::Compare(a, b) < 0;
                });
            if (raw_found != raw_begin + streamed_pair_count &&
                sct::Compare(*raw_found, key) == 0)
              report.pair = state.candidate_facet_pair_count +
                  static_cast<std::size_t>(raw_found - raw_begin);
            DescribeMotionFailure(
                state.active_use, state.buffers.prepared_triangles,
                state.buffers.facet_motion, state.buffers.facet_quadratic,
                state.buffers.swept_facet_bounds, facet_pair, &report);
            if (observer)
              sct::QualificationAccess::ObserveCandidateFailure(
                  *this, observer, report, facet_pair,
                  base_stamp, authentic, assembly, activity_receipt);
          }
        }
        return state.Fail(report);
      }
      diagnostics.Stage(Stage::Policy);
      const auto raw_crossings = crossing.results;
      if (!raw_crossings.complete ||
          raw_crossings.count != crossing_pair_count ||
          (crossing_pair_count && !raw_crossings.data))
        return state.Fail(Failure(S::CrossingFailure,
            "Crossing chunk publication is incomplete"));
      raw_pair = 0;
      std::size_t crossing_pair = 0;
      for (std::size_t pair = 0; pair < pair_count; ++pair) {
        while (raw_pair < streamed_pair_count &&
               (state.buffers.chunk_motion_actions[raw_pair] ==
                    sct::PairMotionAction::ExcludedSameRigidGroup ||
                state.buffers.chunk_motion_actions[raw_pair] ==
                    sct::PairMotionAction::CertifiedLinearSeparation))
          ++raw_pair;
        if (raw_pair >= streamed_pair_count)
          return state.Fail(Failure(
              S::IdentityMismatch,
              "Candidate motion roster ended before crossing publication",
              SIZE_MAX, state.candidate_facet_pair_count + raw_pair));
        const auto action =
            state.buffers.chunk_motion_actions[raw_pair];
        if (action ==
                sct::PairMotionAction::
                    CertifiedResidualLinearSeparation ||
            action ==
                sct::PairMotionAction::
                    CertifiedPersistentLinearContact ||
            action ==
                sct::PairMotionAction::
                    CertifiedQuadraticResidualSeparation ||
            action ==
                sct::PairMotionAction::
                    CertifiedPersistentQuadraticContact ||
            action ==
                sct::PairMotionAction::
                    CertifiedQuadraticAcceptedCoverage ||
            action ==
                sct::PairMotionAction::
                    CertifiedQuadraticExactExclusion ||
            action ==
                sct::PairMotionAction::CertifiedQuadraticLocalIntersection ||
            action ==
                sct::PairMotionAction::
                    CertifiedRigidArcSeparation) {
          ++raw_pair;
          continue;
        }
        if (action == sct::PairMotionAction::UnsupportedRigidArc) {
          ++raw_pair;
          continue;
        }
        if (crossing_pair >= raw_crossings.count)
          return state.Fail(Failure(
              S::IdentityMismatch,
              "Candidate crossing publication ended before motion roster",
              SIZE_MAX, state.candidate_facet_pair_count + raw_pair));
        auto value = raw_crossings.data[crossing_pair++];
        const auto represented_work = value.work;
        // Complete same-attempt accepted assembly, prepared activity and
        // regularity, and native edge policy are authenticated above. Exact
        // common translation preserves all relative gaps and the complete
        // local intersection set, including existing thickness obligations.
        const auto translated_local = sct::NormalizeExactTranslatedLocal(
            intersections, &value, sorted_intersections);
        if (translated_local == sct::TranslatedLocalStatus::InvalidInput)
          return state.Fail(Failure(
              S::IdentityMismatch,
              "Exact translated local proof has invalid native metadata",
              SIZE_MAX, state.candidate_facet_pair_count + raw_pair));
        // Every other first witness still requires a whole-interval proof.
        if (translated_local != sct::TranslatedLocalStatus::Certified &&
            (value.classification ==
                RepresentedIntervalClassification::CertifiedCrossingContact ||
            (value.classification ==
                RepresentedIntervalClassification::Unresolved &&
            value.reason ==
                RepresentedIntervalReason::WorkExhausted))) {
          const auto facet_pair =
              state.buffers.facet_pair_chunk[pair];
          const auto first_parent =
              state.buffers.facet_motion[facet_pair.first].parent;
          const auto second_parent =
              state.buffers.facet_motion[facet_pair.second].parent;
          if (first_parent >= parents.size() ||
              second_parent >= parents.size())
            return state.Fail(Failure(
                S::IdentityMismatch,
                "Linear policy coverage has no active parent",
                SIZE_MAX,
                state.candidate_facet_pair_count + raw_pair));
          sct::CandidateExclusions exclusion_context{
              state.active_use,
              state.buffers.accepted_triangles[facet_pair.first],
              state.buffers.accepted_triangles[facet_pair.second],
              state.buffers.chunk_feature_task_masks[pair],
              state.buffers.facet_descriptors, state.buffers.triangle_order,
              triangles, activity, state.candidate_facet_pair_count + raw_pair};
          auto exclusion_source = exclusion_context.source();
          const auto coverage =
              sct::CertifyQuadraticFacetPolicyCoverage(
                  state.buffers.accepted_triangles[
                      facet_pair.first],
                  state.buffers.prepared_triangles[
                      facet_pair.first],
                  state.buffers.facet_quadratic[
                      facet_pair.first],
                  parents[first_parent].
                      reference_half_thickness_m,
                  state.buffers.accepted_triangles[
                      facet_pair.second],
                  state.buffers.prepared_triangles[
                      facet_pair.second],
                  state.buffers.facet_quadratic[
                      facet_pair.second],
                  parents[second_parent].
                      reference_half_thickness_m,
                  duration,
                  coverage_ledger,
                  nullptr, 0,
                  state.storage_forecast.crossing_work_per_pair,
                  state.storage_forecast.crossing_depth, &exclusion_source);
          if (exclusion_source.report.status != S::Ok)
            return state.Fail(exclusion_source.report);
          if (coverage.status ==
              sct::NonlinearSeparationStatus::InvalidInput)
            return state.Fail(Failure(
                S::IdentityMismatch,
                "Linear policy coverage input is invalid",
                SIZE_MAX,
                state.candidate_facet_pair_count + raw_pair));
          ++summary.linear_policy_coverage_pairs;
          if (coverage.work >
              SIZE_MAX - summary.linear_policy_coverage_work)
            return state.Fail(Failure(
                S::ResourceLimit,
                "Linear policy coverage work accounting overflowed",
                SIZE_MAX,
                state.candidate_facet_pair_count + raw_pair));
          summary.linear_policy_coverage_work += coverage.work;
          if (coverage.work > SIZE_MAX - represented_work)
            return state.Fail(Failure(
                S::ResourceLimit, "Linear policy total work accounting overflowed",
                SIZE_MAX, state.candidate_facet_pair_count + raw_pair));
          const auto total_work = represented_work + coverage.work;
          if (coverage.status ==
              sct::NonlinearSeparationStatus::CertifiedSeparated) {
            value.classification =
                RepresentedIntervalClassification::
                    CertifiedSeparated;
            value.reason = RepresentedIntervalReason::None;
            value.work = coverage.work;
            ++summary.linear_policy_certified_separated;
          } else if (
              coverage.status ==
              sct::NonlinearSeparationStatus::
                  CertifiedAcceptedCoverage) {
            value.classification =
                RepresentedIntervalClassification::
                    CertifiedCrossingContact;
            value.reason = RepresentedIntervalReason::None;
            value.feature = coverage.feature;
            value.geometry =
                RepresentedIntersectionGeometry::
                    PersistentAcceptedLedgerCoverage;
            value.accepted_event =
                coverage.accepted_certificate;
            value.work = coverage.work;
            ++summary.linear_policy_accepted_coverage;
          } else if (
              coverage.status ==
              sct::NonlinearSeparationStatus::CertifiedLocalIntersection) {
            value.classification =
                RepresentedIntervalClassification::CertifiedCrossingContact;
            value.reason = RepresentedIntervalReason::None;
            value.feature = {};
            value.feature.kind = RepresentedFeatureKind::TriangleIntersection;
            value.geometry =
                RepresentedIntersectionGeometry::CertifiedLocalTopology;
            value.witness_time_numerator = 0;
            value.witness_time_depth = 0;
            value.accepted_event = SIZE_MAX;
            value.work = coverage.work;
            ++summary.linear_policy_exact_exclusion;
          } else if (
              coverage.status ==
              sct::NonlinearSeparationStatus::
                  CertifiedExactExclusion) {
            value.classification =
                RepresentedIntervalClassification::
                    CertifiedExactExclusion;
            value.reason = RepresentedIntervalReason::None;
            value.work = coverage.work;
            ++summary.linear_policy_exact_exclusion;
          } else {
            value.classification =
                RepresentedIntervalClassification::Unresolved;
            value.reason = RepresentedIntervalReason::WorkExhausted;
            ++summary.linear_policy_unresolved;
            summary.linear_policy_potential_contact +=
                coverage.status ==
                sct::NonlinearSeparationStatus::PotentialContact;
            summary.linear_policy_work_exhausted +=
                coverage.status ==
                sct::NonlinearSeparationStatus::WorkExhausted;
            summary.linear_policy_depth_exhausted +=
                coverage.status ==
                sct::NonlinearSeparationStatus::DepthExhausted;
            summary.linear_policy_missing_accepted_owner +=
                coverage.status ==
                sct::NonlinearSeparationStatus::
                    MissingAcceptedOwner;
            summary.linear_policy_owner_ambiguity +=
                coverage.status ==
                sct::NonlinearSeparationStatus::OwnerAmbiguity;
            summary.linear_policy_possible_geometric_crossing +=
                coverage.status ==
                sct::NonlinearSeparationStatus::
                    PossibleGeometricCrossing;
          }
          value.work = total_work;
        }
        state.buffers.chunk_crossings[pair] = value;
        const auto work = value.work;
        if (work >
            state.storage_forecast.complete_crossing_work_capacity -
                crossing_work) {
          auto report = Failure(
              S::CrossingFailure,
              "Complete crossing stream exceeds its hard work cap",
              SIZE_MAX,
              state.candidate_facet_pair_count + raw_pair);
          report.crossing_status =
              RepresentedIntervalStatus::ResourceLimit;
          return state.Fail(report);
        }
        crossing_work += work;
        ++raw_pair;
      }
      if (crossing_pair != raw_crossings.count)
        return state.Fail(Failure(
            S::IdentityMismatch,
            "Candidate crossing publication exceeds motion roster"));
      auto validated = sct::ValidateCandidatePublications({
          state.buffers.chunk_canonical_pairs,
          pair_count,
          state.candidate_discovery.features(),
          state.candidate_discovery.intersections(),
          {state.buffers.chunk_crossings, pair_count, true},
          state.buffers.accepted_certificates,
          state.accepted_event_count,
          state.buffers.chunk_validated_outcomes,
          state.storage_forecast.policy_chunk_capacity,
          &validated_count}, sorted_intersections);
      if (validated.status != S::Ok) {
        if (validated.pair < pair_count) {
          const auto facet_pair =
              state.buffers.facet_pair_chunk[validated.pair];
          DescribeMotionFailure(
              state.active_use, state.buffers.prepared_triangles,
              state.buffers.facet_motion,
              state.buffers.facet_quadratic,
              state.buffers.swept_facet_bounds,
              facet_pair, &validated);
          if (observer)
            sct::QualificationAccess::ObserveCandidateFailure(
                *this, observer, validated, facet_pair,
                base_stamp, authentic, assembly, activity_receipt);
        }
        return state.Fail(validated);
      }
      if (validated_count != pair_count)
        return state.Fail(Failure(S::IdentityMismatch,
            "Policy chunk does not cover every crossing-required pair"));
    }

    std::size_t validated_index = 0;
    for (std::size_t pair = 0;
         pair < streamed_pair_count; ++pair) {
      if (state.buffers.chunk_motion_actions[pair] ==
          sct::PairMotionAction::ExcludedSameRigidGroup) {
        auto& outcome = state.buffers.chunk_policy_outcomes[pair];
        outcome = {};
        outcome.pair =
            state.buffers.chunk_raw_canonical_pairs[pair];
        outcome.disposition =
            SelfContactCandidateDisposition::ExcludedSameRigidGroup;
      } else if (state.buffers.chunk_motion_actions[pair] ==
                 sct::PairMotionAction::CertifiedLinearSeparation) {
        auto& outcome = state.buffers.chunk_policy_outcomes[pair];
        outcome = {};
        outcome.pair =
            state.buffers.chunk_raw_canonical_pairs[pair];
        outcome.disposition =
            SelfContactCandidateDisposition::CertifiedSeparated;
      } else {
        if (validated_index >= validated_count ||
            sct::Compare(
                state.buffers.chunk_validated_outcomes[
                    validated_index].pair,
                state.buffers.chunk_raw_canonical_pairs[pair]) != 0)
          return state.Fail(Failure(S::IdentityMismatch,
              "Motion-filtered policy outcome identity differs",
              SIZE_MAX,
              state.candidate_facet_pair_count + pair));
        state.buffers.chunk_policy_outcomes[pair] =
            state.buffers.chunk_validated_outcomes[validated_index++];
      }
    }
    if (validated_index != validated_count)
      return state.Fail(Failure(S::IdentityMismatch,
          "Motion-filtered policy outcomes are incomplete"));
    sct::FoldPolicyOutcomes(
        state.buffers.chunk_policy_outcomes, streamed_pair_count,
        &summary);
    if (retain_detailed &&
        streamed_pair_count <=
            state.storage_forecast.policy_outcome_capacity -
                policy_outcomes) {
      std::copy_n(state.buffers.chunk_policy_outcomes,
                  streamed_pair_count,
                  state.buffers.policy_outcomes + policy_outcomes);
      policy_outcomes += streamed_pair_count;
    } else {
      retain_detailed = false;
      policy_outcomes = 0;
    }
    state.candidate_facet_pair_count += streamed_pair_count;
  }
  diagnostics.Stage(Stage::Finalization);
  sct::StreamingCandidateSourceReceipt stream_receipt;
  streamed = state.candidate_source.Finish(&stream_receipt);
  if (streamed.status != S::Ok ||
      !state.candidate_source.Authenticates(stream_receipt) ||
      stream_receipt.parent_pairs() !=
          state.candidate_broadphase_pair_count ||
      stream_receipt.facet_pairs() !=
          state.candidate_facet_pair_count ||
      summary.outcomes != state.candidate_facet_pair_count)
    return state.Fail(Failure(S::IdentityMismatch,
        "Candidate canonical stream lacks its complete receipt"));
  if (summary.axis_certified_linear_separated >
          summary.motion_certified_linear_separated ||
      summary.edge_axis_certified_linear_separated >
          summary.axis_certified_linear_separated ||
      summary.vertex_edge_axis_separated >
          summary.axis_certified_linear_separated -
              summary.edge_axis_certified_linear_separated ||
      summary.vertex_vertex_axis_separated >
          summary.axis_certified_linear_separated -
              summary.edge_axis_certified_linear_separated -
              summary.vertex_edge_axis_separated ||
      summary.motion_certified_linear_separated > summary.outcomes ||
      summary.motion_excluded_same_rigid_group >
          summary.outcomes -
              summary.motion_certified_linear_separated ||
      summary.motion_certified_nonlinear_separated >
          summary.nonlinear_subdivision_pairs ||
      summary.motion_certified_nonlinear_accepted_coverage >
          summary.nonlinear_subdivision_pairs -
              summary.motion_certified_nonlinear_separated ||
      summary.motion_certified_nonlinear_exact_exclusion >
          summary.nonlinear_subdivision_pairs -
              summary.motion_certified_nonlinear_separated -
              summary.
                  motion_certified_nonlinear_accepted_coverage ||
      summary.nonlinear_subdivision_unresolved !=
          summary.nonlinear_subdivision_pairs -
              summary.motion_certified_nonlinear_separated -
              summary.
                  motion_certified_nonlinear_accepted_coverage -
              summary.
                  motion_certified_nonlinear_exact_exclusion ||
      summary.nonlinear_subdivision_work_exhausted >
          summary.nonlinear_subdivision_unresolved ||
      summary.nonlinear_subdivision_depth_exhausted >
          summary.nonlinear_subdivision_unresolved ||
      summary.nonlinear_subdivision_work >
          state.storage_forecast.
              complete_nonlinear_subdivision_work_capacity ||
      summary.exact_crossing_pairs !=
          summary.outcomes -
              summary.motion_certified_linear_separated -
              summary.motion_excluded_same_rigid_group ||
      summary.exact_crossing_pairs > SIZE_MAX / 15 ||
      potential_tasks != 15 * summary.exact_crossing_pairs ||
      local_masked_tasks > potential_tasks ||
      exact_executed_tasks !=
          potential_tasks - local_masked_tasks ||
      summary.linear_policy_coverage_pairs !=
          summary.linear_policy_certified_separated +
              summary.linear_policy_accepted_coverage +
              summary.linear_policy_exact_exclusion +
              summary.linear_policy_unresolved ||
      summary.linear_policy_unresolved !=
          summary.linear_policy_potential_contact +
              summary.linear_policy_work_exhausted +
              summary.linear_policy_depth_exhausted +
              summary.linear_policy_missing_accepted_owner +
              summary.linear_policy_owner_ambiguity +
              summary.linear_policy_possible_geometric_crossing)
    return state.Fail(Failure(S::IdentityMismatch,
        "Candidate motion/local-task filter lacks complete work accounting"));
  summary.exact_crossing_work = crossing_work;
  summary.complete = true;
  summary.detailed_publication =
      retain_detailed &&
      policy_outcomes == state.candidate_facet_pair_count;

  fe::ShellPhysicalScratchParticipationReceipt participation;
  const auto sealed = state.participation.SealSelfContactCandidate(
      state.config.source_id, owner, token, authentic,
      &participation);
  if (sealed.status != fe::ShellPublicationStatus::Success) {
    auto report = Failure(S::PublicationFailure, sealed.message);
    report.publication_status = sealed.status;
    report.owner_status = sealed.nodal_status;
    return state.Fail(report);
  }

  state.phase = Impl::Phase::CandidateSealed;
  state.prepared_activity = activity_receipt;
  state.policy_outcome_count =
      summary.detailed_publication ? policy_outcomes : 0;
  state.policy_summary = summary;
  state.policy_complete = summary.detailed_publication;
  SelfContactTransactionReceipt next;
  next.transaction_ = this;
  next.owner_ = &owner;
  next.active_use_identity_ = state.active_use.identity();
  next.source_id_ = state.config.source_id;
  next.configuration_id_ = state.config.force.configuration_id;
  next.qualification_id_ = state.config.force.qualification_id;
  next.owner_id_ = state.owner_id;
  next.base_epoch_ = state.base_epoch;
  next.attempt_ = state.attempt;
  next.regularity_generation_ =
      regularity_receipt.generation();
  next.broadphase_pairs_ =
      state.candidate_broadphase_pair_count;
  next.facet_pairs_ = state.candidate_facet_pair_count;
  next.potential_tasks_ = potential_tasks;
  next.local_masked_tasks_ = local_masked_tasks;
  next.exact_executed_tasks_ = exact_executed_tasks;
  next.policy_outcomes_ = state.policy_summary.outcomes;
  next.policy_summary_ = state.policy_summary;
  next.active_parents_ =
      regularity_results.summary.active_parents;
  next.removing_parents_ =
      regularity_results.summary.removing_parents;
  next.skipped_parents_ =
      regularity_results.summary.skipped_parents;
  next.activity_ = activity_receipt;
  next.participation_ = participation;
  *output = next;
  diagnostics.Success();
  return {};
}

SelfContactTransactionReport
self_contact_transaction::QualificationAccess::
ClassifyPreparedCandidateCensusImpl(
    SelfContactTransaction& owner_transaction,
    fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    sct::NonlinearCandidateRosterEntry* roster,
    std::size_t roster_capacity,
    std::size_t* roster_count,
    sct::NonlinearCandidateRosterSummary* summary,
    sct::LinearWorkExhaustedRosterEntry* linear_roster,
    std::size_t linear_capacity,
    std::size_t* linear_count,
    sct::LinearCandidateCensusSummary* linear_summary,
    sct::PreparedMotionCertificateView* certificates,
    const fe::ShellPhysicalDiagnostics* physical_diagnostics,
    sct::QualificationPreparedCensusReceipt* census_receipt) noexcept {
  const bool collect_linear =
      linear_roster || linear_capacity || linear_count || linear_summary;
  if (!roster_count || !summary || !certificates ||
      (bool(physical_diagnostics) != bool(census_receipt)) ||
      (roster_capacity && !roster) ||
      (collect_linear &&
       (!linear_count || !linear_summary ||
        (linear_capacity && !linear_roster)))) {
    return Failure(S::InvalidInput,
        "Candidate-census qualification storage is invalid");
  }
  if (!owner_transaction.impl_)
    return Failure(S::NotInitialized,
        "Nonlinear qualification transaction is not initialized");
  auto& state = *owner_transaction.impl_;
  using sct::QualificationBorrowedRange;
  const sct::QualificationRange outputs[]{
      QualificationBorrowedRange(roster_count),
      QualificationBorrowedRange(summary),
      QualificationBorrowedRange(certificates),
      QualificationBorrowedRange(roster, roster_capacity),
      QualificationBorrowedRange(linear_roster, linear_capacity),
      QualificationBorrowedRange(linear_count, collect_linear ? 1u : 0u),
      QualificationBorrowedRange(linear_summary, collect_linear ? 1u : 0u),
      QualificationBorrowedRange(census_receipt, census_receipt ? 1u : 0u)};
  const sct::QualificationRange inputs[]{
      QualificationBorrowedRange(&owner_transaction),
      QualificationBorrowedRange(&owner),
      QualificationBorrowedRange(&assembly),
      QualificationBorrowedRange(&token),
      QualificationBorrowedRange(&prepared),
      QualificationBorrowedRange(
          physical_diagnostics, physical_diagnostics ? 1u : 0u)};
  if (!sct::ValidateQualificationRanges(
          outputs, inputs, [&](const void* data, std::size_t bytes) {
            return state.OutputDisjoint(data, bytes);
          }))
    return Failure(S::InvalidInput,
        "Candidate-census qualification output ranges are invalid");
  if (&owner != state.owner ||
      state.phase != SelfContactTransaction::Impl::Phase::
          AssemblyRecorded ||
      !assembly.valid() ||
      (physical_diagnostics && !state.force.Authenticates(assembly.force_)) ||
      assembly.transaction_ != &owner_transaction ||
      assembly.owner_ != &owner ||
      assembly.active_use_identity_ !=
          state.active_use.identity() ||
      assembly.source_id_ != state.config.source_id ||
      assembly.configuration_id_ != state.config.force.configuration_id ||
      assembly.qualification_id_ != state.config.force.qualification_id ||
      assembly.owner_id_ != state.owner_id ||
      assembly.base_epoch_ != state.base_epoch ||
      assembly.attempt_ != state.attempt) {
    return Failure(S::InvalidInput,
        "Candidate-census owner, phase or output is invalid");
  }
  *roster_count = 0;
  *summary = {};
  if (collect_linear) {
    *linear_count = 0;
    *linear_summary = {};
  }
  *certificates = {};
  if (census_receipt) *census_receipt = {};
  auto activity = assembly.activity_.activity();

  const auto node_count =
      state.active_use.facets()->surface()->physical()->
          domain()->node_count();
  fe::NodalStamp accepted_stamp;
  auto owner_report = owner.CopyAccepted(
      {state.buffers.accepted_positions,
       state.buffers.accepted_velocities, node_count},
      &accepted_stamp);
  if (owner_report.status != fe::NodalStatus::Ok) {
    auto report = Failure(S::OwnerFailure, owner_report.message);
    report.owner_status = owner_report.status;
    return report;
  }
  fe::NodalPreparedView authentic;
  owner_report = owner.CopyPrepared(
      token,
      {state.buffers.prepared_positions,
       state.buffers.prepared_velocities, node_count},
      &authentic);
  if (owner_report.status != fe::NodalStatus::Ok) {
    auto report = Failure(S::OwnerFailure, owner_report.message);
    report.owner_status = owner_report.status;
    return report;
  }
  if (!fe::trial_identity::SamePrepared(prepared, authentic) ||
      accepted_stamp.owner_id != state.owner_id ||
      accepted_stamp.epoch != state.base_epoch ||
      authentic.owner_id != state.owner_id ||
      authentic.kinematics.base_epoch != state.base_epoch ||
      authentic.attempt != state.attempt ||
      authentic.stream != state.stream) {
    return Failure(S::IdentityMismatch,
        "Nonlinear qualification owner identity differs from assembly");
  }

  if (state.rigid_group_count) {
    const auto* rigid = state.active_use.rigid();
    if (!rigid ||
        rigid->groups().size() != state.rigid_group_count ||
        authentic.temporal_scheme !=
            fe::NodalTemporalScheme::StaggeredHalfKickStart) {
      return Failure(S::IdentityMismatch,
          "Nonlinear qualification rigid scope is invalid");
    }
    fe::NodalStamp rigid_accepted;
    owner_report = owner.CopyAcceptedRigidGroups(
        {state.buffers.accepted_rigid_groups,
         state.rigid_group_count},
        &rigid_accepted);
    if (owner_report.status != fe::NodalStatus::Ok) {
      auto report = Failure(S::OwnerFailure, owner_report.message);
      report.owner_status = owner_report.status;
      return report;
    }
    fe::NodalPreparedView rigid_prepared;
    owner_report = owner.CopyPreparedRigidGroups(
        token,
        {state.buffers.prepared_rigid_groups,
         state.rigid_group_count},
        &rigid_prepared);
    if (owner_report.status != fe::NodalStatus::Ok) {
      auto report = Failure(S::OwnerFailure, owner_report.message);
      report.owner_status = owner_report.status;
      return report;
    }
    if (!fe::trial_identity::SameStamp(
            accepted_stamp, rigid_accepted) ||
        !fe::trial_identity::SamePrepared(
            authentic, rigid_prepared)) {
      return Failure(S::IdentityMismatch,
          "Nonlinear qualification rigid snapshots differ");
    }
    for (std::size_t group = 0;
         group < state.rigid_group_count; ++group) {
      if (!SameRigidSnapshotIdentity(
              rigid->groups()[group],
              state.buffers.accepted_rigid_groups[group]) ||
          !SameRigidSnapshotIdentity(
              rigid->groups()[group],
              state.buffers.prepared_rigid_groups[group])) {
        return Failure(S::IdentityMismatch,
            "Nonlinear qualification rigid group identity differs",
            group);
      }
    }
  }

  const VectorView accepted_positions{
      state.buffers.accepted_positions,
      static_cast<std::uint32_t>(node_count), 3, 1};
  const VectorView prepared_positions{
      state.buffers.prepared_positions,
      static_cast<std::uint32_t>(node_count), 3, 1};
  if (physical_diagnostics) {
    SelfContactPreparedActivityReceipt prepared_activity;
    const auto captured = state.physical_activity.CapturePrepared(
        owner, token, *physical_diagnostics, prepared,
        assembly.activity_, &prepared_activity);
    if (captured.status != SelfContactPhysicalActivityStatus::Ok) {
      auto report = Failure(S::ActivityFailure, captured.message);
      report.activity_status = captured.status;
      report.publication_status = captured.publication_status;
      report.owner_status = captured.owner_status;
      report.candidate = captured.parent;
      return report;
    }
    activity = prepared_activity.activity();
    if (!activity.base || !activity.current ||
        activity.parent_count != state.active_use.parents().size())
      return Failure(S::ActivityFailure,
          "Prepared census physical activity receipt is incomplete");
    SelfContactCurrentRegularityReceipt regularity_receipt;
    const auto regularity = state.regularity.Certify(
        prepared_positions, activity, &regularity_receipt);
    if (regularity.status != SelfContactCurrentRegularityStatus::Ok ||
        !sct::CompleteRegularity(
            state.active_use, regularity_receipt,
            state.regularity.results(), activity)) {
      auto report = Failure(S::RegularityFailure,
          "Prepared census current regularity is incomplete or unresolved");
      report.regularity_status = regularity.status;
      report.candidate = regularity.parent;
      return report;
    }
    census_receipt->transaction_ = &owner_transaction;
    census_receipt->assembly_ = assembly;
    census_receipt->activity_ = prepared_activity;
    auto& activity_summary = census_receipt->activity_summary_;
    activity_summary.selected = activity.parent_count;
    for (std::size_t parent = 0; parent < activity.parent_count; ++parent) {
      activity_summary.accepted_active += activity.base[parent] != 0;
      activity_summary.prepared_active += activity.current[parent] != 0;
      activity_summary.removing +=
          activity.base[parent] != 0 && activity.current[parent] == 0;
      activity_summary.inactive += activity.base[parent] == 0;
    }
    activity_summary.complete = true;
  }
  auto evaluated = sct::EvaluateCompleteTriangles(
      state.buffers.facet_descriptors, state.facet_count,
      accepted_positions, state.buffers.accepted_triangles);
  if (evaluated.status != S::Ok) return evaluated;
  evaluated = sct::EvaluateCompleteTriangles(
      state.buffers.facet_descriptors, state.facet_count,
      prepared_positions, state.buffers.prepared_triangles);
  if (evaluated.status != S::Ok) return evaluated;
  evaluated = sct::ValidateCompleteTriangleIdentities(
      state.buffers.prepared_triangles, state.facet_count,
      state.buffers.vertex_identity_order,
      state.buffers.edge_identity_order);
  if (evaluated.status != S::Ok) return evaluated;
  for (std::size_t facet = 0;
       facet < state.facet_count; ++facet) {
    if (!sct::Same(state.buffers.accepted_triangles[facet].key,
                   state.buffers.prepared_triangles[facet].key)) {
      return Failure(S::IdentityMismatch,
          "Nonlinear qualification facet identity changed", facet);
    }
  }

  const double duration =
      authentic.proposed_time - authentic.base_time;
  auto bounded = BuildSweptBounds(
      state.active_use, state.buffers.facet_descriptors,
      state.facet_count, state.buffers.parent_motion,
      state.buffers.facet_motion, state.buffers.facet_quadratic,
      state.buffers.node_rigid_groups,
      state.buffers.accepted_rigid_groups,
      state.buffers.prepared_rigid_groups,
      state.rigid_group_count, accepted_positions,
      prepared_positions, authentic.rigid_member_trajectory,
      duration, authentic.kick_dt,
      state.buffers.swept_facet_bounds,
      state.buffers.swept_parent_bounds,
      state.surface_parent_count);
  if (bounded.status != S::Ok) return bounded;

  for (std::size_t facet = 0;
       facet < state.facet_count; ++facet) {
    if (state.buffers.facet_motion[facet].motion ==
        SelfContactFacetMotion::LinearNodalV1)
      continue;
    ++summary->rigid_or_mixed_facets;
    summary->affine_rigid_or_mixed_facets +=
        state.buffers.facet_motion[facet].certified_affine;
  }

  const auto broadphase = state.broadphase.Evaluate(
      {{}, {}, SelfContactBoundsMotion::ConservativeSweptParentBounds,
       state.config.broadphase_axis,
       state.buffers.swept_parent_bounds,
       state.surface_parent_count},
      state.stream);
  if (broadphase.status != SelfContactBroadphaseStatus::Ok) {
    auto report = Failure(S::BroadphaseFailure, broadphase.message);
    report.broadphase_status = broadphase.status;
    report.candidate =
        broadphase.status == SelfContactBroadphaseStatus::PairCapacity
            ? static_cast<std::size_t>(broadphase.required_pairs)
            : static_cast<std::size_t>(broadphase.parent);
    return report;
  }
  std::size_t broadphase_count = 0;
  auto streamed = sct::ReadBroadphase(
      state.broadphase, state.stream,
      state.buffers.broadphase_pairs,
      state.storage_forecast.broadphase_pair_capacity,
      &broadphase_count);
  if (streamed.status != S::Ok) return streamed;
  summary->broadphase_parent_pairs = broadphase_count;
  const auto parents = state.active_use.parents();
  streamed = state.candidate_source.Begin(
      state.buffers.broadphase_pairs, broadphase_count,
      state.buffers.surface_to_active, state.surface_parent_count,
      state.buffers.parent_facet_offsets, parents.size(), activity);
  if (streamed.status != S::Ok) return streamed;

  std::size_t published = 0;
  std::size_t linear_published = 0;
  std::size_t nonlinear_work = 0;
  for (;;) {
    const FixedTrianglePair* pairs = nullptr;
    std::size_t pair_count = 0;
    streamed = state.candidate_source.Next(&pairs, &pair_count);
    if (streamed.status != S::Ok) return streamed;
    if (!pair_count) break;
    if (pair_count >
        SIZE_MAX - summary->streamed_facet_pairs) {
      return Failure(S::ResourceLimit,
          "Nonlinear qualification facet-pair count overflowed");
    }
    summary->streamed_facet_pairs += pair_count;
    std::size_t chunk_work = 0;
    std::size_t linear_crossing_count = 0;
    for (std::size_t pair = 0; pair < pair_count; ++pair) {
      const auto facets = pairs[pair];
      const auto action = sct::ClassifyCandidatePairMotion(
          state.buffers.facet_motion[facets.first],
          state.buffers.swept_facet_bounds[facets.first],
          state.buffers.facet_motion[facets.second],
          state.buffers.swept_facet_bounds[facets.second]);
      if (collect_linear &&
          (action == sct::PairMotionAction::LinearNodalV1 ||
           action ==
               sct::PairMotionAction::CertifiedLinearSeparation)) {
        ++linear_summary->affine_pairs;
        if (action ==
            sct::PairMotionAction::CertifiedLinearSeparation) {
          ++linear_summary->swept_bounds_separated;
          continue;
        }
        const auto first_parent =
            state.buffers.facet_motion[facets.first].parent;
        const auto second_parent =
            state.buffers.facet_motion[facets.second].parent;
        if (first_parent >= parents.size() ||
            second_parent >= parents.size()) {
          return Failure(
              S::IdentityMismatch,
              "Linear qualification pair has no active parent",
              facets.first,
              summary->streamed_facet_pairs - pair_count + pair);
        }
        bool valid = false;
        sct::FacetPrismSeparationAxis axis =
            sct::FacetPrismSeparationAxis::None;
        if (sct::CertifiedLinearFacetPrismSeparation(
                state.buffers.accepted_triangles[facets.first],
                state.buffers.prepared_triangles[facets.first],
                parents[first_parent].reference_half_thickness_m,
                state.buffers.accepted_triangles[facets.second],
                state.buffers.prepared_triangles[facets.second],
                parents[second_parent].reference_half_thickness_m,
                sct::FacetPrismAxisLimit::VertexVertex,
                &axis, &valid)) {
          ++linear_summary->prism_separated;
          continue;
        }
        if (!valid) {
          return Failure(
              S::IdentityMismatch,
              "Linear qualification prism input is invalid",
              facets.first,
              summary->streamed_facet_pairs - pair_count + pair);
        }
        ++linear_summary->exact_geometry_pairs;

        FixedTriangleFeatureTaskMask mask;
        if (BuildFixedTriangleFeatureTaskMask(
                state.buffers.prepared_triangles[facets.first],
                state.buffers.prepared_triangles[facets.second],
                &mask) != FixedTriangleDiscoveryStatus::Ok) {
          return Failure(
              S::DiscoveryFailure,
              "Linear qualification task mask failed",
              facets.first,
              summary->streamed_facet_pairs - pair_count + pair);
        }
        FixedTriangleFeatureCandidate direct_features[15];
        fixed_triangle_features::PairFeatureResult feature_result;
        if (fixed_triangle_features::EvaluatePairFeaturesMaskedOnce(
                state.buffers.prepared_triangles[facets.first],
                state.buffers.prepared_triangles[facets.second],
                mask, direct_features, 15, &feature_result) !=
            FixedTriangleDiscoveryStatus::Ok) {
          return Failure(
              S::DiscoveryFailure,
              "Linear qualification feature replay failed",
              facets.first,
              summary->streamed_facet_pairs - pair_count + pair);
        }
        FixedTriangleIntersection direct_intersection;
        bool intersects = false;
        if (fixed_triangle_features::ClassifyPairIntersection(
                state.buffers.prepared_triangles[facets.first],
                state.buffers.prepared_triangles[facets.second],
                &direct_intersection, &intersects) !=
            FixedTriangleDiscoveryStatus::Ok) {
          return Failure(
              S::DiscoveryFailure,
              "Linear qualification intersection replay failed",
              facets.first,
              summary->streamed_facet_pairs - pair_count + pair);
        }
        const FixedTriangleFeatureView direct_view{
            direct_features, feature_result.feature_count, true};
        const FixedTriangleIntersectionView intersection_view{
            intersects ? &direct_intersection : nullptr,
            intersects ? 1u : 0u, true};
        const auto residual = ResidualLinearCertificate(
            state.buffers.accepted_triangles[facets.first],
            state.buffers.prepared_triangles[facets.first],
            parents[first_parent].reference_half_thickness_m,
            state.buffers.accepted_triangles[facets.second],
            state.buffers.prepared_triangles[facets.second],
            parents[second_parent].reference_half_thickness_m,
            mask, direct_view, intersection_view);
        if (residual.status ==
            sct::LinearResidualSeparationStatus::InvalidInput) {
          return Failure(
              S::IdentityMismatch,
              "Linear qualification residual input is invalid",
              facets.first,
              summary->streamed_facet_pairs - pair_count + pair);
        }
        linear_summary->common_translation +=
            residual.exact_common_translation;
        if (residual.status ==
            sct::LinearResidualSeparationStatus::
                CertifiedSeparated) {
          ++linear_summary->residual_separated;
          continue;
        }
        const auto persistent = PersistentLinearCertificate(
            state.buffers.accepted_triangles[facets.first],
            state.buffers.prepared_triangles[facets.first],
            parents[first_parent].reference_half_thickness_m,
            state.buffers.accepted_triangles[facets.second],
            state.buffers.prepared_triangles[facets.second],
            parents[second_parent].reference_half_thickness_m,
            mask, direct_view, state.buffers.accepted_certificates,
            state.accepted_event_count);
        if (persistent.status ==
            sct::PersistentLinearContactStatus::InvalidInput) {
          return Failure(
              S::IdentityMismatch,
              "Linear qualification persistence input is invalid",
              facets.first,
              summary->streamed_facet_pairs - pair_count + pair);
        }

        const auto crossing_index = linear_crossing_count++;
        if (crossing_index >=
            state.storage_forecast.candidate_crossing_capacity) {
          return Failure(
              S::ResourceLimit,
              "Linear qualification crossing chunk overflowed",
              facets.first,
              summary->streamed_facet_pairs - pair_count + pair);
        }
        state.buffers.facet_pair_chunk[crossing_index] = facets;
        MakePath(
            state.buffers.accepted_triangles[facets.first],
            state.buffers.prepared_triangles[facets.first],
            state.buffers.facet_motion[facets.first],
            state.buffers.chunk_paths + 2 * crossing_index);
        MakePath(
            state.buffers.accepted_triangles[facets.second],
            state.buffers.prepared_triangles[facets.second],
            state.buffers.facet_motion[facets.second],
            state.buffers.chunk_paths + 2 * crossing_index + 1);
        state.buffers.chunk_represented_pairs[crossing_index] = {
            static_cast<std::uint32_t>(2 * crossing_index),
            static_cast<std::uint32_t>(2 * crossing_index + 1)};
        continue;
      }
      if (action != sct::PairMotionAction::UnsupportedRigidArc)
        continue;
      const auto first_parent =
          state.buffers.facet_motion[facets.first].parent;
      const auto second_parent =
          state.buffers.facet_motion[facets.second].parent;
      if (first_parent >= parents.size() ||
          second_parent >= parents.size()) {
        return Failure(S::IdentityMismatch,
            "Nonlinear qualification pair has no active parent",
            facets.first, summary->streamed_facet_pairs - pair_count + pair);
      }
      const auto chunk_remaining =
          state.storage_forecast.
                      nonlinear_subdivision_work_per_chunk > chunk_work
              ? state.storage_forecast.
                        nonlinear_subdivision_work_per_chunk - chunk_work
              : 0;
      const auto complete_remaining =
          state.storage_forecast.
                      complete_nonlinear_subdivision_work_capacity >
                  nonlinear_work
              ? state.storage_forecast.
                        complete_nonlinear_subdivision_work_capacity -
                    nonlinear_work
              : 0;
      const auto allowed = std::min({
          state.storage_forecast.nonlinear_subdivision_work_per_pair,
          chunk_remaining, complete_remaining});
      const auto nonlinear = allowed
          ? sct::CertifyQuadraticFacetSeparation(
                state.buffers.accepted_triangles[facets.first],
                state.buffers.prepared_triangles[facets.first],
                state.buffers.facet_quadratic[facets.first],
                parents[first_parent].reference_half_thickness_m,
                state.buffers.accepted_triangles[facets.second],
                state.buffers.prepared_triangles[facets.second],
                state.buffers.facet_quadratic[facets.second],
                parents[second_parent].reference_half_thickness_m,
                duration, allowed,
                state.storage_forecast.nonlinear_subdivision_depth)
          : sct::NonlinearSeparationResult{
                sct::NonlinearSeparationStatus::WorkExhausted, 0, 0};
      if (nonlinear.status ==
          sct::NonlinearSeparationStatus::InvalidInput) {
        return Failure(S::IdentityMismatch,
            "Nonlinear qualification certificate input is invalid",
            facets.first, summary->streamed_facet_pairs - pair_count + pair);
      }
      if (nonlinear.work > SIZE_MAX - chunk_work ||
          nonlinear.work > SIZE_MAX - nonlinear_work ||
          nonlinear.work > SIZE_MAX - summary->work) {
        return Failure(S::ResourceLimit,
            "Nonlinear qualification work count overflowed");
      }
      chunk_work += nonlinear.work;
      nonlinear_work += nonlinear.work;
      summary->work += nonlinear.work;
      ++summary->nonlinear_pairs;
      if (nonlinear.status ==
          sct::NonlinearSeparationStatus::CertifiedSeparated) {
        ++summary->certified_separated;
      } else {
        ++summary->unresolved;
        summary->potential_contact +=
            nonlinear.status ==
            sct::NonlinearSeparationStatus::PotentialContact;
        summary->work_exhausted +=
            nonlinear.status ==
            sct::NonlinearSeparationStatus::WorkExhausted;
        summary->depth_exhausted +=
            nonlinear.status ==
            sct::NonlinearSeparationStatus::DepthExhausted;
      }
      if (published < roster_capacity) {
        roster[published] = {
            facets,
            PairKey(
                PathKey(state.buffers.prepared_triangles[
                            facets.first].key),
                PathKey(state.buffers.prepared_triangles[
                            facets.second].key)),
            nonlinear};
      }
      ++published;
    }
    if (collect_linear && linear_crossing_count) {
      const auto execution = state.crossing.Certify(
          state.buffers.chunk_paths, 2 * linear_crossing_count,
          state.buffers.chunk_represented_pairs,
          linear_crossing_count,
          state.storage_forecast.crossing_batch_pair_capacity,
          state.buffers.chunk_raw_crossings,
          state.storage_forecast.raw_crossing_result_capacity);
      const auto& crossing_report = execution.native;
      if (crossing_report.status != RepresentedIntervalStatus::Ok) {
        auto report = Failure(
            S::CrossingFailure,
            "Linear qualification represented chunk failed",
            crossing_report.native_called
                ? crossing_report.native_report.input_path : SIZE_MAX,
            crossing_report.input_pair);
        report.crossing_status = crossing_report.status;
        report.crossing_diagnostics =
            sct::CrossingBatchDiagnostics(crossing_report);
        sct::DescribeCrossingDeviceFailure(execution.device, report);
        if (crossing_report.input_pair < linear_crossing_count)
          DescribeMotionFailure(
              state.active_use, state.buffers.prepared_triangles,
              state.buffers.facet_motion, state.buffers.facet_quadratic,
              state.buffers.swept_facet_bounds,
              state.buffers.facet_pair_chunk[crossing_report.input_pair],
              &report);
        return report;
      }
      const auto crossing_view = crossing_report.results;
      if (!crossing_view.complete ||
          crossing_view.count != linear_crossing_count ||
          !crossing_view.data) {
        return Failure(
            S::CrossingFailure,
            "Linear qualification crossing chunk is incomplete");
      }
      if (linear_crossing_count >
          SIZE_MAX - linear_summary->represented_pairs) {
        return Failure(
            S::ResourceLimit,
            "Linear qualification represented count overflows");
      }
      linear_summary->represented_pairs += linear_crossing_count;
      for (std::size_t crossing = 0;
           crossing < linear_crossing_count; ++crossing) {
        const auto crossing_result = crossing_view.data[crossing];
        if (linear_summary->represented_work >
                state.storage_forecast.complete_crossing_work_capacity ||
            crossing_result.work >
                state.storage_forecast.complete_crossing_work_capacity -
                    linear_summary->represented_work) {
          return Failure(
              S::ResourceLimit,
              "Linear qualification represented stream exceeds its hard work cap");
        }
        linear_summary->represented_work += crossing_result.work;
        if (crossing_result.classification ==
            RepresentedIntervalClassification::CertifiedSeparated) {
          ++linear_summary->represented_separated;
          continue;
        }
        if (crossing_result.classification ==
            RepresentedIntervalClassification::
                CertifiedCrossingContact) {
          ++linear_summary->represented_crossing;
          continue;
        }
        if (crossing_result.reason ==
            RepresentedIntervalReason::DegenerateGeometry) {
          ++linear_summary->represented_degenerate;
          continue;
        }
        if (crossing_result.reason ==
            RepresentedIntervalReason::ExactArithmeticRange) {
          ++linear_summary->represented_arithmetic_range;
          continue;
        }
        if (crossing_result.reason !=
            RepresentedIntervalReason::WorkExhausted) {
          return Failure(
              S::IdentityMismatch,
              "Linear qualification crossing status is unclassified");
        }

        ++linear_summary->represented_work_exhausted;
        const auto facets =
            state.buffers.facet_pair_chunk[crossing];
        const auto first_parent =
            state.buffers.facet_motion[facets.first].parent;
        const auto second_parent =
            state.buffers.facet_motion[facets.second].parent;
        if (first_parent >= parents.size() ||
            second_parent >= parents.size()) {
          return Failure(
              S::IdentityMismatch,
              "Linear qualification exhausted pair has no active parent",
              facets.first);
        }
        FixedTriangleFeatureTaskMask mask;
        if (BuildFixedTriangleFeatureTaskMask(
                state.buffers.prepared_triangles[facets.first],
                state.buffers.prepared_triangles[facets.second],
                &mask) != FixedTriangleDiscoveryStatus::Ok) {
          return Failure(
              S::DiscoveryFailure,
              "Linear qualification exhausted mask replay failed",
              facets.first);
        }
        FixedTriangleFeatureCandidate features[15];
        fixed_triangle_features::PairFeatureResult feature_result;
        if (fixed_triangle_features::EvaluatePairFeaturesMaskedOnce(
                state.buffers.prepared_triangles[facets.first],
                state.buffers.prepared_triangles[facets.second],
                mask, features, 15, &feature_result) !=
            FixedTriangleDiscoveryStatus::Ok) {
          return Failure(
              S::DiscoveryFailure,
              "Linear qualification exhausted feature replay failed",
              facets.first);
        }
        FixedTriangleIntersection intersection;
        bool intersects = false;
        if (fixed_triangle_features::ClassifyPairIntersection(
                state.buffers.prepared_triangles[facets.first],
                state.buffers.prepared_triangles[facets.second],
                &intersection, &intersects) !=
            FixedTriangleDiscoveryStatus::Ok) {
          return Failure(
              S::DiscoveryFailure,
              "Linear qualification exhausted intersection replay failed",
              facets.first);
        }
        const FixedTriangleFeatureView feature_view{
            features, feature_result.feature_count, true};
        const FixedTriangleIntersectionView intersection_view{
            intersects ? &intersection : nullptr,
            intersects ? 1u : 0u, true};
        const auto residual = ResidualLinearCertificate(
            state.buffers.accepted_triangles[facets.first],
            state.buffers.prepared_triangles[facets.first],
            parents[first_parent].reference_half_thickness_m,
            state.buffers.accepted_triangles[facets.second],
            state.buffers.prepared_triangles[facets.second],
            parents[second_parent].reference_half_thickness_m,
            mask, feature_view, intersection_view);
        const auto persistent = PersistentLinearCertificate(
            state.buffers.accepted_triangles[facets.first],
            state.buffers.prepared_triangles[facets.first],
            parents[first_parent].reference_half_thickness_m,
            state.buffers.accepted_triangles[facets.second],
            state.buffers.prepared_triangles[facets.second],
            parents[second_parent].reference_half_thickness_m,
            mask, feature_view,
            state.buffers.accepted_certificates,
            state.accepted_event_count);
        if (residual.status ==
                sct::LinearResidualSeparationStatus::InvalidInput ||
            persistent.status ==
                sct::PersistentLinearContactStatus::InvalidInput ||
            residual.status ==
                sct::LinearResidualSeparationStatus::
                    CertifiedSeparated) {
          return Failure(
              S::IdentityMismatch,
              "Linear qualification exhausted pair replay changed class",
              facets.first);
        }
        if (linear_published < linear_capacity) {
          linear_roster[linear_published] = {
              facets, crossing_result.key, mask,
              residual, persistent, crossing_result};
        }
        ++linear_published;
      }
    }
  }
  sct::StreamingCandidateSourceReceipt stream_receipt;
  streamed = state.candidate_source.Finish(&stream_receipt);
  if (streamed.status != S::Ok ||
      !state.candidate_source.Authenticates(stream_receipt) ||
      stream_receipt.parent_pairs() != broadphase_count ||
      stream_receipt.facet_pairs() !=
          summary->streamed_facet_pairs) {
    return Failure(S::IdentityMismatch,
        "Nonlinear qualification stream lacks its complete receipt");
  }
  if (summary->unresolved !=
          summary->potential_contact + summary->work_exhausted +
              summary->depth_exhausted ||
      summary->nonlinear_pairs !=
          summary->certified_separated + summary->unresolved) {
    return Failure(S::IdentityMismatch,
        "Nonlinear qualification census is inconsistent");
  }
  *roster_count = published;
  summary->complete = true;
  summary->roster_complete = published <= roster_capacity;
  if (collect_linear) {
    if (linear_summary->exact_geometry_pairs !=
            linear_summary->residual_separated +
                linear_summary->persistent_accepted +
                linear_summary->represented_pairs ||
        linear_summary->represented_pairs !=
            linear_summary->represented_separated +
                linear_summary->represented_crossing +
                linear_summary->represented_degenerate +
                linear_summary->represented_work_exhausted +
                linear_summary->represented_arithmetic_range ||
        linear_published !=
            linear_summary->represented_work_exhausted) {
      return Failure(
          S::IdentityMismatch,
          "Linear qualification census is inconsistent");
    }
    *linear_count = linear_published;
    linear_summary->complete = true;
    linear_summary->roster_complete =
        linear_published <= linear_capacity;
  }
  *certificates = {
      state.buffers.facet_descriptors,
      state.buffers.facet_motion,
      state.buffers.facet_quadratic,
      state.buffers.accepted_triangles,
      state.buffers.prepared_triangles,
      state.buffers.swept_facet_bounds,
      state.facet_count,
      true};
  if (!summary->roster_complete) {
    auto report = Failure(S::ResourceLimit,
        "Nonlinear qualification roster exceeds caller capacity");
    report.candidate = published;
    return report;
  }
  if (collect_linear && !linear_summary->roster_complete) {
    auto report = Failure(
        S::ResourceLimit,
        "Linear WorkExhausted roster exceeds caller capacity");
    report.candidate = linear_published;
    return report;
  }
  return {};
}

SelfContactTransactionReport
self_contact_transaction::QualificationAccess::
ClassifyPreparedNonlinearCandidates(
    SelfContactTransaction& owner_transaction,
    fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    sct::NonlinearCandidateRosterEntry* roster,
    std::size_t roster_capacity,
    std::size_t* roster_count,
    sct::NonlinearCandidateRosterSummary* summary,
    sct::PreparedMotionCertificateView* certificates) noexcept {
  return ClassifyPreparedCandidateCensus(
      owner_transaction, owner, token, prepared, assembly,
      roster, roster_capacity, roster_count, summary,
      nullptr, 0, nullptr, nullptr, certificates);
}

}  // namespace tlfea::contact
