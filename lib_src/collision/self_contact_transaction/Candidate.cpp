// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

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

void MakePath(const CurrentFixedTriangle& base,
              const CurrentFixedTriangle& current,
              const sct::MotionSupport& motion,
              RepresentedTrianglePath* output) noexcept {
  RepresentedTrianglePath next;
  next.key = PathKey(current.key);
  next.motion =
      motion.motion == SelfContactFacetMotion::LinearNodalV1
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
    std::size_t group_count, double duration,
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
  const auto& accepted = accepted_groups[group].state;
  const auto& prepared = prepared_groups[group].state;
  const double increment = duration * std::hypot(
      std::hypot(prepared.omega.x, prepared.omega.y),
      prepared.omega.z);
  constexpr double Pi = 3.141592653589793238462643383279502884;
  if (!std::isfinite(increment) || increment >= Pi)
    return false;
  const auto arm_norm = [](Vec3 point, tl::math::Vec3 center) {
    return Up(Up(std::fabs(point.x - center.x) +
                 std::fabs(point.y - center.y)) +
              std::fabs(point.z - center.z));
  };
  const double arm = std::max(
      arm_norm(first, accepted.center),
      arm_norm(second, prepared.center));
  // The admitted owner step has |omega|*drift_dt < pi and
  // drift_dt/kick_dt <= 2.  Both its cross-product fallback and
  // finite-velocity two-member branch keep the complete second-order
  // relative drift below 12*|r0|, including the half-kick startup.  L1
  // radius plus outward rounding deliberately overbounds every orientation;
  // this box can prove only separation, never crossing.
  const double radius = Up(12 * arm);
  if (!std::isfinite(radius)) return false;
  for (unsigned component = 0; component < 3; ++component) {
    const auto a = Component(accepted.center, component);
    const auto b = Component(prepared.center, component);
    SetComponent(&next.lower, component,
                 Down(std::min(a, b) - radius));
    SetComponent(&next.upper, component,
                 Up(std::max(a, b) + radius));
  }
  *output = next;
  return IsFinite(next.lower) && IsFinite(next.upper);
}

bool PointSweepBounds(
    const WeightedSurfacePoint& point, VectorView base,
    VectorView current, const std::uint32_t* node_rigid_groups,
    const fe::NodalRigidGroupSnapshot* accepted_groups,
    const fe::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count, double duration,
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
              group_count, duration, kick_duration, &node))
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
    const std::uint32_t* node_rigid_groups,
    const fe::NodalRigidGroupSnapshot* accepted_groups,
    const fe::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count, VectorView base, VectorView current,
    double duration, double kick_duration,
    SelfContactSweptParentBounds* facet_bounds,
    SelfContactSweptParentBounds* parent_bounds,
    std::size_t surface_parents) noexcept {
  if (!descriptors || !facets || !parent_motion ||
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
    bool bounded = true;
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      SelfContactSweptParentBounds point;
      if (!PointSweepBounds(
              descriptors[facet].vertices[vertex],
              base, current, node_rigid_groups,
              accepted_groups, prepared_groups, group_count,
              duration, kick_duration, &point)) {
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
    const sct::MotionSupport* motion, FixedTrianglePair pair,
    SelfContactTransactionReport* report) noexcept {
  const auto* rigid = active_use.rigid();
  const std::uint32_t facets[2]{pair.first, pair.second};
  for (unsigned side = 0; side < 2; ++side) {
    const auto facet = facets[side];
    auto& output = report->offending_motion[side];
    output.facet = triangles[facet].key;
    output.active_parent = motion[facet].parent;
    output.motion = motion[facet].motion;
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

}  // namespace

SelfContactTransactionReport SelfContactTransaction::SealCandidate(
    fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::ShellPhysicalDiagnostics& physical_diagnostics,
    const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    SelfContactTransactionReceipt* output) {
  if (!impl_)
    return Failure(S::NotInitialized,
        "Self-contact transaction is not initialized");
  auto& state = *impl_;
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
      state.buffers.node_rigid_groups,
      state.buffers.accepted_rigid_groups,
      state.buffers.prepared_rigid_groups,
      state.rigid_group_count, base_positions, current_positions,
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
  SelfContactCandidatePolicySummary summary;
  std::size_t policy_outcomes = 0;
  std::size_t crossing_work = 0;
  std::size_t potential_tasks = 0;
  std::size_t local_masked_tasks = 0;
  std::size_t exact_executed_tasks = 0;
  bool retain_detailed = true;
  for (;;) {
    const FixedTrianglePair* pairs = nullptr;
    std::size_t streamed_pair_count = 0;
    streamed = state.candidate_source.Next(
        &pairs, &streamed_pair_count);
    if (streamed.status != S::Ok) return state.Fail(streamed);
    if (!streamed_pair_count) break;

    std::size_t pair_count = 0;
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
        if (sct::CertifiedLinearFacetPrismSeparation(
                state.buffers.accepted_triangles[value.first],
                state.buffers.prepared_triangles[value.first],
                parents[first_parent].reference_half_thickness_m,
                state.buffers.accepted_triangles[value.second],
                state.buffers.prepared_triangles[value.second],
                parents[second_parent].reference_half_thickness_m,
                sct::FacetPrismAxisLimit::VertexVertex,
                &separated_axis, &valid)) {
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
      if (action == sct::PairMotionAction::UnsupportedRigidArc) {
        auto report = Failure(
            S::UnsupportedMotion,
            "Rigid-arc swept facet boxes overlap; exact arc crossing is unresolved",
            value.first, state.candidate_facet_pair_count + pair);
        report.crossing_reason =
            RepresentedIntervalReason::UnsupportedMotion;
        DescribeMotionFailure(
            state.active_use, state.buffers.prepared_triangles,
            state.buffers.facet_motion, value, &report);
        return state.Fail(report);
      }
      if (action == sct::PairMotionAction::ExcludedSameRigidGroup) {
        ++summary.motion_excluded_same_rigid_group;
        continue;
      }
      if (action == sct::PairMotionAction::CertifiedLinearSeparation) {
        ++summary.motion_certified_linear_separated;
        continue;
      }
      state.buffers.facet_pair_chunk[pair_count] = value;
      MakePath(state.buffers.accepted_triangles[value.first],
               state.buffers.prepared_triangles[value.first],
               state.buffers.facet_motion[value.first],
               state.buffers.chunk_paths + 2 * pair_count);
      MakePath(state.buffers.accepted_triangles[value.second],
               state.buffers.prepared_triangles[value.second],
               state.buffers.facet_motion[value.second],
               state.buffers.chunk_paths + 2 * pair_count + 1);
      state.buffers.chunk_represented_pairs[pair_count] = {
          static_cast<std::uint32_t>(2 * pair_count),
          static_cast<std::uint32_t>(2 * pair_count + 1)};
      state.buffers.chunk_canonical_pairs[pair_count] = key;
      ++pair_count;
    }
    if (pair_count >
        SIZE_MAX - summary.exact_crossing_pairs)
      return state.Fail(Failure(
          S::ResourceLimit,
          "Candidate exact crossing pair count overflowed"));
    summary.exact_crossing_pairs += pair_count;

    std::size_t validated_count = 0;
    if (pair_count) {
      auto masked = sct::BuildLocalFeatureTaskMasks(
          state.buffers.facet_descriptors, triangles,
          state.buffers.facet_pair_chunk, pair_count,
          state.buffers.chunk_feature_task_masks,
          state.storage_forecast.feature_task_mask_capacity);
      if (masked.status != S::Ok)
        return state.Fail(masked);
      const auto discovery = state.candidate_discovery.DiscoverMasked(
          state.buffers.prepared_triangles, triangles,
          state.buffers.facet_pair_chunk, pair_count,
          state.buffers.chunk_feature_task_masks);
      if (discovery.status != FixedTriangleDiscoveryStatus::Ok) {
        auto report = Failure(
            S::DiscoveryFailure, discovery.message,
            SIZE_MAX, discovery.input_pair);
        report.discovery_status = discovery.status;
        report.discovery_task = discovery.input_task;
        report.discovery_reason = discovery.arithmetic_reason;
        return state.Fail(report);
      }
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
      auto edge_policy = sct::ValidateCandidateEdgePolicy(
          state.active_use, state.regularity, regularity_receipt,
          state.candidate_discovery.features(),
          state.buffers.facet_descriptors,
          state.buffers.triangle_order, triangles, activity,
          state.buffers.accepted_certificates,
          state.accepted_event_count);
      if (edge_policy.status != S::Ok)
        return state.Fail(edge_policy);
      const auto crossing = state.crossing.Certify(
          state.buffers.chunk_paths, 2 * pair_count,
          state.buffers.chunk_represented_pairs, pair_count);
      if (crossing.status != RepresentedIntervalStatus::Ok) {
        auto report = Failure(
            S::CrossingFailure, crossing.message,
            crossing.input_path, crossing.input_pair);
        report.crossing_status = crossing.status;
        return state.Fail(report);
      }
      const auto raw_crossings = state.crossing.results();
      if (!raw_crossings.complete ||
          raw_crossings.count != pair_count ||
          (pair_count && !raw_crossings.data))
        return state.Fail(Failure(S::CrossingFailure,
            "Crossing chunk publication is incomplete"));
      std::size_t raw_pair = 0;
      for (std::size_t pair = 0; pair < pair_count; ++pair) {
        while (state.buffers.chunk_motion_actions[raw_pair] ==
                   sct::PairMotionAction::ExcludedSameRigidGroup ||
               state.buffers.chunk_motion_actions[raw_pair] ==
                   sct::PairMotionAction::CertifiedLinearSeparation)
          ++raw_pair;
        const auto action =
            state.buffers.chunk_motion_actions[raw_pair];
        const auto& value = raw_crossings.data[pair];
        if (action ==
            sct::PairMotionAction::CertifiedRigidArcSeparation) {
          if (value.classification !=
                  RepresentedIntervalClassification::Unresolved ||
              value.reason !=
                  RepresentedIntervalReason::UnsupportedMotion)
            return state.Fail(Failure(S::IdentityMismatch,
                "Rigid-arc separator did not replace one exact unsupported crossing",
                SIZE_MAX,
                state.candidate_facet_pair_count + raw_pair));
          state.buffers.chunk_crossings[pair] = {};
          state.buffers.chunk_crossings[pair].key = value.key;
          state.buffers.chunk_crossings[pair].classification =
              RepresentedIntervalClassification::CertifiedSeparated;
          state.buffers.chunk_crossings[pair].reason =
              RepresentedIntervalReason::None;
        } else {
          state.buffers.chunk_crossings[pair] = value;
        }
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
          &validated_count});
      if (validated.status != S::Ok)
        return state.Fail(validated);
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
      summary.exact_crossing_pairs !=
          summary.outcomes -
              summary.motion_certified_linear_separated -
              summary.motion_excluded_same_rigid_group ||
      summary.exact_crossing_pairs > SIZE_MAX / 15 ||
      potential_tasks != 15 * summary.exact_crossing_pairs ||
      local_masked_tasks > potential_tasks ||
      exact_executed_tasks !=
          potential_tasks - local_masked_tasks)
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
  return {};
}

}  // namespace tlfea::contact
