// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include "lib_src/solvers/NodalTrialIdentity.h"

#include <algorithm>

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
              RepresentedTrianglePath* output) noexcept {
  RepresentedTrianglePath next;
  next.key = PathKey(current.key);
  next.motion = RepresentedMotion::LinearNodalV1;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    next.vertices[vertex].key = current.vertex_keys[vertex];
    next.vertices[vertex].endpoint[0] = base.vertices[vertex];
    next.vertices[vertex].endpoint[1] = current.vertices[vertex];
    next.edge_keys[vertex] = current.edge_keys[vertex];
  }
  *output = next;
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

  if (state.has_rigid_motion)
    return state.Fail(Failure(S::UnsupportedMotion,
        "Rigid-arc motion is not representable by LinearNodalV1"));
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

  const VectorView accepted_device{
      authentic.base_kinematics.position_xyz,
      static_cast<std::uint32_t>(node_count), 3, 1};
  const VectorView prepared_device{
      authentic.kinematics.position_xyz,
      static_cast<std::uint32_t>(node_count), 3, 1};
  const auto broadphase = state.broadphase.Evaluate(
      {accepted_device, prepared_device,
       SelfContactBoundsMotion::LinearNodalEndpoints,
       state.config.broadphase_axis}, state.stream);
  if (broadphase.status != SelfContactBroadphaseStatus::Ok) {
    auto report = Failure(S::BroadphaseFailure, broadphase.message);
    report.broadphase_status = broadphase.status;
    report.candidate = broadphase.parent;
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
  bool retain_detailed = true;
  for (;;) {
    const FixedTrianglePair* pairs = nullptr;
    std::size_t pair_count = 0;
    streamed = state.candidate_source.Next(&pairs, &pair_count);
    if (streamed.status != S::Ok) return state.Fail(streamed);
    if (!pair_count) break;

    for (std::size_t pair = 0; pair < pair_count; ++pair) {
      const auto value = pairs[pair];
      MakePath(state.buffers.accepted_triangles[value.first],
               state.buffers.prepared_triangles[value.first],
               state.buffers.chunk_paths + 2 * pair);
      MakePath(state.buffers.accepted_triangles[value.second],
               state.buffers.prepared_triangles[value.second],
               state.buffers.chunk_paths + 2 * pair + 1);
      state.buffers.chunk_represented_pairs[pair] = {
          static_cast<std::uint32_t>(2 * pair),
          static_cast<std::uint32_t>(2 * pair + 1)};
      state.buffers.chunk_canonical_pairs[pair] = PairKey(
          state.buffers.chunk_paths[2 * pair].key,
          state.buffers.chunk_paths[2 * pair + 1].key);
      if (pair && sct::Compare(
              state.buffers.chunk_canonical_pairs[pair - 1],
              state.buffers.chunk_canonical_pairs[pair]) >= 0)
        return state.Fail(Failure(S::IdentityMismatch,
            "Candidate chunk is not strict immutable pair order",
            SIZE_MAX, state.candidate_facet_pair_count + pair));
    }

    const auto discovery = state.candidate_discovery.Discover(
        state.buffers.prepared_triangles, triangles,
        pairs, pair_count);
    if (discovery.status != FixedTriangleDiscoveryStatus::Ok) {
      auto report = Failure(
          S::DiscoveryFailure, discovery.message,
          SIZE_MAX,
          discovery.input_pair == SIZE_MAX
              ? SIZE_MAX
              : state.candidate_facet_pair_count +
                    discovery.input_pair);
      report.discovery_status = discovery.status;
      report.discovery_task = discovery.input_task;
      report.discovery_reason = discovery.arithmetic_reason;
      return state.Fail(report);
    }
    auto edge_policy = sct::ValidateCandidateEdgePolicy(
        state.active_use, state.regularity, regularity_receipt,
        state.candidate_discovery.features(),
        state.buffers.facet_descriptors,
        state.buffers.triangle_order, triangles, activity);
    if (edge_policy.status != S::Ok)
      return state.Fail(edge_policy);
    const auto crossing = state.crossing.Certify(
        state.buffers.chunk_paths, 2 * pair_count,
        state.buffers.chunk_represented_pairs, pair_count);
    if (crossing.status != RepresentedIntervalStatus::Ok) {
      auto report = Failure(
          S::CrossingFailure, crossing.message,
          crossing.input_path,
          crossing.input_pair == SIZE_MAX
              ? SIZE_MAX
              : state.candidate_facet_pair_count +
                    crossing.input_pair);
      report.crossing_status = crossing.status;
      return state.Fail(report);
    }
    const auto crossing_results = state.crossing.results();
    if (!crossing_results.complete ||
        crossing_results.count != pair_count)
      return state.Fail(Failure(S::CrossingFailure,
          "Crossing chunk publication is incomplete"));
    for (std::size_t pair = 0; pair < pair_count; ++pair) {
      const auto work = crossing_results.data[pair].work;
      if (work >
          state.storage_forecast.complete_crossing_work_capacity -
              crossing_work) {
        auto report = Failure(
            S::CrossingFailure,
            "Complete crossing stream exceeds its hard work cap",
            SIZE_MAX, state.candidate_facet_pair_count + pair);
        report.crossing_status =
            RepresentedIntervalStatus::ResourceLimit;
        return state.Fail(report);
      }
      crossing_work += work;
    }
    std::size_t chunk_outcomes = 0;
    auto validated = sct::ValidateCandidatePublications({
        state.buffers.chunk_canonical_pairs,
        pair_count,
        state.candidate_discovery.features(),
        state.candidate_discovery.intersections(),
        crossing_results,
        state.buffers.accepted_certificates,
        state.accepted_event_count,
        state.buffers.chunk_policy_outcomes,
        state.storage_forecast.policy_chunk_capacity,
        &chunk_outcomes});
    if (validated.status != S::Ok) {
      if (validated.pair != SIZE_MAX)
        validated.pair += state.candidate_facet_pair_count;
      return state.Fail(validated);
    }
    if (chunk_outcomes != pair_count)
      return state.Fail(Failure(S::IdentityMismatch,
          "Policy chunk does not cover every facet pair"));
    sct::FoldPolicyOutcomes(
        state.buffers.chunk_policy_outcomes, chunk_outcomes,
        &summary);
    if (retain_detailed &&
        chunk_outcomes <=
            state.storage_forecast.policy_outcome_capacity -
                policy_outcomes) {
      std::copy_n(state.buffers.chunk_policy_outcomes,
                  chunk_outcomes,
                  state.buffers.policy_outcomes + policy_outcomes);
      policy_outcomes += chunk_outcomes;
    } else {
      retain_detailed = false;
      policy_outcomes = 0;
    }
    state.candidate_facet_pair_count += pair_count;
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
