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

bool ValidRange(const void* pointer, std::size_t count,
                std::size_t width) noexcept {
  if (!count) return pointer == nullptr;
  if (!pointer || count > SIZE_MAX / width) return false;
  const auto address = reinterpret_cast<std::uintptr_t>(pointer);
  return count * width <= UINTPTR_MAX - address;
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

bool PairLess(const RepresentedIntervalPairKey& a,
              const RepresentedIntervalPairKey& b) noexcept {
  return sct::Compare(a, b) < 0;
}

bool CompleteRegularity(
    const SelfContactActiveUseBinding& active_use,
    const SelfContactCurrentRegularityReceipt& receipt,
    SelfContactCurrentRegularityView view) noexcept {
  const auto parents = active_use.parents();
  const auto source_instance_id =
      active_use.facets()->surface()->physical()->
          domain()->source_instance_id();
  if (!receipt.prepared() || !view.complete ||
      view.count != parents.size() ||
      view.summary.generation != receipt.generation() ||
      view.summary.parents != parents.size() ||
      view.summary.certified_parents != parents.size() ||
      view.summary.active_parents != parents.size() ||
      view.summary.removing_parents || view.summary.skipped_parents)
    return false;
  for (std::size_t parent = 0; parent < parents.size(); ++parent) {
    const auto& expected = parents[parent];
    const auto& result = view.data[parent];
    if (result.source_instance_id != source_instance_id ||
        result.source_eid != expected.source.source_parent_id ||
        result.binding_parent != parent ||
        result.surface_parent != expected.surface_parent ||
        result.arity != expected.arity ||
        result.level != expected.level ||
        result.facet_count != expected.facet_count ||
        result.facets_evaluated != result.facet_count ||
        result.state != SelfContactCurrentParentState::Active ||
        result.chart ==
            SelfContactCurrentChartStatus::SkippedLongInactive ||
        !result.geometry_evaluated)
      return false;
  }
  return true;
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
    const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    const SelfContactCandidateEvidence& evidence,
    SelfContactTransactionReceipt* output) {
  if (!impl_)
    return Failure(S::NotInitialized,
        "Self-contact transaction is not initialized");
  auto& state = *impl_;
  using fe::trial_identity::Disjoint;
  const auto triangle_bytes =
      evidence.triangle_count <=
          SIZE_MAX / sizeof(SelfContactCandidateTriangle)
      ? evidence.triangle_count *
          sizeof(SelfContactCandidateTriangle) : SIZE_MAX;
  const auto pair_bytes =
      evidence.pair_count <= SIZE_MAX / sizeof(FixedTrianglePair)
      ? evidence.pair_count * sizeof(FixedTrianglePair) : SIZE_MAX;
  const auto decision_bytes =
      evidence.decision_count <=
          SIZE_MAX / sizeof(SelfContactCrossingDecision)
      ? evidence.decision_count *
          sizeof(SelfContactCrossingDecision) : SIZE_MAX;
  const auto& diagnostics = assembly.force_.diagnostics();
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
      diagnostics.owner_id == state.owner_id &&
      diagnostics.base_epoch == state.base_epoch &&
      diagnostics.attempt == state.attempt &&
      diagnostics.configuration_id ==
          state.config.force.configuration_id &&
      diagnostics.qualification_id ==
          state.config.force.qualification_id &&
      diagnostics.active_use_identity == state.active_use.identity();
  if (&owner != state.owner || !output ||
      state.phase != Impl::Phase::AssemblyRecorded ||
      !same_assembly ||
      !state.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &prepared, sizeof(prepared)) ||
      !Disjoint(output, sizeof(*output), &assembly, sizeof(assembly)) ||
      !Disjoint(output, sizeof(*output), &evidence, sizeof(evidence)) ||
      !evidence.regularity || !evidence.discovery ||
      !evidence.crossing || !evidence.triangle_count ||
      evidence.triangle_count >
          state.storage_forecast.candidate_triangle_capacity ||
      !evidence.pair_count ||
      evidence.pair_count >
          state.storage_forecast.candidate_pair_capacity ||
      !ValidRange(evidence.triangles, evidence.triangle_count,
                  sizeof(SelfContactCandidateTriangle)) ||
      !ValidRange(evidence.pairs, evidence.pair_count,
                  sizeof(FixedTrianglePair)) ||
      !ValidRange(evidence.decisions, evidence.decision_count,
                  sizeof(SelfContactCrossingDecision)) ||
      !Disjoint(evidence.triangles, triangle_bytes,
                state.arena.data(), state.arena.bytes()) ||
      !Disjoint(evidence.pairs, pair_bytes,
                state.arena.data(), state.arena.bytes()) ||
      (evidence.decision_count &&
       !Disjoint(evidence.decisions, decision_bytes,
                 state.arena.data(), state.arena.bytes())) ||
      !Disjoint(output, sizeof(*output),
                evidence.triangles, triangle_bytes) ||
      !Disjoint(output, sizeof(*output),
                evidence.pairs, pair_bytes) ||
      (evidence.decision_count &&
       !Disjoint(output, sizeof(*output),
                 evidence.decisions, decision_bytes)) ||
      !Disjoint(output, sizeof(*output),
                evidence.regularity, sizeof(*evidence.regularity)) ||
      !Disjoint(output, sizeof(*output),
                evidence.discovery, sizeof(*evidence.discovery)) ||
      !Disjoint(output, sizeof(*output),
                evidence.crossing, sizeof(*evidence.crossing)))
    return state.Fail(Failure(S::InvalidInput,
        "Candidate transaction receipt, evidence or bounded ranges are invalid"));

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

  if (!evidence.regularity->binding() ||
      !evidence.regularity->binding()->SharesStorage(
          state.active_use)) {
    return state.Fail(Failure(S::IdentityMismatch,
        "Current regularity retains a foreign active-use source"));
  }
  const VectorView current_positions{
      state.buffers.prepared_positions,
      static_cast<std::uint32_t>(node_count), 3, 1};
  const SelfContactActivityView activity{
      state.buffers.activity, state.buffers.activity,
      state.active_use.parents().size()};
  SelfContactCurrentRegularityReceipt regularity_receipt;
  const auto regularity = evidence.regularity->Certify(
      current_positions, activity, &regularity_receipt);
  if (regularity.status !=
      SelfContactCurrentRegularityStatus::Ok) {
    auto report =
        Failure(S::RegularityFailure, regularity.message);
    report.regularity_status = regularity.status;
    report.candidate = regularity.parent;
    return state.Fail(report);
  }
  const auto regularity_results = evidence.regularity->results();
  if (!CompleteRegularity(
          state.active_use, regularity_receipt,
          regularity_results))
    return state.Fail(Failure(S::RegularityFailure,
        "Current regularity publication is incomplete or unresolved"));

  const VectorView base_positions{
      state.buffers.accepted_positions,
      static_cast<std::uint32_t>(node_count), 3, 1};
  const auto parents = state.active_use.parents();
  const auto* facets = state.active_use.facets();
  for (std::size_t candidate = 0;
       candidate < evidence.triangle_count; ++candidate) {
    const auto declaration = evidence.triangles[candidate];
    if (declaration.active_use_parent >= parents.size()) {
      return state.Fail(Failure(S::InvalidInput,
          "Candidate triangle parent is out of range", candidate));
    }
    const auto& parent = parents[declaration.active_use_parent];
    if (declaration.local_facet >= parent.facet_count) {
      return state.Fail(Failure(S::InvalidInput,
          "Candidate local facet is out of range", candidate));
    }
    FixedContactFacet facet;
    const auto described = facets->Describe(
        parent.surface_parent, declaration.local_facet, &facet);
    if (described.status != FixedContactFacetStatus::Ok) {
      return state.Fail(Failure(S::DiscoveryFailure,
          described.message, candidate));
    }
    CurrentFixedTriangle base;
    if (EvaluateCurrentFixedTriangle(
            facet, base_positions, &base) != Status::kOk ||
        EvaluateCurrentFixedTriangle(
            facet, current_positions,
            state.buffers.current_triangles + candidate) !=
            Status::kOk) {
      return state.Fail(Failure(S::DiscoveryFailure,
          "Actual owner facet geometry cannot be represented",
          candidate));
    }
    const auto& current =
        state.buffers.current_triangles[candidate];
    if (!sct::Same(base.key, current.key)) {
      return state.Fail(Failure(S::IdentityMismatch,
          "Accepted and prepared facet identities differ",
          candidate));
    }
    MakePath(base, current,
             state.buffers.represented_paths + candidate);
    for (std::size_t prior = 0; prior < candidate; ++prior) {
      if (sct::Same(
              state.buffers.current_triangles[prior].key,
              current.key))
        return state.Fail(Failure(S::IdentityMismatch,
            "Candidate triangle roster repeats an exact facet",
            candidate));
    }
  }

  bool all_referenced = true;
  for (std::size_t triangle = 0;
       triangle < evidence.triangle_count; ++triangle) {
    bool referenced = false;
    for (std::size_t pair = 0; pair < evidence.pair_count; ++pair)
      referenced = referenced ||
          evidence.pairs[pair].first == triangle ||
          evidence.pairs[pair].second == triangle;
    all_referenced = all_referenced && referenced;
  }
  if (!all_referenced)
    return state.Fail(Failure(S::InvalidInput,
        "Candidate triangle roster contains an unqueried facet"));

  for (std::size_t pair = 0; pair < evidence.pair_count; ++pair) {
    const auto value = evidence.pairs[pair];
    if (value.first >= evidence.triangle_count ||
        value.second >= evidence.triangle_count ||
        value.first == value.second)
      return state.Fail(Failure(S::InvalidInput,
          "Candidate pair is invalid", SIZE_MAX, pair));
    state.buffers.represented_pairs[pair] =
        {value.first, value.second};
    state.buffers.canonical_pairs[pair] = PairKey(
        state.buffers.represented_paths[value.first].key,
        state.buffers.represented_paths[value.second].key);
  }
  std::sort(state.buffers.canonical_pairs,
            state.buffers.canonical_pairs + evidence.pair_count,
            PairLess);
  for (std::size_t pair = 1; pair < evidence.pair_count; ++pair)
    if (sct::Compare(state.buffers.canonical_pairs[pair - 1],
                     state.buffers.canonical_pairs[pair]) == 0)
      return state.Fail(Failure(S::IdentityMismatch,
          "Candidate pair roster repeats an exact pair",
          SIZE_MAX, pair));

  const auto discovery = evidence.discovery->Discover(
      state.buffers.current_triangles, evidence.triangle_count,
      evidence.pairs, evidence.pair_count);
  if (discovery.status != FixedTriangleDiscoveryStatus::Ok) {
    auto report = Failure(
        S::DiscoveryFailure, discovery.message,
        SIZE_MAX, discovery.input_pair);
    report.discovery_status = discovery.status;
    return state.Fail(report);
  }
  const auto crossing = evidence.crossing->Certify(
      state.buffers.represented_paths, evidence.triangle_count,
      state.buffers.represented_pairs, evidence.pair_count);
  if (crossing.status != RepresentedIntervalStatus::Ok) {
    auto report = Failure(
        S::CrossingFailure, crossing.message,
        crossing.input_path, crossing.input_pair);
    report.crossing_status = crossing.status;
    return state.Fail(report);
  }
  auto validated = sct::ValidateCandidatePublications({
      state.buffers.canonical_pairs, evidence.pair_count,
      evidence.discovery->features(),
      evidence.discovery->intersections(),
      evidence.crossing->results(),
      evidence.decisions, evidence.decision_count,
      state.buffers.accepted_events,
      state.accepted_event_count});
  if (validated.status != S::Ok) return state.Fail(validated);

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
  next.participation_ = participation;
  *output = next;
  return {};
}

}  // namespace tlfea::contact
