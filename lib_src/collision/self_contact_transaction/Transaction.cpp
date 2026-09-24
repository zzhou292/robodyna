// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include "lib_src/solvers/NodalTrialIdentity.h"

#include <algorithm>
#include <new>

namespace tlfea::contact {
namespace {

using S = SelfContactTransactionStatus;
namespace fe = tl::fea;
namespace sct = self_contact_transaction;

SelfContactTransactionReport Failure(S status,
                                     const char* message) noexcept {
  SelfContactTransactionReport result;
  result.status = status;
  result.message = message;
  return result;
}

}  // namespace

SelfContactTransaction::SelfContactTransaction() noexcept = default;
SelfContactTransaction::~SelfContactTransaction() = default;

bool SelfContactTransaction::Impl::OutputDisjoint(
    const void* output, std::size_t bytes) const noexcept {
  using fe::trial_identity::Disjoint;
  return output && active_use.OutputDisjoint(output, bytes) &&
      Disjoint(output, bytes, this, sizeof(*this)) &&
      Disjoint(output, bytes, arena.data(), arena.bytes()) &&
      (!facet_filters || facet_filters->OutputDisjoint(output, bytes)) &&
      publication && publication->PhysicalOutputDisjoint(output, bytes);
}

void SelfContactTransaction::Impl::DiscardLocal() noexcept {
  if (facet_filters) facet_filters->Discard();
  physical_activity.DiscardTrial();
  force.DiscardTrial();
  participation.DiscardTrial();
  prepared_activity = {};
  accepted_broadphase_pair_count = 0;
  candidate_broadphase_pair_count = 0;
  accepted_facet_pair_count = 0;
  candidate_facet_pair_count = 0;
  accepted_event_count = 0;
  accepted_feature_observation_count = 0;
  accepted_potential_task_count = 0;
  accepted_local_masked_task_count = 0;
  accepted_exact_executed_task_count = 0;
  policy_outcome_count = 0;
  policy_summary = {};
  policy_complete = false;
  owner_id = base_epoch = attempt = 0;
  stream = nullptr;
  phase = Phase::Idle;
}

SelfContactTransactionReport SelfContactTransaction::Impl::Fail(
    SelfContactTransactionReport report) noexcept {
  if (owner) owner->Discard();
  if (publication) publication->DiscardTrial();
  DiscardLocal();
  return report;
}

SelfContactTransactionReport SelfContactTransaction::Initialize(
    const SelfContactTransactionConfig& config,
    const SelfContactActiveUseBinding& active_use,
    fe::FENodalState& owner,
    fe::ShellBatchPublication& publication,
    const fe::ShellPhysicalBinding& physical,
    const fe::ShellPhysicalParticipants& participants,
    const fe::ShellPhysicalPublicationIdentity& identity,
    cudaStream_t owner_stream,
    SelfContactTransactionLimits limits) try {
  if (impl_)
    return Failure(S::AlreadyInitialized,
        "Self-contact transaction is immutable");
  if (!active_use.OutputDisjoint(this, sizeof(*this)) ||
      !fe::trial_identity::Disjoint(
          this, sizeof(*this), &config, sizeof(config)) ||
      !active_use.facets() || !active_use.facets()->surface() ||
      !active_use.facets()->surface()->MatchesPhysical(physical) ||
      !fe::trial_identity::SameStamp(
          owner.accepted(), config.force.owner))
    return Failure(S::IdentityMismatch,
        "Transaction active-use, physical or fresh owner identity differs");

  const auto source = publication.ValidatePhysicalSources(
      owner, physical, participants, identity);
  if (source.status != fe::ShellPublicationStatus::Success) {
    auto report = Failure(S::PublicationFailure, source.message);
    report.publication_status = source.status;
    report.owner_status = source.nodal_status;
    return report;
  }
  const auto stream_report = owner.ValidateOwnerStream(owner_stream);
  if (stream_report.status != fe::NodalStatus::Ok) {
    auto report = Failure(S::OwnerFailure, stream_report.message);
    report.owner_status = stream_report.status;
    return report;
  }
  const auto preflight = Forecast(config, active_use, identity, limits);
  if (preflight.report.status != S::Ok) return preflight.report;

  auto next = std::make_unique<Impl>(active_use);
  next->owner = &owner;
  next->publication = &publication;
  next->config = config;
  next->owner_stream = owner_stream;
  next->storage_forecast = preflight.forecast;
  sct::Layout layout;
  const auto nodes = physical.domain()->node_count();
  const auto surface_parents =
      active_use.facets()->surface()->parents().size();
  const auto facets = active_use.facet_uses().size();
  const auto rigid_groups = active_use.rigid()
      ? active_use.rigid()->groups().size() : 0;
  if (!sct::MakeLayout(
          nodes, surface_parents, active_use.parents().size(), facets,
          rigid_groups,
          preflight.forecast.broadphase_pair_capacity,
          limits.max_facet_pair_chunk,
          config.force.event_capacity,
          limits.max_event_identity_census,
          limits.max_event_hash_slots,
          limits.max_policy_outcomes,
          limits.max_host_bytes, layout) ||
      layout.bytes != preflight.forecast.candidate_arena_bytes ||
      !next->arena.Initialize(layout.bytes))
    return Failure(S::ResourceLimit,
        "Transaction candidate arena allocation failed");
  next->layout = layout;
  if (!next->arena.Construct<double>(layout.accepted_positions) ||
      !next->arena.Construct<double>(layout.accepted_velocities) ||
      !next->arena.Construct<double>(layout.prepared_positions) ||
      !next->arena.Construct<double>(layout.prepared_velocities) ||
      !next->arena.Construct<fe::NodalRigidGroupSnapshot>(
          layout.accepted_rigid_groups) ||
      !next->arena.Construct<fe::NodalRigidGroupSnapshot>(
          layout.prepared_rigid_groups) ||
      !next->arena.Construct<std::uint32_t>(
          layout.node_rigid_groups) ||
      !next->arena.Construct<std::uint32_t>(
          layout.surface_to_active) ||
      !next->arena.Construct<std::uint32_t>(
          layout.parent_facet_offsets) ||
      !next->arena.Construct<FixedContactFacet>(
          layout.facet_descriptors) ||
      !next->arena.Construct<sct::MotionSupport>(
          layout.parent_motion) ||
      !next->arena.Construct<sct::MotionSupport>(
          layout.facet_motion) ||
      !next->arena.Construct<std::uint32_t>(layout.triangle_order) ||
      !next->arena.Construct<std::uint32_t>(
          layout.vertex_identity_order) ||
      !next->arena.Construct<std::uint32_t>(
          layout.edge_identity_order) ||
      !next->arena.Construct<CurrentFixedTriangle>(
          layout.accepted_triangles) ||
      !next->arena.Construct<CurrentFixedTriangle>(
          layout.prepared_triangles) ||
      !next->arena.Construct<SelfContactPairKey>(
          layout.broadphase_pairs) ||
      !next->arena.Construct<sct::FacetPairCursor>(
          layout.facet_pair_cursors) ||
      !next->arena.Construct<std::uint32_t>(
          layout.facet_pair_heap) ||
      !next->arena.Construct<FixedTrianglePair>(
          layout.facet_pair_chunk) ||
      !next->arena.Construct<FixedTriangleFeatureTaskMask>(
          layout.chunk_feature_task_masks) ||
      !next->arena.Construct<RepresentedTrianglePath>(
          layout.chunk_paths) ||
      !next->arena.Construct<RepresentedTrianglePair>(
          layout.chunk_represented_pairs) ||
      !next->arena.Construct<RepresentedIntervalPairKey>(
          layout.chunk_canonical_pairs) ||
      !next->arena.Construct<RepresentedIntervalPairKey>(
          layout.chunk_raw_canonical_pairs) ||
      !next->arena.Construct<sct::PairMotionAction>(
          layout.chunk_motion_actions) ||
      !next->arena.Construct<RepresentedIntervalResult>(
          layout.chunk_crossings) ||
      !next->arena.Construct<SelfContactCandidatePolicyOutcome>(
          layout.chunk_validated_outcomes) ||
      !next->arena.Construct<SelfContactForceEvent>(
          layout.chunk_events) ||
      !next->arena.Construct<sct::AcceptedEventCertificate>(
          layout.chunk_certificates) ||
      !next->arena.Construct<SelfContactSweptParentBounds>(
          layout.swept_parent_bounds) ||
      !next->arena.Construct<SelfContactSweptParentBounds>(
          layout.swept_facet_bounds) ||
      !next->arena.Construct<SelfContactForceEventIdentity>(
          layout.accepted_event_identities) ||
      !next->arena.Construct<SelfContactForceEvent>(
          layout.accepted_events) ||
      !next->arena.Construct<sct::AcceptedEventCertificate>(
          layout.accepted_certificates) ||
      !next->arena.Construct<std::uint32_t>(
          layout.accepted_event_hash) ||
      !next->arena.Construct<SelfContactCandidatePolicyOutcome>(
          layout.chunk_policy_outcomes) ||
      !next->arena.Construct<SelfContactCandidatePolicyOutcome>(
          layout.policy_outcomes))
    return Failure(S::ResourceLimit,
        "Transaction typed candidate arena construction failed");
  next->buffers = sct::Bind(next->arena.data(), layout);
  next->surface_parent_count = surface_parents;
  next->facet_count = facets;
  next->rigid_group_count = rigid_groups;
  auto pipeline = sct::InitializeStaticPipeline(
      active_use, next->buffers, nodes, surface_parents, facets);
  if (pipeline.status != S::Ok) return pipeline;
  pipeline = next->candidate_source.Initialize(
      next->buffers.facet_descriptors, facets,
      next->buffers.facet_pair_cursors,
      preflight.forecast.parent_pair_cursor_capacity,
      next->buffers.facet_pair_heap,
      preflight.forecast.parent_pair_cursor_capacity,
      next->buffers.facet_pair_chunk,
      preflight.forecast.facet_pair_chunk_capacity,
      preflight.forecast.complete_facet_pair_capacity);
  if (pipeline.status != S::Ok) return pipeline;

  const auto activity = next->physical_activity.Initialize(
      active_use, owner, publication, physical, participants,
      identity, limits.activity);
  if (activity.status != SelfContactPhysicalActivityStatus::Ok) {
    auto report = Failure(S::ActivityFailure, activity.message);
    report.activity_status = activity.status;
    report.publication_status = activity.publication_status;
    report.owner_status = activity.owner_status;
    report.candidate = activity.parent;
    return report;
  }
  const auto broadphase = next->broadphase.Initialize(
      *active_use.facets()->surface(), limits.broadphase,
      owner_stream);
  if (broadphase.status != SelfContactBroadphaseStatus::Ok) {
    auto report = Failure(S::BroadphaseFailure, broadphase.message);
    report.broadphase_status = broadphase.status;
    return report;
  }
  const auto force =
      next->force.Initialize(config.force, active_use, owner, limits.force);
  if (force.status != SelfContactForceStatus::Ok) {
    auto report = Failure(S::ForceFailure, force.message);
    report.force_status = force.status;
    report.owner_status = force.owner_status;
    return report;
  }
  const auto regularity =
      next->regularity.Initialize(active_use, limits.regularity);
  if (regularity.status != SelfContactCurrentRegularityStatus::Ok) {
    auto report = Failure(S::RegularityFailure, regularity.message);
    report.regularity_status = regularity.status;
    return report;
  }
  const auto accepted_discovery =
      next->accepted_discovery.Initialize(
          limits.accepted_discovery);
  if (accepted_discovery.status !=
      FixedTriangleDiscoveryStatus::Ok) {
    auto report =
        Failure(S::DiscoveryFailure, accepted_discovery.message);
    report.discovery_status = accepted_discovery.status;
    return report;
  }
  const auto candidate_discovery =
      next->candidate_discovery.Initialize(
          limits.candidate_discovery);
  if (candidate_discovery.status !=
      FixedTriangleDiscoveryStatus::Ok) {
    auto report =
        Failure(S::DiscoveryFailure, candidate_discovery.message);
    report.discovery_status = candidate_discovery.status;
    return report;
  }
  const auto crossing =
      next->crossing.Initialize(limits.crossing);
  if (crossing.status != RepresentedIntervalStatus::Ok) {
    auto report = Failure(S::CrossingFailure, crossing.message);
    report.crossing_status = crossing.status;
    return report;
  }
  if (config.enable_cuda_facet_filters) {
    next->facet_filters = std::make_unique<sct::FacetFilters>();
    const auto filters = next->facet_filters->Initialize(next->active_use,
        limits.max_facet_pair_chunk, limits.max_host_bytes, limits.max_device_bytes, owner_stream);
    if (filters.status != self_contact_filters::Status::Ok)
      return sct::FacetFilterFailure(filters);
  }
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return Failure(S::ResourceLimit,
      "Self-contact transaction startup allocation failed");
}

fe::ShellPhysicalScratchRosterEntry
SelfContactTransaction::roster_entry() noexcept {
  if (!impl_) return {};
  return {&impl_->participation, impl_->config.source_id};
}

SelfContactTransactionReport SelfContactTransaction::AssembleAccepted(
    fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::NodalAssemblyView& view,
    SelfContactAcceptedAssemblyReceipt* output) {
  if (!impl_)
    return Failure(S::NotInitialized,
        "Self-contact transaction is not initialized");
  auto& state = *impl_;
  state.diagnostics.candidate = {};
  sct::DiagnosticAttempt diagnostics(state.diagnostics.accepted,
      state.config.enable_diagnostics, view.owner_id, view.accepted.base_epoch,
      view.attempt, state.diagnostic_clock);
  using Stage = SelfContactDiagnosticStage;
  using fe::trial_identity::Disjoint;
  const auto parents = state.active_use.parents().size();
  const auto authenticated = owner.AuthenticateAssemblyView(token, view);
  if (authenticated.status != fe::NodalStatus::Ok) {
    auto report = Failure(S::OwnerFailure, authenticated.message);
    report.owner_status = authenticated.status;
    return state.Fail(report);
  }
  if (&owner != state.owner || !output ||
      !state.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &view, sizeof(view)) ||
      !owner.AssemblyRangeDisjoint(
          token, view, output, sizeof(*output)))
    return state.Fail(Failure(S::InvalidInput,
        "Accepted transaction owner or output is invalid"));

  if (state.phase != Impl::Phase::Idle) {
    if (view.attempt != state.attempt &&
        (owner.accepted().epoch == state.base_epoch ||
         owner.accepted().epoch == state.base_epoch + 1)) {
      state.DiscardLocal();
    } else {
      return state.Fail(Failure(S::PublicationFailure,
          "Self-contact assembly was already recorded for this attempt"));
    }
  }
  if (view.stream != state.owner_stream) {
    auto report = Failure(S::OwnerFailure,
        "Assembly stream differs from startup owner stream");
    report.owner_status = fe::NodalStatus::InvalidInput;
    return state.Fail(report);
  }
  SelfContactAcceptedActivityReceipt activity_receipt;
  const auto captured = state.physical_activity.CaptureAccepted(
      owner, token, view, &activity_receipt);
  if (captured.status != SelfContactPhysicalActivityStatus::Ok) {
    auto report = Failure(S::ActivityFailure, captured.message);
    report.activity_status = captured.status;
    report.publication_status = captured.publication_status;
    report.owner_status = captured.owner_status;
    report.candidate = captured.parent;
    return state.Fail(report);
  }
  const auto activity = activity_receipt.activity();
  if (!activity.base || activity.current != activity.base ||
      activity.parent_count != parents)
    return state.Fail(Failure(S::ActivityFailure,
        "Accepted physical activity receipt is incomplete"));

  const auto node_count =
      state.active_use.facets()->surface()->physical()->
          domain()->node_count();
  fe::NodalStamp accepted_stamp;
  const fe::NodalSnapshotBuffer accepted_output{
      state.buffers.accepted_positions,
      state.buffers.accepted_velocities, node_count};
  const auto copied =
      owner.CopyAccepted(accepted_output, &accepted_stamp);
  if (copied.status != fe::NodalStatus::Ok) {
    auto report = Failure(S::OwnerFailure, copied.message);
    report.owner_status = copied.status;
    return state.Fail(report);
  }
  if (accepted_stamp.owner_id != view.owner_id ||
      accepted_stamp.epoch != view.accepted.base_epoch ||
      accepted_stamp.node_count != view.accepted.node_count ||
      view.attempt == 0) {
    return state.Fail(Failure(S::IdentityMismatch,
        "Accepted snapshot differs from assembly source identity"));
  }

  diagnostics.Authenticate();
  const VectorView accepted_positions{
      state.buffers.accepted_positions,
      static_cast<std::uint32_t>(node_count), 3, 1};
  SelfContactCurrentRegularityReceipt regularity_receipt;
  const auto regularity = state.regularity.Certify(
      accepted_positions, activity, &regularity_receipt);
  if (regularity.status !=
      SelfContactCurrentRegularityStatus::Ok) {
    auto report =
        Failure(S::RegularityFailure, regularity.message);
    report.regularity_status = regularity.status;
    report.candidate = regularity.parent;
    return state.Fail(report);
  }
  if (!sct::CompleteRegularity(
          state.active_use, regularity_receipt,
          state.regularity.results(), activity))
    return state.Fail(Failure(S::RegularityFailure,
        "Accepted regularity publication is incomplete or unresolved"));

  auto evaluated = sct::EvaluateCompleteTriangles(
      state.buffers.facet_descriptors, state.facet_count,
      accepted_positions, state.buffers.accepted_triangles);
  if (evaluated.status != S::Ok) return state.Fail(evaluated);
  evaluated = sct::ValidateCompleteTriangleIdentities(
      state.buffers.accepted_triangles, state.facet_count,
      state.buffers.vertex_identity_order,
      state.buffers.edge_identity_order);
  if (evaluated.status != S::Ok) return state.Fail(evaluated);
  const VectorView accepted_device{
      view.accepted.position_xyz,
      static_cast<std::uint32_t>(node_count), 3, 1};
  const auto broadphase = state.broadphase.Evaluate(
      {accepted_device, {}, SelfContactBoundsMotion::Current,
       state.config.broadphase_axis}, view.stream);
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
      state.broadphase, view.stream,
      state.buffers.broadphase_pairs,
      state.storage_forecast.broadphase_pair_capacity,
      &state.accepted_broadphase_pair_count);
  if (expanded.status != S::Ok) return state.Fail(expanded);
  auto streamed = state.candidate_source.Begin(
      state.buffers.broadphase_pairs,
      state.accepted_broadphase_pair_count,
      state.buffers.surface_to_active, state.surface_parent_count,
      state.buffers.parent_facet_offsets, parents, activity);
  if (streamed.status != S::Ok) return state.Fail(streamed);
  if (state.facet_filters) {
    diagnostics.Stage(Stage::Filtering);
    const auto filters = state.facet_filters->AcceptedScene(
        state.buffers.accepted_triangles, state.buffers.facet_motion);
    if (filters.status != self_contact_filters::Status::Ok)
      return state.Fail(sct::FacetFilterFailure(filters));
  }
  const bool direct_ledger =
      state.storage_forecast.accepted_event_capacity ==
      state.storage_forecast.
          accepted_event_identity_census_capacity;
  std::fill_n(state.buffers.accepted_event_hash,
              direct_ledger
                  ? state.storage_forecast.event_hash_capacity
                  : state.storage_forecast.
                        event_identity_hash_capacity,
              UINT32_MAX);
  std::size_t event_count = 0;
  SelfContactTransactionReport census_limit;
  bool census_exceeded = false;
  std::size_t feature_observations = 0;
  std::size_t potential_tasks = 0;
  std::size_t local_masked_tasks = 0;
  std::size_t exact_executed_tasks = 0;
  for (;;) {
    const FixedTrianglePair* pairs = nullptr;
    std::size_t pair_count = 0;
    diagnostics.Stage(Stage::Filtering);
    streamed = state.candidate_source.Next(&pairs, &pair_count);
    if (streamed.status != S::Ok) return state.Fail(streamed);
    if (!pair_count) break;
    const auto streamed_pair_count = pair_count;
    auto filtered = state.facet_filters
        ? state.facet_filters->AcceptedPairs(state.buffers.facet_pair_chunk, &pair_count)
        : sct::FilterAcceptedFacetPairs(
            state.active_use, state.buffers.accepted_triangles,
            state.buffers.facet_motion, state.facet_count,
            state.buffers.facet_pair_chunk, &pair_count);
    if (filtered.status != S::Ok) return state.Fail(filtered);
    auto masked = sct::BuildLocalFeatureTaskMasks(
        state.buffers.facet_descriptors, state.facet_count,
        pairs, pair_count, state.buffers.chunk_feature_task_masks,
        state.storage_forecast.feature_task_mask_capacity);
    if (masked.status != S::Ok) return state.Fail(masked);
    diagnostics.Stage(Stage::Discovery);
    const auto discovery = state.accepted_discovery.DiscoverMasked(
        state.buffers.accepted_triangles, state.facet_count,
        pairs, pair_count, state.buffers.chunk_feature_task_masks);
    diagnostics.Discovery(discovery);
    if (discovery.status != FixedTriangleDiscoveryStatus::Ok) {
      auto report = Failure(S::DiscoveryFailure, discovery.message);
      report.discovery_status = discovery.status;
      report.pair = discovery.input_pair == SIZE_MAX
          ? SIZE_MAX
          : state.accepted_facet_pair_count + discovery.input_pair;
      report.discovery_task = discovery.input_task;
      report.discovery_reason = discovery.arithmetic_reason;
      return state.Fail(report);
    }
    diagnostics.Stage(Stage::EventAssembly);
    if (discovery.potential_tasks >
            SIZE_MAX - potential_tasks ||
        discovery.local_masked_tasks >
            SIZE_MAX - local_masked_tasks ||
        discovery.exact_executed_tasks >
            SIZE_MAX - exact_executed_tasks)
      return state.Fail(Failure(
          S::ResourceLimit,
          "Accepted feature task diagnostics overflowed"));
    potential_tasks += discovery.potential_tasks;
    local_masked_tasks += discovery.local_masked_tasks;
    exact_executed_tasks += discovery.exact_executed_tasks;
    if (discovery.feature_candidates >
        SIZE_MAX - feature_observations)
      return state.Fail(Failure(
          S::ResourceLimit,
          "Accepted feature observation count overflowed"));
    feature_observations += discovery.feature_candidates;
    std::size_t chunk_events = 0;
    auto events = sct::BuildAcceptedEvents(
        state.active_use, state.regularity, regularity_receipt,
        state.accepted_discovery.features(),
        state.accepted_discovery.intersections(),
        state.buffers.facet_descriptors,
        state.buffers.triangle_order, state.facet_count,
        activity, state.buffers.chunk_events,
        state.buffers.chunk_certificates,
        15 * state.storage_forecast.facet_pair_chunk_capacity,
        &chunk_events);
    if (events.status != S::Ok) {
      if (events.candidate != SIZE_MAX) {
        const auto chunk_feature_base =
            feature_observations - discovery.feature_candidates;
        if (events.candidate >
            SIZE_MAX - chunk_feature_base)
          return state.Fail(Failure(
              S::ResourceLimit,
              "Accepted feature diagnostic ordinal overflowed"));
        events.candidate += chunk_feature_base;
      }
      return state.Fail(events);
    }
    if (direct_ledger) {
      events = sct::MergeAcceptedEventChunk(
          state.buffers.chunk_certificates, chunk_events,
          state.buffers.accepted_certificates,
          state.storage_forecast.accepted_event_ledger_capacity,
          state.buffers.accepted_event_hash,
          state.storage_forecast.event_hash_capacity,
          &event_count);
      if (events.status == S::ResourceLimit) {
        events.candidate = event_count + 1;
        events.count_kind =
            SelfContactTransactionCountKind::
                AcceptedEventsLowerBound;
      }
      if (events.status != S::Ok) return state.Fail(events);
    } else if (!census_exceeded) {
      for (std::size_t event = 0; event < chunk_events; ++event) {
        const auto identity = SelfContactForceEventIdentityOf(
            state.buffers.chunk_certificates[event].event);
        events = sct::MergeAcceptedEventIdentityChunk(
            &identity, 1, state.buffers.accepted_event_identities,
            state.storage_forecast.
                accepted_event_identity_census_capacity,
            state.buffers.accepted_event_hash,
            state.storage_forecast.event_identity_hash_capacity,
            &event_count);
        if (events.status == S::ResourceLimit &&
            events.count_kind ==
                SelfContactTransactionCountKind::
                    AcceptedEventsLowerBound) {
          census_limit = events;
          census_exceeded = true;
          break;
        }
        if (events.status != S::Ok) return state.Fail(events);
      }
    }
    state.accepted_facet_pair_count += streamed_pair_count;
  }
  diagnostics.Stage(Stage::EventAssembly);
  sct::StreamingCandidateSourceReceipt stream_receipt;
  streamed = state.candidate_source.Finish(&stream_receipt);
  if (streamed.status != S::Ok ||
      !state.candidate_source.Authenticates(stream_receipt) ||
      stream_receipt.parent_pairs() !=
          state.accepted_broadphase_pair_count ||
      stream_receipt.facet_pairs() !=
          state.accepted_facet_pair_count)
    return state.Fail(Failure(S::IdentityMismatch,
        "Accepted canonical stream lacks its complete receipt"));
  if (local_masked_tasks > potential_tasks ||
      exact_executed_tasks != potential_tasks - local_masked_tasks)
    return state.Fail(Failure(S::IdentityMismatch,
        "Accepted local feature mask accounting is incomplete"));
  if (census_exceeded) return state.Fail(census_limit);
  if (event_count >
      state.storage_forecast.accepted_event_capacity) {
    auto report = Failure(S::ResourceLimit,
        "Complete accepted VF+EE event set exceeds force capacity");
    report.candidate = event_count;
    report.count_kind =
        SelfContactTransactionCountKind::ExactAcceptedEvents;
    return state.Fail(report);
  }
  SelfContactTransactionReport events;
  if (!direct_ledger) {
    events = sct::CanonicalizeAcceptedEventIdentityCensus(
        state.buffers.accepted_event_identities, event_count);
    if (events.status != S::Ok) return state.Fail(events);

    streamed = state.candidate_source.Begin(
        state.buffers.broadphase_pairs,
        state.accepted_broadphase_pair_count,
        state.buffers.surface_to_active, state.surface_parent_count,
        state.buffers.parent_facet_offsets, parents, activity);
    if (streamed.status != S::Ok) return state.Fail(streamed);
    std::fill_n(state.buffers.accepted_event_hash,
                state.storage_forecast.event_hash_capacity,
                UINT32_MAX);
    std::size_t verified_event_count = 0;
    std::size_t verified_facet_pair_count = 0;
    std::size_t verified_feature_observations = 0;
    std::size_t verified_potential_tasks = 0;
    std::size_t verified_local_masked_tasks = 0;
    std::size_t verified_exact_executed_tasks = 0;
    for (;;) {
    const FixedTrianglePair* pairs = nullptr;
    std::size_t pair_count = 0;
    diagnostics.Stage(Stage::Filtering);
    streamed = state.candidate_source.Next(&pairs, &pair_count);
    if (streamed.status != S::Ok) return state.Fail(streamed);
    if (!pair_count) break;
    const auto streamed_pair_count = pair_count;
    auto filtered = state.facet_filters
        ? state.facet_filters->AcceptedPairs(state.buffers.facet_pair_chunk, &pair_count)
        : sct::FilterAcceptedFacetPairs(
            state.active_use, state.buffers.accepted_triangles,
            state.buffers.facet_motion, state.facet_count,
            state.buffers.facet_pair_chunk, &pair_count);
    if (filtered.status != S::Ok) return state.Fail(filtered);
    auto masked = sct::BuildLocalFeatureTaskMasks(
        state.buffers.facet_descriptors, state.facet_count,
        pairs, pair_count, state.buffers.chunk_feature_task_masks,
        state.storage_forecast.feature_task_mask_capacity);
    if (masked.status != S::Ok) return state.Fail(masked);
    diagnostics.Stage(Stage::Discovery);
    const auto discovery = state.accepted_discovery.DiscoverMasked(
        state.buffers.accepted_triangles, state.facet_count,
        pairs, pair_count, state.buffers.chunk_feature_task_masks);
    diagnostics.Discovery(discovery);
    if (discovery.status != FixedTriangleDiscoveryStatus::Ok) {
      auto report = Failure(S::DiscoveryFailure, discovery.message);
      report.discovery_status = discovery.status;
      report.pair = discovery.input_pair == SIZE_MAX
          ? SIZE_MAX
          : verified_facet_pair_count + discovery.input_pair;
      report.discovery_task = discovery.input_task;
      report.discovery_reason = discovery.arithmetic_reason;
      return state.Fail(report);
    }
    diagnostics.Stage(Stage::EventAssembly);
    if (discovery.potential_tasks >
            SIZE_MAX - verified_potential_tasks ||
        discovery.local_masked_tasks >
            SIZE_MAX - verified_local_masked_tasks ||
        discovery.exact_executed_tasks >
            SIZE_MAX - verified_exact_executed_tasks ||
        discovery.feature_candidates >
            SIZE_MAX - verified_feature_observations)
      return state.Fail(Failure(
          S::ResourceLimit,
          "Accepted verification diagnostics overflowed"));
    verified_potential_tasks += discovery.potential_tasks;
    verified_local_masked_tasks += discovery.local_masked_tasks;
    verified_exact_executed_tasks +=
        discovery.exact_executed_tasks;
    const auto chunk_feature_base =
        verified_feature_observations;
    verified_feature_observations +=
        discovery.feature_candidates;
    std::size_t chunk_events = 0;
    events = sct::BuildAcceptedEvents(
        state.active_use, state.regularity, regularity_receipt,
        state.accepted_discovery.features(),
        state.accepted_discovery.intersections(),
        state.buffers.facet_descriptors,
        state.buffers.triangle_order, state.facet_count,
        activity, state.buffers.chunk_events,
        state.buffers.chunk_certificates,
        15 * state.storage_forecast.facet_pair_chunk_capacity,
        &chunk_events);
    if (events.status != S::Ok) {
      if (events.candidate != SIZE_MAX) {
        if (events.candidate >
            SIZE_MAX - chunk_feature_base)
          return state.Fail(Failure(
              S::ResourceLimit,
              "Accepted verification feature ordinal overflowed"));
        events.candidate += chunk_feature_base;
      }
      return state.Fail(events);
    }
    events = sct::VerifyAcceptedEventIdentityChunk(
        state.buffers.chunk_certificates, chunk_events,
        state.buffers.accepted_event_identities, event_count);
    if (events.status != S::Ok) return state.Fail(events);
    events = sct::MergeAcceptedEventChunk(
        state.buffers.chunk_certificates, chunk_events,
        state.buffers.accepted_certificates,
        state.storage_forecast.accepted_event_ledger_capacity,
        state.buffers.accepted_event_hash,
        state.storage_forecast.event_hash_capacity,
        &verified_event_count);
    if (events.status != S::Ok) return state.Fail(events);
    verified_facet_pair_count += streamed_pair_count;
    }
    diagnostics.Stage(Stage::EventAssembly);
    sct::StreamingCandidateSourceReceipt verification_receipt;
    streamed = state.candidate_source.Finish(&verification_receipt);
    if (streamed.status != S::Ok ||
        !state.candidate_source.Authenticates(
            verification_receipt) ||
        verification_receipt.parent_pairs() !=
            state.accepted_broadphase_pair_count ||
        verification_receipt.facet_pairs() !=
            verified_facet_pair_count ||
        verified_facet_pair_count !=
            state.accepted_facet_pair_count ||
        verified_event_count != event_count ||
        verified_feature_observations != feature_observations ||
        verified_potential_tasks != potential_tasks ||
        verified_local_masked_tasks != local_masked_tasks ||
        verified_exact_executed_tasks != exact_executed_tasks)
      return state.Fail(Failure(S::IdentityMismatch,
          "Accepted verification stream differs from its census"));
    if (verified_local_masked_tasks > verified_potential_tasks ||
        verified_exact_executed_tasks !=
            verified_potential_tasks - verified_local_masked_tasks)
      return state.Fail(Failure(S::IdentityMismatch,
          "Accepted verification feature accounting is incomplete"));
  }

  diagnostics.Stage(Stage::EventAssembly);
  events = sct::FinalizeAcceptedEventLedger(
      state.buffers.accepted_certificates, event_count,
      state.buffers.accepted_events,
      state.storage_forecast.accepted_event_capacity);
  if (events.status != S::Ok) return state.Fail(events);
  SelfContactForceAssemblyReceipt force_receipt;
  diagnostics.Stage(Stage::ForceAssembly);
  const auto force = state.force.AssembleAccepted(
      owner, token, view, activity,
      {state.buffers.accepted_events, event_count},
      &force_receipt);
  if (force.status != SelfContactForceStatus::Ok) {
    auto report = Failure(S::ForceFailure, force.message);
    report.force_status = force.status;
    report.owner_status = force.owner_status;
    return state.Fail(report);
  }
  diagnostics.Stage(Stage::Finalization);
  const auto recorded =
      state.participation.RecordSelfContactAcceptedAssembly(
      state.config.source_id, owner, token, view);
  if (recorded.status != fe::ShellPublicationStatus::Success) {
    auto report = Failure(S::PublicationFailure, recorded.message);
    report.publication_status = recorded.status;
    report.owner_status = recorded.nodal_status;
    return state.Fail(report);
  }

  state.accepted_event_count = event_count;
  state.accepted_feature_observation_count =
      feature_observations;
  state.accepted_potential_task_count = potential_tasks;
  state.accepted_local_masked_task_count = local_masked_tasks;
  state.accepted_exact_executed_task_count = exact_executed_tasks;
  state.owner_id = view.owner_id;
  state.base_epoch = view.accepted.base_epoch;
  state.attempt = view.attempt;
  state.stream = view.stream;
  state.phase = Impl::Phase::AssemblyRecorded;

  SelfContactAcceptedAssemblyReceipt next;
  next.transaction_ = this;
  next.owner_ = &owner;
  next.active_use_identity_ = state.active_use.identity();
  next.source_id_ = state.config.source_id;
  next.configuration_id_ = state.config.force.configuration_id;
  next.qualification_id_ = state.config.force.qualification_id;
  next.owner_id_ = state.owner_id;
  next.base_epoch_ = state.base_epoch;
  next.attempt_ = state.attempt;
  next.broadphase_pairs_ =
      state.accepted_broadphase_pair_count;
  next.facet_pairs_ = state.accepted_facet_pair_count;
  next.discovered_features_ =
      state.accepted_feature_observation_count;
  next.potential_tasks_ =
      state.accepted_potential_task_count;
  next.local_masked_tasks_ =
      state.accepted_local_masked_task_count;
  next.exact_executed_tasks_ =
      state.accepted_exact_executed_task_count;
  next.activity_ = activity_receipt;
  next.force_ = force_receipt;
  *output = next;
  diagnostics.Success();
  return {};
}

SelfContactTransactionDiagnostics SelfContactTransaction::diagnostics() const noexcept {
  return impl_ ? impl_->diagnostics : SelfContactTransactionDiagnostics{};
}

void sct::QualificationAccess::SetDiagnosticClock(
    SelfContactTransaction& transaction, sct::DiagnosticClock clock) noexcept {
  if (transaction.impl_) transaction.impl_->diagnostic_clock = clock;
}

void SelfContactTransaction::DiscardTrial() noexcept {
  if (impl_) impl_->DiscardLocal();
}

SelfContactTransactionForecast SelfContactTransaction::forecast()
    const noexcept {
  return impl_ ? impl_->storage_forecast :
      SelfContactTransactionForecast{};
}

SelfContactTransactionAllocationInfo SelfContactTransaction::allocations()
    const noexcept {
  if (!impl_) return {};
  const auto unused = impl_->facet_filters
      ? impl_->facet_filters->UnallocatedDevice() : fe::NodalAllocationInfo{};
  return {impl_->physical_activity.allocations(),
      {impl_->storage_forecast.device_bytes - unused.device_bytes,
       impl_->storage_forecast.device_allocations - unused.device_allocations}};
}

SelfContactCandidatePolicyView
SelfContactTransaction::policy_outcomes() const noexcept {
  if (!impl_ || !impl_->policy_complete ||
      impl_->phase != Impl::Phase::CandidateSealed ||
      !impl_->prepared_activity.valid())
    return {};
  return {impl_->policy_outcome_count
              ? impl_->buffers.policy_outcomes : nullptr,
          impl_->policy_outcome_count, true};
}

SelfContactCandidatePolicySummary
SelfContactTransaction::policy_summary() const noexcept {
  if (!impl_ || !impl_->policy_summary.complete ||
      impl_->phase != Impl::Phase::CandidateSealed ||
      !impl_->prepared_activity.valid())
    return {};
  return impl_->policy_summary;
}

}  // namespace tlfea::contact
