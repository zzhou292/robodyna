// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SelfContactTransaction.h"
#include "../fixed_triangle_features/Geometry.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::self_contact_transaction {

struct AcceptedEventCertificate {
  SelfContactForceEvent event;
  FixedTriangleFeatureCandidate discovery;
  std::uint32_t vertex_facet = UINT32_MAX;
  std::uint32_t target_facet = UINT32_MAX;
};

struct Layout {
  tl::util::ArenaRegion accepted_positions;
  tl::util::ArenaRegion accepted_velocities;
  tl::util::ArenaRegion prepared_positions;
  tl::util::ArenaRegion prepared_velocities;
  tl::util::ArenaRegion activity_base;
  tl::util::ArenaRegion activity_current;
  tl::util::ArenaRegion surface_to_active;
  tl::util::ArenaRegion parent_facet_offsets;
  tl::util::ArenaRegion facet_descriptors;
  tl::util::ArenaRegion triangle_order;
  tl::util::ArenaRegion accepted_triangles;
  tl::util::ArenaRegion prepared_triangles;
  tl::util::ArenaRegion represented_paths;
  tl::util::ArenaRegion accepted_broadphase_pairs;
  tl::util::ArenaRegion candidate_broadphase_pairs;
  tl::util::ArenaRegion accepted_facet_pairs;
  tl::util::ArenaRegion candidate_facet_pairs;
  tl::util::ArenaRegion represented_pairs;
  tl::util::ArenaRegion canonical_pairs;
  tl::util::ArenaRegion accepted_events;
  tl::util::ArenaRegion accepted_certificates;
  tl::util::ArenaRegion policy_outcomes;
  std::size_t bytes = 0;
};

struct Buffers {
  double* accepted_positions = nullptr;
  double* accepted_velocities = nullptr;
  double* prepared_positions = nullptr;
  double* prepared_velocities = nullptr;
  std::uint8_t* activity_base = nullptr;
  std::uint8_t* activity_current = nullptr;
  std::uint32_t* surface_to_active = nullptr;
  std::uint32_t* parent_facet_offsets = nullptr;
  FixedContactFacet* facet_descriptors = nullptr;
  std::uint32_t* triangle_order = nullptr;
  CurrentFixedTriangle* accepted_triangles = nullptr;
  CurrentFixedTriangle* prepared_triangles = nullptr;
  RepresentedTrianglePath* represented_paths = nullptr;
  SelfContactPairKey* accepted_broadphase_pairs = nullptr;
  SelfContactPairKey* candidate_broadphase_pairs = nullptr;
  FixedTrianglePair* accepted_facet_pairs = nullptr;
  FixedTrianglePair* candidate_facet_pairs = nullptr;
  RepresentedTrianglePair* represented_pairs = nullptr;
  RepresentedIntervalPairKey* canonical_pairs = nullptr;
  SelfContactForceEvent* accepted_events = nullptr;
  AcceptedEventCertificate* accepted_certificates = nullptr;
  SelfContactCandidatePolicyOutcome* policy_outcomes = nullptr;
};

bool MakeLayout(std::size_t nodes, std::size_t surface_parents,
                std::size_t parents, std::size_t facets,
                std::size_t broadphase_pair_capacity,
                std::size_t pair_capacity,
                std::size_t event_capacity, std::size_t max_bytes,
                Layout&) noexcept;
Buffers Bind(void*, const Layout&) noexcept;

int Compare(const RepresentedTrianglePathKey&,
            const RepresentedTrianglePathKey&) noexcept;
int Compare(const RepresentedIntervalPairKey&,
            const RepresentedIntervalPairKey&) noexcept;
bool Same(const FixedTriangleKey&, const FixedTriangleKey&) noexcept;
bool Same(const FixedTriangleIntersection&,
          const FixedTriangleIntersection&) noexcept;
bool Same(const FixedTriangleFeatureKey&,
          const FixedTriangleFeatureKey&) noexcept;

struct CandidateValidationInput {
  const RepresentedIntervalPairKey* canonical_pairs = nullptr;
  std::size_t pair_count = 0;
  FixedTriangleFeatureView features;
  FixedTriangleIntersectionView intersections;
  RepresentedIntervalResultView crossings;
  const AcceptedEventCertificate* accepted_events = nullptr;
  std::size_t accepted_event_count = 0;
  SelfContactCandidatePolicyOutcome* outcomes = nullptr;
  std::size_t outcome_capacity = 0;
  std::size_t* outcome_count = nullptr;
};

SelfContactTransactionReport ValidateCandidatePublications(
    const CandidateValidationInput&) noexcept;

SelfContactTransactionReport ExpandFacetPairs(
    const SelfContactPairKey*, std::size_t,
    const std::uint32_t* surface_to_active, std::size_t surface_parents,
    const std::uint32_t* parent_facet_offsets, std::size_t parents,
    FixedTrianglePair*, std::size_t capacity,
    std::size_t* output_count) noexcept;

SelfContactTransactionReport InitializeStaticPipeline(
    const SelfContactActiveUseBinding&, Buffers,
    std::size_t surface_parents, std::size_t facets,
    bool* has_rigid_motion) noexcept;
SelfContactTransactionReport EvaluateCompleteTriangles(
    const FixedContactFacet*, std::size_t, VectorView,
    CurrentFixedTriangle*) noexcept;
SelfContactTransactionReport ReadAndExpandBroadphase(
    const SelfContactBroadphase&, cudaStream_t,
    SelfContactPairKey*, std::size_t broadphase_capacity,
    const std::uint32_t* surface_to_active,
    std::size_t surface_parents,
    const std::uint32_t* parent_facet_offsets,
    std::size_t parents, FixedTrianglePair*,
    std::size_t facet_pair_capacity,
    std::size_t* broadphase_count,
    std::size_t* facet_pair_count) noexcept;
SelfContactTransactionReport BuildAcceptedEvents(
    const SelfContactActiveUseBinding&,
    const SelfContactCurrentRegularity&,
    const SelfContactCurrentRegularityReceipt&,
    FixedTriangleFeatureView, FixedTriangleIntersectionView,
    const FixedContactFacet*, const std::uint32_t* triangle_order,
    std::size_t facet_count, SelfContactActivityView,
    SelfContactForceEvent*, AcceptedEventCertificate*,
    std::size_t capacity, std::size_t* count) noexcept;
SelfContactTransactionReport ValidateCandidateEdgePolicy(
    const SelfContactActiveUseBinding&,
    const SelfContactCurrentRegularity&,
    const SelfContactCurrentRegularityReceipt&,
    FixedTriangleFeatureView,
    const FixedContactFacet*, const std::uint32_t* triangle_order,
    std::size_t facet_count, SelfContactActivityView) noexcept;

}  // namespace tlfea::contact::self_contact_transaction

namespace tlfea::contact {

struct SelfContactTransaction::Impl {
  enum class Phase : std::uint8_t {
    Idle,
    AssemblyRecorded,
    CandidateSealed,
  };

  explicit Impl(const SelfContactActiveUseBinding& source)
      : active_use(source) {}

  SelfContactActiveUseBinding active_use;
  tl::fea::FENodalState* owner = nullptr;
  tl::fea::ShellBatchPublication* publication = nullptr;
  SelfContactTransactionConfig config;
  SelfContactTransactionForecast storage_forecast;
  self_contact_transaction::Layout layout;
  tl::util::HostArena arena;
  self_contact_transaction::Buffers buffers;
  SelfContactBroadphase broadphase;
  FixedTriangleFeatureDiscovery accepted_discovery;
  FixedTriangleFeatureDiscovery candidate_discovery;
  SelfContactCurrentRegularity regularity;
  RepresentedIntervalCrossing crossing;
  SelfContactForceAssembly force;
  tl::fea::ShellPhysicalScratchParticipation participation;
  std::size_t surface_parent_count = 0;
  std::size_t facet_count = 0;
  std::size_t accepted_broadphase_pair_count = 0;
  std::size_t candidate_broadphase_pair_count = 0;
  std::size_t accepted_facet_pair_count = 0;
  std::size_t candidate_facet_pair_count = 0;
  std::size_t accepted_event_count = 0;
  std::size_t policy_outcome_count = 0;
  bool policy_complete = false;
  bool has_rigid_motion = false;
  std::uint64_t owner_id = 0;
  std::uint64_t base_epoch = 0;
  std::uint64_t attempt = 0;
  cudaStream_t owner_stream = nullptr;
  cudaStream_t stream = nullptr;
  Phase phase = Phase::Idle;

  bool OutputDisjoint(const void*, std::size_t) const noexcept;
  void DiscardLocal() noexcept;
  SelfContactTransactionReport Fail(
      SelfContactTransactionReport) noexcept;
};

}  // namespace tlfea::contact
