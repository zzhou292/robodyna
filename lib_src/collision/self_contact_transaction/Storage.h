// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SelfContactTransaction.h"
#include "../SelfContactFilterCertificates.h"
#include "../SelfContactPhysicalActivity.h"
#include "../fixed_triangle_features/Geometry.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::self_contact_transaction {

using FacetPrismSeparationAxis =
    SelfContactFacetPrismSeparationAxis;
using FacetPrismAxisLimit =
    SelfContactFacetPrismAxisLimit;
using ::tlfea::contact::CertifiedLinearFacetPrismSeparation;

enum class AcceptedEventCertificateKind : std::uint8_t {
  VertexFace,
  EdgeEdge,
};

struct AcceptedEventCertificate {
  AcceptedEventCertificateKind kind =
      AcceptedEventCertificateKind::VertexFace;
  SelfContactForceEvent event;
  FixedTriangleFeatureCandidate discovery;
  std::uint32_t vertex_facet = UINT32_MAX;
  std::uint32_t target_facet = UINT32_MAX;
  std::uint32_t edge_facet[2]{UINT32_MAX, UINT32_MAX};
};

struct MotionSupport {
  SelfContactFacetMotion motion =
      SelfContactFacetMotion::LinearNodalV1;
  std::uint32_t parent = UINT32_MAX;
  std::uint32_t complete_rigid_group = UINT32_MAX;
  std::uint32_t rigid_groups[4]{
      UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
  std::uint8_t rigid_group_count = 0;
  // Candidate-owner certificate over the represented facet polynomial.
  // Static source classification sets this only for ordinary nodal motion;
  // candidate sealing recomputes it from authenticated rigid snapshots.
  bool certified_affine = false;
};

enum class PairMotionAction : std::uint8_t {
  LinearNodalV1,
  CertifiedLinearSeparation,
  ExcludedSameRigidGroup,
  CertifiedRigidArcSeparation,
  UnsupportedRigidArc,
};

PairMotionAction ClassifyCandidatePairMotion(
    const MotionSupport&, const SelfContactSweptParentBounds&,
    const MotionSupport&, const SelfContactSweptParentBounds&) noexcept;

enum class RigidMemberSweepStatus : std::uint8_t {
  Ok,
  InvalidInput,
  UnsupportedTrajectory,
  RotationLimit,
  NonfiniteResult,
};

// Host value function used by candidate sealing after the owner snapshots and
// NodalPreparedView have independently authenticated identity and timing.
RigidMemberSweepStatus BuildRigidMemberSweepBounds(
    Vec3 accepted_member, Vec3 prepared_member,
    const tl::fea::NodalRigidGroupSnapshot& accepted_group,
    const tl::fea::NodalRigidGroupSnapshot& prepared_group,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, SelfContactSweptParentBounds*) noexcept;

RigidMemberSweepStatus CertifyRigidPointAffineMotion(
    const WeightedSurfacePoint&, VectorView accepted, VectorView prepared,
    const std::uint32_t* node_rigid_groups,
    const tl::fea::NodalRigidGroupSnapshot* accepted_groups,
    const tl::fea::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, bool* affine) noexcept;

RigidMemberSweepStatus CertifyRigidFacetAffineMotion(
    const FixedContactFacet&, VectorView accepted, VectorView prepared,
    const std::uint32_t* node_rigid_groups,
    const tl::fea::NodalRigidGroupSnapshot* accepted_groups,
    const tl::fea::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, bool* affine) noexcept;

struct FacetPairCursor {
  std::uint32_t first_begin = 0;
  std::uint32_t first_end = 0;
  std::uint32_t second_begin = 0;
  std::uint32_t second_end = 0;
  std::uint32_t first = 0;
  std::uint32_t second = 0;
  std::uint32_t parent_pair = UINT32_MAX;
};

class StreamingCandidateSource;

class StreamingCandidateSourceReceipt {
 public:
  StreamingCandidateSourceReceipt() noexcept = default;
  bool complete() const noexcept {
    return source_ != nullptr && identity_ != 0 &&
        emitted_facet_pairs_ == required_facet_pairs_;
  }
  std::size_t parent_pairs() const noexcept { return parent_pairs_; }
  std::size_t facet_pairs() const noexcept {
    return required_facet_pairs_;
  }
  std::size_t chunks() const noexcept { return chunks_; }

 private:
  friend class StreamingCandidateSource;
  const StreamingCandidateSource* source_ = nullptr;
  std::uint64_t identity_ = 0;
  std::size_t parent_pairs_ = 0;
  std::size_t required_facet_pairs_ = 0;
  std::size_t emitted_facet_pairs_ = 0;
  std::size_t chunks_ = 0;
};
struct Layout {
  tl::util::ArenaRegion accepted_positions;
  tl::util::ArenaRegion accepted_velocities;
  tl::util::ArenaRegion prepared_positions;
  tl::util::ArenaRegion prepared_velocities;
  tl::util::ArenaRegion accepted_rigid_groups;
  tl::util::ArenaRegion prepared_rigid_groups;
  tl::util::ArenaRegion node_rigid_groups;
  tl::util::ArenaRegion surface_to_active;
  tl::util::ArenaRegion parent_facet_offsets;
  tl::util::ArenaRegion facet_descriptors;
  tl::util::ArenaRegion parent_motion;
  tl::util::ArenaRegion facet_motion;
  tl::util::ArenaRegion triangle_order;
  tl::util::ArenaRegion vertex_identity_order;
  tl::util::ArenaRegion edge_identity_order;
  tl::util::ArenaRegion accepted_triangles;
  tl::util::ArenaRegion prepared_triangles;
  tl::util::ArenaRegion broadphase_pairs;
  tl::util::ArenaRegion facet_pair_cursors;
  tl::util::ArenaRegion facet_pair_heap;
  tl::util::ArenaRegion facet_pair_chunk;
  tl::util::ArenaRegion chunk_feature_task_masks;
  tl::util::ArenaRegion chunk_paths;
  tl::util::ArenaRegion chunk_represented_pairs;
  tl::util::ArenaRegion chunk_canonical_pairs;
  tl::util::ArenaRegion chunk_raw_canonical_pairs;
  tl::util::ArenaRegion chunk_motion_actions;
  tl::util::ArenaRegion chunk_crossings;
  tl::util::ArenaRegion chunk_validated_outcomes;
  tl::util::ArenaRegion chunk_events;
  tl::util::ArenaRegion chunk_certificates;
  tl::util::ArenaRegion swept_parent_bounds;
  tl::util::ArenaRegion swept_facet_bounds;
  tl::util::ArenaRegion accepted_event_identities;
  tl::util::ArenaRegion accepted_events;
  tl::util::ArenaRegion accepted_certificates;
  tl::util::ArenaRegion accepted_event_hash;
  tl::util::ArenaRegion chunk_policy_outcomes;
  tl::util::ArenaRegion policy_outcomes;
  std::size_t bytes = 0;
};

struct Buffers {
  double* accepted_positions = nullptr;
  double* accepted_velocities = nullptr;
  double* prepared_positions = nullptr;
  double* prepared_velocities = nullptr;
  tl::fea::NodalRigidGroupSnapshot* accepted_rigid_groups = nullptr;
  tl::fea::NodalRigidGroupSnapshot* prepared_rigid_groups = nullptr;
  std::uint32_t* node_rigid_groups = nullptr;
  std::uint32_t* surface_to_active = nullptr;
  std::uint32_t* parent_facet_offsets = nullptr;
  FixedContactFacet* facet_descriptors = nullptr;
  MotionSupport* parent_motion = nullptr;
  MotionSupport* facet_motion = nullptr;
  std::uint32_t* triangle_order = nullptr;
  std::uint32_t* vertex_identity_order = nullptr;
  std::uint32_t* edge_identity_order = nullptr;
  CurrentFixedTriangle* accepted_triangles = nullptr;
  CurrentFixedTriangle* prepared_triangles = nullptr;
  SelfContactPairKey* broadphase_pairs = nullptr;
  FacetPairCursor* facet_pair_cursors = nullptr;
  std::uint32_t* facet_pair_heap = nullptr;
  FixedTrianglePair* facet_pair_chunk = nullptr;
  FixedTriangleFeatureTaskMask* chunk_feature_task_masks = nullptr;
  RepresentedTrianglePath* chunk_paths = nullptr;
  RepresentedTrianglePair* chunk_represented_pairs = nullptr;
  RepresentedIntervalPairKey* chunk_canonical_pairs = nullptr;
  RepresentedIntervalPairKey* chunk_raw_canonical_pairs = nullptr;
  PairMotionAction* chunk_motion_actions = nullptr;
  RepresentedIntervalResult* chunk_crossings = nullptr;
  SelfContactCandidatePolicyOutcome* chunk_validated_outcomes = nullptr;
  SelfContactForceEvent* chunk_events = nullptr;
  AcceptedEventCertificate* chunk_certificates = nullptr;
  SelfContactSweptParentBounds* swept_parent_bounds = nullptr;
  SelfContactSweptParentBounds* swept_facet_bounds = nullptr;
  SelfContactForceEventIdentity* accepted_event_identities = nullptr;
  SelfContactForceEvent* accepted_events = nullptr;
  AcceptedEventCertificate* accepted_certificates = nullptr;
  std::uint32_t* accepted_event_hash = nullptr;
  SelfContactCandidatePolicyOutcome* chunk_policy_outcomes = nullptr;
  SelfContactCandidatePolicyOutcome* policy_outcomes = nullptr;
};

bool MakeLayout(std::size_t nodes, std::size_t surface_parents,
                std::size_t parents, std::size_t facets,
                std::size_t rigid_groups,
                std::size_t broadphase_pair_capacity,
                std::size_t pair_chunk_capacity,
                std::size_t event_capacity,
                std::size_t event_identity_census_capacity,
                std::size_t event_hash_capacity,
                std::size_t policy_outcome_capacity,
                std::size_t max_bytes,
                Layout&) noexcept;
Buffers Bind(void*, const Layout&) noexcept;

class StreamingCandidateSource {
 public:
  SelfContactTransactionReport Initialize(
      const FixedContactFacet*, std::size_t facets,
      FacetPairCursor*, std::size_t cursor_capacity,
      std::uint32_t* heap, std::size_t heap_capacity,
      FixedTrianglePair* chunk, std::size_t chunk_capacity,
      std::size_t complete_pair_capacity) noexcept;
  SelfContactTransactionReport Begin(
      const SelfContactPairKey*, std::size_t,
      const std::uint32_t* surface_to_active,
      std::size_t surface_parents,
      const std::uint32_t* parent_facet_offsets,
      std::size_t parents, SelfContactActivityView) noexcept;
  SelfContactTransactionReport Next(
      const FixedTrianglePair**, std::size_t*) noexcept;
  SelfContactTransactionReport Finish(
      StreamingCandidateSourceReceipt*) noexcept;
  bool Authenticates(
      const StreamingCandidateSourceReceipt&) const noexcept;

 private:
  const FixedContactFacet* descriptors_ = nullptr;
  FacetPairCursor* cursors_ = nullptr;
  std::uint32_t* heap_ = nullptr;
  FixedTrianglePair* chunk_ = nullptr;
  std::size_t facets_ = 0;
  std::size_t cursor_capacity_ = 0;
  std::size_t heap_capacity_ = 0;
  std::size_t chunk_capacity_ = 0;
  std::size_t complete_pair_capacity_ = 0;
  std::size_t heap_count_ = 0;
  std::size_t parent_pair_count_ = 0;
  std::size_t required_facet_pairs_ = 0;
  std::size_t emitted_facet_pairs_ = 0;
  std::size_t chunks_ = 0;
  std::uint64_t identity_ = 0;
  FixedTrianglePair previous_;
  bool have_previous_ = false;
  bool initialized_ = false;
  bool active_ = false;
};

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
    SelfContactActivityView,
    FixedTrianglePair*, std::size_t capacity,
    std::size_t* output_count) noexcept;

SelfContactTransactionReport InitializeStaticPipeline(
    const SelfContactActiveUseBinding&, Buffers,
    std::size_t nodes, std::size_t surface_parents,
    std::size_t facets) noexcept;
SelfContactTransactionReport FilterAcceptedFacetPairs(
    const SelfContactActiveUseBinding&,
    const CurrentFixedTriangle*, const MotionSupport*,
    std::size_t facets,
    FixedTrianglePair*, std::size_t* pair_count) noexcept;
SelfContactTransactionReport BuildLocalFeatureTaskMasks(
    const FixedContactFacet*, std::size_t facets,
    const FixedTrianglePair*, std::size_t pair_count,
    FixedTriangleFeatureTaskMask*, std::size_t mask_capacity) noexcept;
SelfContactTransactionReport EvaluateCompleteTriangles(
    const FixedContactFacet*, std::size_t, VectorView,
    CurrentFixedTriangle*) noexcept;
SelfContactTransactionReport ValidateCompleteTriangleIdentities(
    const CurrentFixedTriangle*, std::size_t,
    const std::uint32_t* vertex_order,
    const std::uint32_t* edge_order) noexcept;
SelfContactTransactionReport ReadBroadphase(
    const SelfContactBroadphase&, cudaStream_t,
    SelfContactPairKey*, std::size_t broadphase_capacity,
    std::size_t* broadphase_count) noexcept;
bool CompleteRegularity(
    const SelfContactActiveUseBinding&,
    const SelfContactCurrentRegularityReceipt&,
    SelfContactCurrentRegularityView,
    SelfContactActivityView) noexcept;
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
    std::size_t facet_count, SelfContactActivityView,
    const AcceptedEventCertificate*, std::size_t) noexcept;
bool ExactFacetPair(const FixedTriangleFeatureCandidate&,
                    const FixedTriangleFeatureCandidate&) noexcept;
SelfContactTransactionReport MergeAcceptedEventIdentityChunk(
    const SelfContactForceEventIdentity*, std::size_t,
    SelfContactForceEventIdentity*, std::size_t census_capacity,
    std::uint32_t*, std::size_t hash_capacity,
    std::size_t*) noexcept;
SelfContactTransactionReport CanonicalizeAcceptedEventIdentityCensus(
    SelfContactForceEventIdentity*, std::size_t) noexcept;
SelfContactTransactionReport VerifyAcceptedEventIdentityChunk(
    const AcceptedEventCertificate*, std::size_t,
    const SelfContactForceEventIdentity*, std::size_t) noexcept;
SelfContactTransactionReport MergeAcceptedEventChunk(
    const AcceptedEventCertificate*, std::size_t,
    AcceptedEventCertificate*, std::size_t ledger_capacity,
    std::uint32_t*, std::size_t hash_capacity,
    std::size_t*) noexcept;
SelfContactTransactionReport FinalizeAcceptedEventLedger(
    AcceptedEventCertificate*, std::size_t,
    SelfContactForceEvent*, std::size_t force_capacity) noexcept;
void FoldPolicyOutcomes(
    const SelfContactCandidatePolicyOutcome*, std::size_t,
    SelfContactCandidatePolicySummary*) noexcept;

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
  self_contact_transaction::StreamingCandidateSource candidate_source;
  SelfContactPhysicalActivity physical_activity;
  SelfContactBroadphase broadphase;
  FixedTriangleFeatureDiscovery accepted_discovery;
  FixedTriangleFeatureDiscovery candidate_discovery;
  SelfContactCurrentRegularity regularity;
  RepresentedIntervalCrossing crossing;
  SelfContactForceAssembly force;
  tl::fea::ShellPhysicalScratchParticipation participation;
  SelfContactPreparedActivityReceipt prepared_activity;
  std::size_t surface_parent_count = 0;
  std::size_t facet_count = 0;
  std::size_t rigid_group_count = 0;
  std::size_t accepted_broadphase_pair_count = 0;
  std::size_t candidate_broadphase_pair_count = 0;
  std::size_t accepted_facet_pair_count = 0;
  std::size_t candidate_facet_pair_count = 0;
  std::size_t accepted_event_count = 0;
  std::size_t accepted_feature_observation_count = 0;
  std::size_t accepted_potential_task_count = 0;
  std::size_t accepted_local_masked_task_count = 0;
  std::size_t accepted_exact_executed_task_count = 0;
  std::size_t policy_outcome_count = 0;
  SelfContactCandidatePolicySummary policy_summary;
  bool policy_complete = false;
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
