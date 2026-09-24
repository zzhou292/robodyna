// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SelfContactTransaction.h"
#include "../SelfContactFilterCertificates.h"
#include "../SelfContactPhysicalActivity.h"
#include "../fixed_triangle_features/Geometry.h"
#include "QualificationReceipt.h"
#include "Diagnostics.h"
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

struct AcceptedEventCertificateView {
  const AcceptedEventCertificate* data = nullptr;
  std::size_t count = 0;
  bool complete = false;
};

enum class AcceptedFeatureDisposition : std::uint8_t {
  AdmittedLedgerCandidate,
  DistanceRepresentationSeparated,
  InactiveParent,
  ExcludedSameRigidSupport,
  ExcludedLocalOrRegularOwnParent,
  TiedOrCinPolicy,
  UnsupportedForceArea,
  FeatureWeightNormalization,
  InvalidProvenance,
};

// Stable qualification record for one exact accepted endpoint feature. It
// intentionally omits live pointer identities while retaining every value
// that selected BuildAcceptedEvents' branch and the complete final-ledger
// owner-search counts for the same canonical feature key.
struct AcceptedFeaturePolicyEvidence {
  AcceptedFeatureDisposition disposition =
      AcceptedFeatureDisposition::InvalidProvenance;
  SelfContactTransactionStatus report_status =
      SelfContactTransactionStatus::Ok;
  SelfContactPairKind kind = SelfContactPairKind::VertexFace;
  SelfContactEdgeEdgeCase edge_edge_case =
      SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum;
  SelfContactPairStatus pair_status =
      SelfContactPairStatus::InactiveParent;
  SelfContactTiedStatus tied = SelfContactTiedStatus::NotRelated;
  SelfContactSupportClassification endpoint_support[2];
  std::uint32_t parent[2]{UINT32_MAX, UINT32_MAX};
  std::uint32_t feature[2]{UINT32_MAX, UINT32_MAX};
  bool classification_complete = false;
  bool active[2]{};
  bool local_incidence = false;
  bool excluded = false;
  bool distance_separated = false;
  bool admitted = false;
  double reference_half_thickness_m[2]{};
  Q4CertifiedIntegral candidate_directed_area_m2;
  Q4CertifiedIntegral admitted_force_area_m2;
  std::size_t ledger_key_matches = 0;
  std::size_t ledger_exact_pair_matches = 0;
  std::size_t ledger_canonical_lower_matches = 0;
  std::size_t ledger_foreign_owner_matches = 0;
  std::uint64_t first_ledger_source_order = UINT64_MAX;
  FixedTriangleKey first_ledger_triangles[2];
  std::uint32_t first_ledger_parent[2]{UINT32_MAX, UINT32_MAX};
};

struct PreparedMotionCertificateView;
struct NonlinearSeparationResult;
struct NonlinearCandidateRosterEntry;
struct NonlinearCandidateRosterSummary;
struct LinearWorkExhaustedRosterEntry;
struct LinearCandidateCensusSummary;
struct CandidateFailureObserver;

// Internal qualification-only read access. This never supplies authority to
// production policy and is available only through this private storage header.
class QualificationAccess {
 public:
  // Observation-only qualification seam. Does not touch mechanics authority.
  static void SetDiagnosticClock(SelfContactTransaction&, DiagnosticClock) noexcept;
  // Same production sealing path. Only a proved failing source pair is
  // observed, synchronously before the ordinary complete rollback.
  static SelfContactTransactionReport SealCandidateWithFailureObserver(
      SelfContactTransaction&, tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::ShellPhysicalDiagnostics&,
      const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedAssemblyReceipt&,
      SelfContactTransactionReceipt*, const CandidateFailureObserver&);
  static AcceptedEventCertificateView AcceptedCertificates(
      const SelfContactTransaction&) noexcept;
  static SelfContactTransactionReport
  ClassifyAcceptedFeaturePolicies(
      SelfContactTransaction&,
      const SelfContactAcceptedAssemblyReceipt&,
      FixedTriangleFeatureView,
      AcceptedFeaturePolicyEvidence*, std::size_t capacity,
      std::size_t* count) noexcept;
  // Replays accepted endpoint policy from the preserved base of an actual
  // prepared census. The ordinary accepted receipt has been consumed.
  static SelfContactTransactionReport
  ClassifyAcceptedFeaturePolicies(
      SelfContactTransaction&,
      const QualificationPreparedCensusReceipt&,
      FixedTriangleFeatureView,
      AcceptedFeaturePolicyEvidence*, std::size_t capacity,
      std::size_t* count) noexcept;
  // Replays only authenticated motion construction, conservative broadphase
  // and nonlinear subdivision for a live accepted-assembly attempt. Ordinary
  // linear feature discovery and represented crossing are deliberately not
  // entered. Returned views borrow transaction scratch until candidate
  // sealing, discard, failure or destruction.
  static SelfContactTransactionReport
  ClassifyPreparedNonlinearCandidates(
      SelfContactTransaction&, tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedAssemblyReceipt&,
      NonlinearCandidateRosterEntry*, std::size_t roster_capacity,
      std::size_t* roster_count,
      NonlinearCandidateRosterSummary*,
      PreparedMotionCertificateView*) noexcept;
  // Extends the same no-force authenticated source pass with the complete
  // affine frontier.  It exactly mirrors the production prism,
  // residual/persistent and represented-interval stages, but publishes only
  // pairs whose production 4095-work traversal returns WorkExhausted.
  static SelfContactTransactionReport
  ClassifyPreparedCandidateCensus(
      SelfContactTransaction&, tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedAssemblyReceipt&,
      NonlinearCandidateRosterEntry*, std::size_t nonlinear_capacity,
      std::size_t* nonlinear_count,
      NonlinearCandidateRosterSummary*,
      LinearWorkExhaustedRosterEntry*, std::size_t linear_capacity,
      std::size_t* linear_count,
      LinearCandidateCensusSummary*,
      PreparedMotionCertificateView*) noexcept;
  // Actual structural-candidate variant: authenticates common publication
  // diagnostics, removal/activity and current regularity before traversing.
  // Consumes accepted activity authority; the diagnostic candidate must be
  // discarded after capture and cannot subsequently enter SealCandidate.
  static SelfContactTransactionReport
  ClassifyPreparedCandidateCensus(
      SelfContactTransaction&, tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::ShellPhysicalDiagnostics&,
      const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedAssemblyReceipt&,
      NonlinearCandidateRosterEntry*, std::size_t nonlinear_capacity,
      std::size_t* nonlinear_count,
      NonlinearCandidateRosterSummary*,
      LinearWorkExhaustedRosterEntry*, std::size_t linear_capacity,
      std::size_t* linear_count,
      LinearCandidateCensusSummary*,
      PreparedMotionCertificateView*,
      QualificationPreparedCensusReceipt*) noexcept;

 private:
  friend class ::tlfea::contact::SelfContactTransaction;
  static void ObserveCandidateFailure(
      SelfContactTransaction&, const CandidateFailureObserver*,
      const SelfContactTransactionReport&, FixedTrianglePair,
      const tl::fea::NodalStamp&, const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedAssemblyReceipt&,
      const SelfContactPreparedActivityReceipt&,
      const NonlinearSeparationResult* nonlinear_baseline = nullptr) noexcept;
  static void ObserveCandidateFeatureFailure(
      SelfContactTransaction&, const CandidateFailureObserver*,
      const SelfContactTransactionReport&,
      const tl::fea::NodalStamp&, const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedAssemblyReceipt&,
      const SelfContactPreparedActivityReceipt&) noexcept;
  static SelfContactTransactionReport
  ClassifyAcceptedFeaturePoliciesImpl(
      SelfContactTransaction&,
      const SelfContactAcceptedAssemblyReceipt&,
      const QualificationPreparedCensusReceipt*,
      FixedTriangleFeatureView,
      AcceptedFeaturePolicyEvidence*, std::size_t,
      std::size_t*) noexcept;
  static SelfContactTransactionReport
  ClassifyPreparedCandidateCensusImpl(
      SelfContactTransaction&, tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedAssemblyReceipt&,
      NonlinearCandidateRosterEntry*, std::size_t,
      std::size_t*, NonlinearCandidateRosterSummary*,
      LinearWorkExhaustedRosterEntry*, std::size_t,
      std::size_t*, LinearCandidateCensusSummary*,
      PreparedMotionCertificateView*,
      const tl::fea::ShellPhysicalDiagnostics*,
      QualificationPreparedCensusReceipt*) noexcept;
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

struct DirectedInterval {
  double lower = 0;
  double upper = 0;
};

// Exact represented dyadic q, rounded once to an outward binary64 interval
// for each vertex coordinate. Ordinary contributors add exact zero.
struct FacetQuadraticCoefficients {
  DirectedInterval q[3][3];
  bool complete = false;
};

enum class NonlinearSeparationStatus : std::uint8_t {
  CertifiedSeparated = 0,
  PotentialContact = 1,
  WorkExhausted = 2,
  DepthExhausted = 3,
  InvalidInput = 4,
  CertifiedAcceptedCoverage = 5,
  MissingAcceptedOwner = 6,
  OwnerAmbiguity = 7,
  PossibleGeometricCrossing = 8,
  CertifiedExactExclusion = 9,
  CertifiedLocalIntersection = 10,
};

struct NonlinearSeparationResult {
  NonlinearSeparationStatus status =
      NonlinearSeparationStatus::InvalidInput;
  std::size_t work = 0;
  unsigned deepest = 0;
  std::size_t separated_cells = 0;
  std::size_t covered_cells = 0;
  std::size_t closed_covered_cells = 0;
  std::size_t accepted_certificate = SIZE_MAX;
  std::uint64_t accepted_source_order = UINT64_MAX;
  RepresentedFeaturePathKey feature;
  // A real midsurface intersection proved by the geometry certificate,
  // independent of the accepted physical-contact owner above.  The feature
  // is canonical in the represented pair; time is the exact dyadic
  // numerator / 2^depth.  It remains absent for separation-only proofs.
  RepresentedFeaturePathKey intersection_feature;
  std::uint64_t intersection_time_numerator = 0;
  unsigned intersection_time_depth = 0;
  bool has_intersection = false;
  std::uint64_t proof_digest = 1469598103934665603ull;
  bool work_exhausted = false;
  bool depth_exhausted = false;
  // Canonical dyadic cell that prevented a conclusive proof.  The interval
  // is [unresolved_path/2^unresolved_depth,
  //     (unresolved_path+1)/2^unresolved_depth].
  std::uint64_t unresolved_path = 0;
  unsigned unresolved_depth = 0;
  bool has_unresolved_cell = false;
  RepresentedFeaturePathKey transition_feature;
  std::uint64_t transition_time_lower_numerator = 0;
  unsigned transition_time_depth = 0;
  bool transition_time_exact = false;
  bool transition_zero_geometry_separated = false;
  bool has_contact_transition = false;
  std::uint32_t excluded_rigid_group = UINT32_MAX;
};

struct AcceptedFeatureExclusionCertificate {
  FixedTriangleFeatureCandidate feature;
  std::uint32_t complete_rigid_group = UINT32_MAX;
};

struct PreparedMotionCertificateView {
  const FixedContactFacet* descriptors = nullptr;
  const MotionSupport* motion = nullptr;
  const FacetQuadraticCoefficients* quadratic = nullptr;
  const CurrentFixedTriangle* accepted_triangles = nullptr;
  const CurrentFixedTriangle* prepared_triangles = nullptr;
  const SelfContactSweptParentBounds* swept_bounds = nullptr;
  std::size_t facet_count = 0;
  bool complete = false;
};

struct NonlinearCandidateRosterEntry {
  FixedTrianglePair facets;
  RepresentedIntervalPairKey key;
  NonlinearSeparationResult separation;
};

struct NonlinearCandidateRosterSummary {
  std::size_t broadphase_parent_pairs = 0;
  std::size_t streamed_facet_pairs = 0;
  std::size_t rigid_or_mixed_facets = 0;
  std::size_t affine_rigid_or_mixed_facets = 0;
  std::size_t nonlinear_pairs = 0;
  std::size_t certified_separated = 0;
  std::size_t unresolved = 0;
  std::size_t potential_contact = 0;
  std::size_t work_exhausted = 0;
  std::size_t depth_exhausted = 0;
  std::size_t work = 0;
  bool complete = false;
  bool roster_complete = false;
};

enum class PairMotionAction : std::uint8_t {
  LinearNodalV1,
  CertifiedLinearSeparation,
  ExcludedSameRigidGroup,
  CertifiedRigidArcSeparation,
  UnsupportedRigidArc,
  CertifiedResidualLinearSeparation,
  CertifiedPersistentLinearContact,
  CertifiedQuadraticResidualSeparation,
  CertifiedPersistentQuadraticContact,
  CertifiedQuadraticAcceptedCoverage,
  CertifiedQuadraticExactExclusion,
  CertifiedQuadraticLocalIntersection,
};

PairMotionAction ClassifyCandidatePairMotion(
    const MotionSupport&, const SelfContactSweptParentBounds&,
    const MotionSupport&, const SelfContactSweptParentBounds&) noexcept;

enum class LinearResidualSeparationStatus : std::uint8_t {
  CertifiedSeparated,
  PotentialContact,
  IncompleteFeatureRoster,
  InvalidInput,
};

// The reference is one exactly represented binary64 translation.  Residual
// bounds are exact L1 upper bounds on each triangle's Hausdorff motion after
// removing that reference.  The certificate requires the complete prepared
// six-VF/nine-EE feature roster and compares exact represented-point dyadic
// distances against representation error, both physical half-thicknesses and
// both residual bounds.  Diagnostics are outward binary64 bounds only; the
// decision itself is made with exact dyadic arithmetic.
// For every prepared feature i it requires, strictly,
//   d_i > representation_error_i + h0 + h1 + H0 + H1,
// where Hk=max_vertex ||(x1-x0)-reference||_1.  Convex interpolation then
// bounds each translated triangle's whole-interval Hausdorff motion by Hk.
struct LinearResidualSeparationResult {
  LinearResidualSeparationStatus status =
      LinearResidualSeparationStatus::InvalidInput;
  Vec3 reference_translation;
  double first_residual_upper_m = 0;
  double second_residual_upper_m = 0;
  double prepared_distance_lower_m = 0;
  double strict_gap_lower_m = 0;
  bool exact_common_translation = false;
};

LinearResidualSeparationResult CertifyLinearResidualSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    double first_half_thickness_m,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    double second_half_thickness_m,
    FixedTriangleFeatureView prepared_features,
    FixedTriangleIntersectionView prepared_intersections) noexcept;

// Extends the exact linear residual proof by the outward Bernstein deviation
// of each authenticated quadratic facet from its endpoint chord. The linear
// strict-gap lower bound must exceed both curvature upper bounds.
LinearResidualSeparationResult CertifyQuadraticResidualSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients& first_quadratic,
    double first_half_thickness_m,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients& second_quadratic,
    double second_half_thickness_m, double duration,
    FixedTriangleFeatureView prepared_features,
    FixedTriangleIntersectionView prepared_intersections) noexcept;

enum class PersistentLinearContactStatus : std::uint8_t {
  CertifiedContact,
  PotentialChange,
  IncompleteFeatureRoster,
  InvalidInput,
};

// Tracks one immutable material VF or EE feature using its prepared
// barycentric/edge parameters. After removing the exact binary64 reference
// translation, H0+H1 bounds its whole-interval relative motion. Certification
// requires an exact accepted-ledger feature identity. EE producing facets must
// either match exactly or normalize componentwise to the ledger's lower
// canonical seam owners. VF permits only a lower canonical source-vertex
// owner while retaining the exact target face. Crossed, target-face, or
// noncanonical provenance is ambiguous.
// Strictly, d_i + representation_error_i + H0 + H1 < h0 + h1.
// This proves thickness persistence only. It cannot replace continuous
// triangle-intersection validation, even for nonlocal pairs.
struct PersistentLinearContactResult {
  PersistentLinearContactStatus status =
      PersistentLinearContactStatus::InvalidInput;
  RepresentedFeaturePathKey feature;
  Vec3 reference_translation;
  double first_residual_upper_m = 0;
  double second_residual_upper_m = 0;
  double prepared_distance_upper_m = 0;
  double face_weight_normalization_upper_m = 0;
  double strict_thickness_margin_lower_m = 0;
  std::size_t accepted_certificate = SIZE_MAX;
  std::size_t bounded_feature_count = 0;
  std::size_t exact_accepted_candidate_count = 0;
  std::size_t full_accepted_candidate_count = 0;
  bool exact_common_translation = false;
};

PersistentLinearContactResult CertifyPersistentLinearContact(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    double first_half_thickness_m,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    double second_half_thickness_m,
    FixedTriangleFeatureView prepared_features,
    const AcceptedEventCertificate* accepted_certificates,
    std::size_t accepted_certificate_count) noexcept;

// Accepted-ledger identity and seam ownership are unchanged. Certification
// additionally subtracts both outward quadratic chord-deviation bounds from
// the existing exact linear persistence margin.
PersistentLinearContactResult CertifyPersistentQuadraticContact(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients& first_quadratic,
    double first_half_thickness_m,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients& second_quadratic,
    double second_half_thickness_m, double duration,
    FixedTriangleFeatureView prepared_features,
    const AcceptedEventCertificate* accepted_certificates,
    std::size_t accepted_certificate_count) noexcept;

struct LinearWorkExhaustedRosterEntry {
  FixedTrianglePair facets;
  RepresentedIntervalPairKey key;
  FixedTriangleFeatureTaskMask task_mask;
  LinearResidualSeparationResult residual;
  PersistentLinearContactResult persistent;
  RepresentedIntervalResult crossing;
};

struct LinearCandidateCensusSummary {
  std::size_t affine_pairs = 0;
  std::size_t swept_bounds_separated = 0;
  std::size_t prism_separated = 0;
  std::size_t exact_geometry_pairs = 0;
  std::size_t common_translation = 0;
  std::size_t residual_separated = 0;
  std::size_t persistent_accepted = 0;
  std::size_t represented_pairs = 0;
  std::size_t represented_separated = 0;
  std::size_t represented_crossing = 0;
  std::size_t represented_degenerate = 0;
  std::size_t represented_work_exhausted = 0;
  std::size_t represented_arithmetic_range = 0;
  std::size_t represented_work = 0;
  bool complete = false;
  bool roster_complete = false;
};

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

RigidMemberSweepStatus BuildRigidFacetQuadraticCoefficients(
    const FixedContactFacet&, VectorView accepted, VectorView prepared,
    const std::uint32_t* node_rigid_groups,
    const tl::fea::NodalRigidGroupSnapshot* accepted_groups,
    const tl::fea::NodalRigidGroupSnapshot* prepared_groups,
    std::size_t group_count,
    tl::fea::NodalRigidMemberTrajectory trajectory,
    double duration, FacetQuadraticCoefficients*, bool* affine) noexcept;

NonlinearSeparationResult CertifyQuadraticFacetSeparation(
    const CurrentFixedTriangle& first_accepted,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients&, double first_thickness,
    const CurrentFixedTriangle& second_accepted,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients&, double second_thickness,
    double duration, std::size_t max_work,
    unsigned max_depth) noexcept;

// Covers one complete quadratic time interval by a deterministic dyadic tree.
// Every accepted leaf has either a strict thick-facet separating axis, or a
// strict zero-thickness separating axis plus one exact accepted-ledger VF/EE
// material feature whose represented distance remains strictly inside the
// summed half-thickness. Canonical seam changes are admitted only through
// independently valid accepted identities. Any missing/ambiguous owner,
// possible geometric crossing, arithmetic failure, or finite cap exhaustion
// remains an explicit fail-closed status.
NonlinearSeparationResult CertifyQuadraticFacetCoverage(
    const CurrentFixedTriangle& first_accepted,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients&, double first_thickness,
    const CurrentFixedTriangle& second_accepted,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients&, double second_thickness,
    double duration,
    const AcceptedEventCertificate*, std::size_t accepted_count,
    std::size_t max_work, unsigned max_depth) noexcept;

struct PolicyExclusionSource;

// Eager certificates and deferred source are mutually exclusive. A deferred
// provider runs at most once, after local/ledger failure and only if work
// remains; its exact failure report must be inspected by transaction callers.
NonlinearSeparationResult CertifyQuadraticFacetPolicyCoverage(
    const CurrentFixedTriangle& first_accepted,
    const CurrentFixedTriangle& first_prepared,
    const FacetQuadraticCoefficients&, double first_thickness,
    const CurrentFixedTriangle& second_accepted,
    const CurrentFixedTriangle& second_prepared,
    const FacetQuadraticCoefficients&, double second_thickness,
    double duration,
    const AcceptedEventCertificate*, std::size_t accepted_count,
    const AcceptedFeatureExclusionCertificate*,
    std::size_t exclusion_count,
    std::size_t max_work, unsigned max_depth,
    PolicyExclusionSource* deferred_exclusions = nullptr) noexcept;

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
  tl::util::ArenaRegion facet_quadratic;
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
  tl::util::ArenaRegion chunk_nonlinear_results;
  tl::util::ArenaRegion chunk_crossings;
  tl::util::ArenaRegion chunk_raw_crossings;
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
  FacetQuadraticCoefficients* facet_quadratic = nullptr;
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
  NonlinearSeparationResult* chunk_nonlinear_results = nullptr;
  RepresentedIntervalResult* chunk_crossings = nullptr;
  RepresentedIntervalResult* chunk_raw_crossings = nullptr;
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

class SortedIntersections;

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
SelfContactTransactionReport ValidateCandidatePublications(
    const CandidateValidationInput&, const SortedIntersections&) noexcept;

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
SelfContactTransactionReport BuildAcceptedSameRigidExclusions(
    const SelfContactActiveUseBinding&,
    FixedTriangleFeatureView,
    const FixedContactFacet*, const std::uint32_t* triangle_order,
    std::size_t facet_count, SelfContactActivityView,
    AcceptedFeatureExclusionCertificate*,
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
  SelfContactTransactionDiagnostics diagnostics;
  self_contact_transaction::DiagnosticClock diagnostic_clock;
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

namespace tlfea::contact::self_contact_transaction {

inline AcceptedEventCertificateView
QualificationAccess::AcceptedCertificates(
    const SelfContactTransaction& owner) noexcept {
  if (!owner.impl_ ||
      owner.impl_->phase !=
          SelfContactTransaction::Impl::Phase::AssemblyRecorded)
    return {};
  return {owner.impl_->buffers.accepted_certificates,
          owner.impl_->accepted_event_count, true};
}

}  // namespace tlfea::contact::self_contact_transaction
