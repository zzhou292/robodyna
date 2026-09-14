// SPDX-License-Identifier: MIT
#include "lib_src/collision/self_contact_transaction/Storage.h"

#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <iostream>
#include <type_traits>
#include <vector>

namespace {
namespace c = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;

c::RepresentedTrianglePathKey Path(std::uint64_t eid,
                                   unsigned facet = 0) {
  return {17, eid, 0, facet};
}

c::RepresentedIntervalPairKey Pair(std::uint64_t first,
                                   std::uint64_t second) {
  return {{Path(first), Path(second)}};
}

c::FacetVertexKey Vertex(std::uint64_t id) {
  c::FacetVertexKey result;
  result.source_instance_id = 17;
  result.first = id;
  result.denominator = 1;
  return result;
}

c::FixedTriangleFeatureKey Feature() {
  c::FixedTriangleFeatureKey result;
  result.vertex_face.vertex = Vertex(100);
  result.vertex_face.target.SetFace({17, 20, 0, 0});
  return result;
}

c::FixedContactFacet Facet(std::uint64_t eid,
                           unsigned local = 0) {
  c::FixedContactFacet result;
  result.source_instance_id = 17;
  result.source.source_parent_id = eid;
  result.local_facet = local;
  return result;
}

sct::AcceptedEventCertificate Certificate(
    std::uint64_t vertex) {
  sct::AcceptedEventCertificate result;
  result.event.feature = Feature();
  result.event.feature.vertex_face.vertex.first = vertex;
  result.event.vertex_use = 2;
  result.event.facet_use = 3;
  result.event.endpoints[0].count = 3;
  result.event.endpoints[1].count = 3;
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
    result.event.endpoints[endpoint].nodes[0] = 1;
    result.event.endpoints[endpoint].nodes[1] = 2;
    result.event.endpoints[endpoint].nodes[2] = 3;
    result.event.endpoints[endpoint].weights[0] = 1;
  }
  result.event.classification.kind = c::SelfContactPairKind::VertexFace;
  result.event.classification.status =
      c::SelfContactPairStatus::AdmittedVertexFace;
  result.event.classification.active[0] = true;
  result.event.classification.active[1] = true;
  result.event.classification.candidate_directed_area_m2 =
      {1, 1, 1, 0};
  result.event.classification.admitted_force_area_m2 =
      {1, 1, 1, 0};
  result.discovery.key = result.event.feature;
  result.discovery.triangles[0] = {17, vertex, 0, 0};
  result.discovery.triangles[1] = {17, 20, 0, 0};
  result.discovery.local_features[0] = 0;
  result.discovery.local_features[1] = 3;
  result.discovery.face_weights[0] = 1;
  result.vertex_facet = 2;
  result.target_facet = 3;
  return result;
}

c::CurrentFixedTriangle Triangle(
    std::uint64_t eid,
    std::array<std::uint64_t, 3> vertices,
    std::array<c::Vec3, 3> points) {
  c::CurrentFixedTriangle result;
  result.key = {17, eid, 0, 0};
  for (unsigned i = 0; i < 3; ++i) {
    result.vertex_keys[i] = Vertex(vertices[i]);
    result.vertices[i] = points[i];
    auto a = result.vertex_keys[i];
    auto b = result.vertex_keys[(i + 1) % 3];
    if (c::fixed_triangle_features::Compare(b, a) < 0)
      std::swap(a, b);
    result.edge_keys[i].parent_boundary = true;
    result.edge_keys[i].endpoints[0] = a;
    result.edge_keys[i].endpoints[1] = b;
  }
  return result;
}

sct::CandidateValidationInput Input(
    const c::RepresentedIntervalPairKey* pair,
    std::size_t pair_count,
    const c::RepresentedIntervalResult* crossing,
    c::SelfContactCandidatePolicyOutcome* outcomes,
    std::size_t* outcome_count) {
  sct::CandidateValidationInput input;
  input.canonical_pairs = pair;
  input.pair_count = pair_count;
  input.features = {nullptr, 0, true};
  input.intersections = {nullptr, 0, true};
  input.crossings = {crossing, pair_count, true};
  input.outcomes = outcomes;
  input.outcome_capacity = pair_count;
  input.outcome_count = outcome_count;
  return input;
}

TEST(SelfContactTransactionValues,
     FinalAuthorityIsNonaggregateAndCannotBeCallerConstructed) {
  using Receipt = c::SelfContactTransactionReceipt;
  using Assembly = c::SelfContactAcceptedAssemblyReceipt;
  EXPECT_FALSE(std::is_aggregate_v<Receipt>);
  EXPECT_FALSE(std::is_aggregate_v<Assembly>);
  EXPECT_TRUE(std::is_default_constructible_v<Receipt>);
  EXPECT_TRUE(std::is_copy_constructible_v<Receipt>);
  EXPECT_EQ(Receipt{}.source_id(), 0u);
  EXPECT_FALSE((std::is_constructible_v<
      Assembly, c::SelfContactAcceptedActivityReceipt>));
  EXPECT_FALSE((std::is_constructible_v<
      Receipt, c::SelfContactPreparedActivityReceipt>));
  EXPECT_FALSE((std::is_constructible_v<
      Receipt, tl::fea::ShellPhysicalScratchParticipationReceipt>));
  EXPECT_FALSE((std::is_convertible_v<
      tl::fea::ShellPhysicalScratchParticipationReceipt, Receipt>));
}

TEST(SelfContactTransactionValues,
     AuthorityMethodsExposeNoCallerPairFacetEventOrDiscoveryArrays) {
  using Accepted = c::SelfContactTransactionReport
      (c::SelfContactTransaction::*)(
          tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
          const tl::fea::NodalAssemblyView&,
          c::SelfContactAcceptedAssemblyReceipt*);
  using Candidate = c::SelfContactTransactionReport
      (c::SelfContactTransaction::*)(
          tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
          const tl::fea::ShellPhysicalDiagnostics&,
          const tl::fea::NodalPreparedView&,
          const c::SelfContactAcceptedAssemblyReceipt&,
          c::SelfContactTransactionReceipt*);
  EXPECT_TRUE((std::is_same_v<
      decltype(&c::SelfContactTransaction::AssembleAccepted),
      Accepted>));
  EXPECT_TRUE((std::is_same_v<
      decltype(&c::SelfContactTransaction::SealCandidate),
      Candidate>));
}

TEST(SelfContactTransactionValues,
     CandidateArenaHasExactInclusiveCap) {
  sct::Layout layout;
  ASSERT_TRUE(sct::MakeLayout(
      15, 4, 4, 8, 2, 6, 3, 7, 8, 16, 6,
      SIZE_MAX, layout));
  ASSERT_GT(layout.bytes, 0u);
  const auto exact = layout.bytes;
  sct::Layout unchanged = layout;
  EXPECT_FALSE(sct::MakeLayout(
      15, 4, 4, 8, 2, 6, 3, 7, 8, 16, 6,
      exact - 1, layout));
  EXPECT_EQ(layout.bytes, unchanged.bytes);
  ASSERT_TRUE(sct::MakeLayout(
      15, 4, 4, 8, 2, 6, 3, 7, 8, 16, 6,
      exact, layout));
  EXPECT_EQ(layout.bytes, exact);
}

TEST(SelfContactTransactionValues,
     MeasuredV5CountsConstructGenericBoundedVehicleShape) {
  const c::SelfContactTransactionLimits::ExactCensus census{
      376930, 337092, 337092, 315963, 653055,
      1584464, 5989248, 0};
  constexpr std::size_t chunk = 4096;
  constexpr std::size_t per_chunk_work = chunk * 4095;
  constexpr std::size_t complete_work = 5989248ull * 4095;
  const auto limits = c::SelfContactTransactionLimits::Vehicle(
      census, chunk, 1, 2, 0, 4095, per_chunk_work,
      complete_work, 20, 64ull << 30, 8ull << 30,
      96ull << 30);
  EXPECT_EQ(limits.broadphase.max_pairs, 1584464u);
  EXPECT_EQ(limits.max_candidate_pairs, 5989248u);
  EXPECT_EQ(limits.max_facet_pair_chunk, chunk);
  EXPECT_EQ(limits.accepted_discovery.max_raw_feature_candidates,
            15 * chunk);
  EXPECT_EQ(limits.crossing.max_paths, 2 * chunk);
  EXPECT_EQ(limits.max_policy_outcomes, 0u);

  sct::Layout minimum;
  ASSERT_TRUE(sct::MakeLayout(
      census.nodes, census.surface_parents,
      census.selected_parents, census.facets,
      779, census.parent_pairs, chunk, 1, 1, 2, 0,
      SIZE_MAX, minimum));
  sct::Layout million_events;
  ASSERT_TRUE(sct::MakeLayout(
      census.nodes, census.surface_parents,
      census.selected_parents, census.facets,
      779, census.parent_pairs, chunk,
      1000000, 1000000, 2000000, 0,
      SIZE_MAX, million_events));
  EXPECT_GT(minimum.bytes, 0u);
  EXPECT_GT(million_events.bytes, minimum.bytes);
  std::cout << "V5_STREAMING_ARENA minimum_event_bytes="
            << minimum.bytes
            << " million_event_bytes=" << million_events.bytes
            << " parent_key_bytes="
            << census.parent_pairs * sizeof(c::SelfContactPairKey)
            << " cursor_bytes="
            << census.parent_pairs * sizeof(sct::FacetPairCursor)
            << " heap_bytes="
            << census.parent_pairs * sizeof(std::uint32_t)
            << " chunk_pair_bytes="
            << chunk * sizeof(c::FixedTrianglePair)
            << '\n';
}

TEST(SelfContactTransactionValues,
     PairMotionIsScopedAndRigidBoxesOnlyProveSeparation) {
  const c::SelfContactSweptParentBounds near{
      {-1, -1, -1}, {1, 1, 1}};
  const c::SelfContactSweptParentBounds far{
      {3, -1, -1}, {4, 1, 1}};
  sct::MotionSupport ordinary;
  sct::MotionSupport rigid_a;
  rigid_a.motion = c::SelfContactFacetMotion::CompleteRigidGroup;
  rigid_a.complete_rigid_group = 7;
  rigid_a.rigid_groups[0] = 7;
  rigid_a.rigid_group_count = 1;
  auto rigid_b = rigid_a;
  rigid_b.complete_rigid_group = 9;
  rigid_b.rigid_groups[0] = 9;
  sct::MotionSupport partial;
  partial.motion = c::SelfContactFacetMotion::PartialOrMixedRigid;
  partial.rigid_groups[0] = 7;
  partial.rigid_group_count = 1;

  // An unrelated rigid facet elsewhere cannot change an ordinary pair.
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      ordinary, near, ordinary, near),
      sct::PairMotionAction::LinearNodalV1);
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      rigid_a, near, rigid_a, near),
      sct::PairMotionAction::ExcludedSameRigidGroup);
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      rigid_a, near, rigid_b, far),
      sct::PairMotionAction::CertifiedRigidArcSeparation);
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      rigid_a, near, rigid_b, near),
      sct::PairMotionAction::UnsupportedRigidArc);
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      partial, near, ordinary, near),
      sct::PairMotionAction::UnsupportedRigidArc);
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      partial, near, ordinary, far),
      sct::PairMotionAction::CertifiedRigidArcSeparation);

  // Numeric source-ID collisions do not merge different actual group rows.
  rigid_b.complete_rigid_group = 8;
  rigid_b.rigid_groups[0] = 8;
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      rigid_a, near, rigid_b, near),
      sct::PairMotionAction::UnsupportedRigidArc);
}

TEST(SelfContactTransactionValues,
     CompleteSeparatedRosterPassesAndUnresolvedNeverPasses) {
  const auto pair = Pair(10, 20);
  c::RepresentedIntervalResult result;
  result.key = pair;
  result.classification =
      c::RepresentedIntervalClassification::CertifiedSeparated;
  c::SelfContactCandidatePolicyOutcome outcome;
  std::size_t outcome_count = 9;
  EXPECT_EQ(sct::ValidateCandidatePublications(
      Input(&pair, 1, &result, &outcome, &outcome_count)).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(outcome_count, 1u);
  EXPECT_EQ(outcome.disposition,
      c::SelfContactCandidateDisposition::CertifiedSeparated);

  result.classification =
      c::RepresentedIntervalClassification::Unresolved;
  result.reason = c::RepresentedIntervalReason::WorkExhausted;
  const auto unresolved = sct::ValidateCandidatePublications(
      Input(&pair, 1, &result, &outcome, &outcome_count));
  EXPECT_EQ(unresolved.status,
            c::SelfContactTransactionStatus::UnresolvedCandidate);
  EXPECT_EQ(unresolved.crossing_reason,
            c::RepresentedIntervalReason::WorkExhausted);
  result.key = Pair(10, 30);
  EXPECT_EQ(sct::ValidateCandidatePublications(
      Input(&pair, 1, &result, &outcome, &outcome_count)).status,
      c::SelfContactTransactionStatus::IdentityMismatch);
}

TEST(SelfContactTransactionValues,
     PassThroughCrossingNeedsFullAcceptedVfCertificate) {
  const auto pair = Pair(10, 20);
  c::RepresentedIntervalResult result;
  result.key = pair;
  result.classification =
      c::RepresentedIntervalClassification::CertifiedCrossingContact;
  result.feature.kind = c::RepresentedFeatureKind::VertexFace;
  result.feature.vertex = Vertex(100);
  result.feature.face = Path(20);

  c::SelfContactCandidatePolicyOutcome outcome;
  std::size_t outcome_count = 0;
  auto input = Input(
      &pair, 1, &result, &outcome, &outcome_count);
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);

  sct::AcceptedEventCertificate event;
  event.event.feature = Feature();
  event.event.source_order = 0;
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
    event.event.endpoints[endpoint].count = 3;
    event.event.endpoints[endpoint].nodes[0] = 0;
    event.event.endpoints[endpoint].nodes[1] = 1;
    event.event.endpoints[endpoint].nodes[2] = 2;
    event.event.endpoints[endpoint].weights[0] = 1;
  }
  event.event.classification.kind = c::SelfContactPairKind::VertexFace;
  event.event.classification.status =
      c::SelfContactPairStatus::AdmittedVertexFace;
  event.event.classification.active[0] = true;
  event.event.classification.active[1] = true;
  event.event.classification.candidate_directed_area_m2 = {1, 1, 1, 0};
  event.event.classification.admitted_force_area_m2 = {1, 1, 1, 0};
  event.discovery.key = Feature();
  event.discovery.local_features[0] = 0;
  event.discovery.local_features[1] = 3;
  event.discovery.face_weights[0] = 1;
  event.vertex_facet = 1;
  event.target_facet = 2;
  input.accepted_events = &event;
  input.accepted_event_count = 1;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(outcome.accepted_event, 0u);
  EXPECT_EQ(outcome.source_order, 0u);

  event.event.source_order = 1;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);
  event.event.source_order = 0;

  event.discovery.face_weights[0] = .5;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);
  event.discovery.face_weights[0] = 1;

  event.event.feature.vertex_face.vertex.first++;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);
  event.event.feature = Feature();

  event.event.classification.admitted_force_area_m2.lower = 2;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);
  event.event.classification.admitted_force_area_m2.lower = 1;

  result.feature.kind = c::RepresentedFeatureKind::EdgeEdge;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);
}

TEST(SelfContactTransactionValues,
     EeCoverageRequiresTheSameExactFixedFacetPair) {
  c::FixedTriangleFeatureCandidate vf;
  vf.key.kind = c::FixedTriangleCandidateKind::VertexFace;
  vf.triangles[0] = {17, 100, 1, 3};
  vf.triangles[1] = {17, 200, 1, 7};
  c::FixedTriangleFeatureCandidate ee;
  ee.key.SetEdgeEdge();
  ee.triangles[0] = vf.triangles[1];
  ee.triangles[1] = vf.triangles[0];
  EXPECT_TRUE(sct::ExactFacetPair(vf, ee));

  // Parent-pair equality is insufficient: this EE may be spatially unrelated
  // to the admitted VF and has no independently authenticated force area.
  ee.triangles[0].local_facet++;
  EXPECT_FALSE(sct::ExactFacetPair(vf, ee));
  ee.triangles[0] = vf.triangles[1];
  ee.triangles[0].level++;
  EXPECT_FALSE(sct::ExactFacetPair(vf, ee));
  ee.triangles[0] = vf.triangles[1];
  ee.triangles[0].source_instance_id++;
  EXPECT_FALSE(sct::ExactFacetPair(vf, ee));
}

TEST(SelfContactTransactionValues,
     LocalIntersectionNeedsNoCallerAdmissionButNonlocalDoes) {
  const auto pair = Pair(10, 20);
  c::RepresentedIntervalResult result;
  result.key = pair;
  result.classification =
      c::RepresentedIntervalClassification::CertifiedCrossingContact;
  result.feature.kind = c::RepresentedFeatureKind::TriangleIntersection;
  c::FixedTriangleIntersection intersection;
  intersection.triangles[0] = {17, 10, 0, 0};
  intersection.triangles[1] = {17, 20, 0, 0};
  intersection.kind =
      c::FixedTriangleIntersectionKind::CoplanarTouch;
  intersection.local_exclusion =
      c::FixedTriangleLocalExclusion::SharedEdgeOnly;

  c::SelfContactCandidatePolicyOutcome outcome;
  std::size_t outcome_count = 0;
  auto input = Input(
      &pair, 1, &result, &outcome, &outcome_count);
  input.intersections = {&intersection, 1, true};
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::Ok);

  intersection.local_exclusion =
      c::FixedTriangleLocalExclusion::None;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);
}

TEST(SelfContactTransactionValues,
     EmptyCompleteCandidatePublishesEmptyPolicy) {
  std::size_t outcomes = 7;
  auto input = Input(nullptr, 0, nullptr, nullptr, &outcomes);
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
            c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(outcomes, 0u);
}

TEST(SelfContactTransactionValues,
     CompleteFacetExpansionCountsBeforeWriteAndRejectsOmissionShapes) {
  const c::SelfContactPairKey key =
      (std::uint64_t{0} << 32) | 1;
  const std::uint32_t map[]{0, 1};
  // Parent 0 has two T3-like facets; parent 1 has four Q4-like facets.
  const std::uint32_t offsets[]{0, 2, 6};
  const std::uint8_t all_active[]{1, 1};
  const c::SelfContactActivityView activity{
      all_active, all_active, 2};
  std::array<c::FixedTrianglePair, 8> pairs{};
  std::size_t count = 99;
  EXPECT_EQ(sct::ExpandFacetPairs(
      &key, 1, map, 2, offsets, 2, activity,
      pairs.data(), 7, &count).status,
      c::SelfContactTransactionStatus::ResourceLimit);
  EXPECT_EQ(count, 99u);
  ASSERT_EQ(sct::ExpandFacetPairs(
      &key, 1, map, 2, offsets, 2, activity,
      pairs.data(), 8, &count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(count, 8u);
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    EXPECT_EQ(pairs[i].first, i / 4);
    EXPECT_EQ(pairs[i].second, 2 + i % 4);
  }

  const c::SelfContactPairKey duplicate[]{key, key};
  EXPECT_EQ(sct::ExpandFacetPairs(
      duplicate, 2, map, 2, offsets, 2,
      activity, pairs.data(), pairs.size(), &count).status,
      c::SelfContactTransactionStatus::IdentityMismatch);

  const std::uint8_t removing[]{0, 1};
  ASSERT_EQ(sct::ExpandFacetPairs(
      &key, 1, map, 2, offsets, 2,
      {all_active, removing, 2},
      pairs.data(), pairs.size(), &count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(count, 0u);
}

TEST(SelfContactTransactionValues,
     CanonicalChunksEqualWholeBatchTinyOracle) {
  const c::SelfContactPairKey keys[]{
      (std::uint64_t{0} << 32) | 1,
      (std::uint64_t{0} << 32) | 2,
      (std::uint64_t{1} << 32) | 2};
  const std::uint32_t map[]{0, 1, 2};
  const std::uint32_t offsets[]{0, 2, 4, 6};
  const std::uint8_t active[]{1, 1, 1};
  std::array<c::FixedContactFacet, 6> descriptors{
      Facet(10, 0), Facet(10, 1),
      Facet(20, 0), Facet(20, 1),
      Facet(30, 0), Facet(30, 1)};
  std::array<c::FixedTrianglePair, 12> whole{};
  std::size_t whole_count = 0;
  ASSERT_EQ(sct::ExpandFacetPairs(
      keys, 3, map, 3, offsets, 3, {active, active, 3},
      whole.data(), whole.size(), &whole_count).status,
      c::SelfContactTransactionStatus::Ok);

  std::array<sct::FacetPairCursor, 3> cursors{};
  std::array<std::uint32_t, 3> heap{};
  std::array<c::FixedTrianglePair, 3> chunk{};
  sct::StreamingCandidateSource stream;
  ASSERT_EQ(stream.Initialize(
      descriptors.data(), descriptors.size(),
      cursors.data(), cursors.size(),
      heap.data(), heap.size(),
      chunk.data(), chunk.size(), whole.size()).status,
      c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(stream.Begin(
      keys, 3, map, 3, offsets, 3,
      {active, active, 3}).status,
      c::SelfContactTransactionStatus::Ok);
  std::vector<c::FixedTrianglePair> streamed;
  for (;;) {
    const c::FixedTrianglePair* values = nullptr;
    std::size_t count = 0;
    ASSERT_EQ(stream.Next(&values, &count).status,
              c::SelfContactTransactionStatus::Ok);
    if (!count) break;
    streamed.insert(streamed.end(), values, values + count);
  }
  sct::StreamingCandidateSourceReceipt receipt;
  ASSERT_EQ(stream.Finish(&receipt).status,
            c::SelfContactTransactionStatus::Ok);
  EXPECT_TRUE(stream.Authenticates(receipt));
  EXPECT_EQ(receipt.parent_pairs(), 3u);
  EXPECT_EQ(receipt.facet_pairs(), whole_count);
  ASSERT_EQ(streamed.size(), whole_count);
  for (std::size_t i = 0; i < whole_count; ++i) {
    EXPECT_EQ(streamed[i].first, whole[i].first);
    EXPECT_EQ(streamed[i].second, whole[i].second);
  }
  const c::SelfContactPairKey duplicate[]{keys[0], keys[0]};
  EXPECT_EQ(stream.Begin(
      duplicate, 2, map, 3, offsets, 3,
      {active, active, 3}).status,
      c::SelfContactTransactionStatus::IdentityMismatch);
  EXPECT_FALSE(stream.Authenticates(receipt));
}

TEST(SelfContactTransactionValues,
     MillionParentPairsUseBoundedChunksAndCompleteReceipt) {
  constexpr std::size_t parents = 1415;
  const std::size_t pair_count = parents * (parents - 1) / 2;
  ASSERT_GT(pair_count, 1000000u);
  std::vector<c::FixedContactFacet> descriptors;
  std::vector<std::uint32_t> map(parents);
  std::vector<std::uint32_t> offsets(parents + 1);
  std::vector<std::uint8_t> active(parents, 1);
  descriptors.reserve(parents);
  for (std::size_t parent = 0; parent < parents; ++parent) {
    descriptors.push_back(Facet(1000 + parent));
    map[parent] = parent;
    offsets[parent] = parent;
  }
  offsets[parents] = parents;
  std::vector<c::SelfContactPairKey> keys;
  keys.reserve(pair_count);
  for (std::uint32_t first = 0; first < parents; ++first)
    for (std::uint32_t second = first + 1;
         second < parents; ++second)
      keys.push_back((std::uint64_t{first} << 32) | second);
  std::vector<sct::FacetPairCursor> cursors(pair_count);
  std::vector<std::uint32_t> heap(pair_count);
  std::array<c::FixedTrianglePair, 257> chunk{};
  sct::StreamingCandidateSource stream;
  ASSERT_EQ(stream.Initialize(
      descriptors.data(), descriptors.size(),
      cursors.data(), cursors.size(), heap.data(), heap.size(),
      chunk.data(), chunk.size(), pair_count).status,
      c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(stream.Begin(
      keys.data(), keys.size(), map.data(), parents,
      offsets.data(), parents, {active.data(), active.data(), parents}).status,
      c::SelfContactTransactionStatus::Ok);
  sct::StreamingCandidateSourceReceipt premature;
  EXPECT_EQ(stream.Finish(&premature).status,
            c::SelfContactTransactionStatus::IdentityMismatch);
  std::size_t emitted = 0;
  for (;;) {
    const c::FixedTrianglePair* values = nullptr;
    std::size_t count = 0;
    ASSERT_EQ(stream.Next(&values, &count).status,
              c::SelfContactTransactionStatus::Ok);
    if (!count) break;
    ASSERT_LE(count, chunk.size());
    for (std::size_t i = 1; i < count; ++i) {
      EXPECT_TRUE(values[i - 1].first < values[i].first ||
          (values[i - 1].first == values[i].first &&
           values[i - 1].second < values[i].second));
    }
    emitted += count;
  }
  sct::StreamingCandidateSourceReceipt receipt;
  ASSERT_EQ(stream.Finish(&receipt).status,
            c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(emitted, pair_count);
  EXPECT_EQ(receipt.facet_pairs(), pair_count);
  EXPECT_TRUE(stream.Authenticates(receipt));
  EXPECT_FALSE(sct::StreamingCandidateSourceReceipt{}.complete());
}

TEST(SelfContactTransactionValues,
     CrossChunkEventDedupIdentityAndExactForceCapAreAtomic) {
  auto first = Certificate(100);
  auto second = Certificate(90);
  std::array<sct::AcceptedEventCertificate, 2> ledger{};
  std::array<std::uint32_t, 5> hash{};
  hash.fill(UINT32_MAX);
  std::size_t count = 0;
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &first, 1, ledger.data(), ledger.size(),
      hash.data(), hash.size(), &count).status,
      c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &first, 1, ledger.data(), ledger.size(),
      hash.data(), hash.size(), &count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(count, 1u);
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &second, 1, ledger.data(), ledger.size(),
      hash.data(), hash.size(), &count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(count, 2u);

  std::array<c::SelfContactForceEvent, 2> output{};
  output[0].source_order = 777;
  const auto short_cap = sct::FinalizeAcceptedEventLedger(
      ledger.data(), count, output.data(), 1);
  EXPECT_EQ(short_cap.status,
            c::SelfContactTransactionStatus::ResourceLimit);
  EXPECT_EQ(short_cap.candidate, 2u);
  EXPECT_EQ(output[0].source_order, 777u);
  ASSERT_EQ(sct::FinalizeAcceptedEventLedger(
      ledger.data(), count, output.data(), 2).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(output[0].feature.vertex_face.vertex.first, 90u);
  EXPECT_EQ(output[0].source_order, 0u);
  EXPECT_EQ(output[1].source_order, 1u);

  hash.fill(UINT32_MAX);
  count = 0;
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &first, 1, ledger.data(), ledger.size(),
      hash.data(), hash.size(), &count).status,
      c::SelfContactTransactionStatus::Ok);
  first.discovery.distance_m = 1;
  EXPECT_EQ(sct::MergeAcceptedEventChunk(
      &first, 1, ledger.data(), ledger.size(),
      hash.data(), hash.size(), &count).status,
      c::SelfContactTransactionStatus::IdentityMismatch);
  first.discovery.distance_m = 0;
  hash.fill(UINT32_MAX);
  count = 0;
  EXPECT_EQ(sct::MergeAcceptedEventChunk(
      &first, 1, ledger.data(), ledger.size(),
      hash.data(), hash.size(), &count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(count, 1u);
}

TEST(SelfContactTransactionValues,
     CompleteIdentityLedgerCatchesCrossChunkVertexMismatch) {
  std::array<c::CurrentFixedTriangle, 2> triangles{
      Triangle(10, {1, 2, 3},
               {{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}}),
      Triangle(20, {1, 4, 5},
               {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}})};
  std::array<std::uint32_t, 6> vertex_order{};
  std::array<std::uint32_t, 6> edge_order{};
  for (std::uint32_t i = 0; i < 6; ++i)
    vertex_order[i] = edge_order[i] = i;
  std::sort(vertex_order.begin(), vertex_order.end(),
            [&](std::uint32_t a, std::uint32_t b) {
              return c::fixed_triangle_features::Compare(
                  triangles[a / 3].vertex_keys[a % 3],
                  triangles[b / 3].vertex_keys[b % 3]) < 0;
            });
  std::sort(edge_order.begin(), edge_order.end(),
            [&](std::uint32_t a, std::uint32_t b) {
              return c::fixed_triangle_features::Compare(
                  triangles[a / 3].edge_keys[a % 3],
                  triangles[b / 3].edge_keys[b % 3]) < 0;
            });
  EXPECT_EQ(sct::ValidateCompleteTriangleIdentities(
      triangles.data(), triangles.size(),
      vertex_order.data(), edge_order.data()).status,
      c::SelfContactTransactionStatus::Ok);
  triangles[1].vertices[0].z = 1;
  EXPECT_EQ(sct::ValidateCompleteTriangleIdentities(
      triangles.data(), triangles.size(),
      vertex_order.data(), edge_order.data()).status,
      c::SelfContactTransactionStatus::IdentityMismatch);
}

TEST(SelfContactTransactionValues,
     FoldedPolicyIsChunkBoundaryInvariant) {
  std::array<c::SelfContactCandidatePolicyOutcome, 3> values{};
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i].pair = Pair(10 + i, 20 + i);
    values[i].disposition = i == 0
        ? c::SelfContactCandidateDisposition::CertifiedSeparated
        : (i == 1
            ? c::SelfContactCandidateDisposition::
                ExcludedLocalIntersection
            : c::SelfContactCandidateDisposition::
                RepresentedByAcceptedVertexFace);
    values[i].accepted_event = i == 2 ? 4 : SIZE_MAX;
    values[i].source_order = i == 2 ? 4 : UINT64_MAX;
  }
  c::SelfContactCandidatePolicySummary whole;
  c::SelfContactCandidatePolicySummary chunks;
  sct::FoldPolicyOutcomes(values.data(), values.size(), &whole);
  sct::FoldPolicyOutcomes(values.data(), 1, &chunks);
  sct::FoldPolicyOutcomes(values.data() + 1, 2, &chunks);
  EXPECT_EQ(whole.outcomes, chunks.outcomes);
  EXPECT_EQ(whole.certified_separated, 1u);
  EXPECT_EQ(whole.excluded_local_intersection, 1u);
  EXPECT_EQ(whole.represented_by_accepted_vf, 1u);
  EXPECT_EQ(whole.digest, chunks.digest);
}

}  // namespace
