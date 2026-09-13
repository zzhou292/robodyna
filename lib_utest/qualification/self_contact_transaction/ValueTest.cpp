// SPDX-License-Identifier: MIT
#include "lib_src/collision/self_contact_transaction/Storage.h"

#include <gtest/gtest.h>

#include <array>
#include <type_traits>

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
  return result;
}

c::FixedTriangleFeatureKey Feature() {
  c::FixedTriangleFeatureKey result;
  result.vertex_face.vertex = Vertex(100);
  result.vertex_face.target.SetFace({17, 20, 0, 0});
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
  EXPECT_FALSE(Receipt{}.valid());
  EXPECT_EQ(Receipt{}.scratch_receipts().self_contact, nullptr);
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
      15, 4, 4, 8, 6, 6, 7, SIZE_MAX, layout));
  ASSERT_GT(layout.bytes, 0u);
  const auto exact = layout.bytes;
  sct::Layout unchanged = layout;
  EXPECT_FALSE(sct::MakeLayout(
      15, 4, 4, 8, 6, 6, 7, exact - 1, layout));
  EXPECT_EQ(layout.bytes, unchanged.bytes);
  ASSERT_TRUE(sct::MakeLayout(
      15, 4, 4, 8, 6, 6, 7, exact, layout));
  EXPECT_EQ(layout.bytes, exact);
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
  std::array<c::FixedTrianglePair, 8> pairs{};
  std::size_t count = 99;
  EXPECT_EQ(sct::ExpandFacetPairs(
      &key, 1, map, 2, offsets, 2, pairs.data(), 7, &count).status,
      c::SelfContactTransactionStatus::ResourceLimit);
  EXPECT_EQ(count, 99u);
  ASSERT_EQ(sct::ExpandFacetPairs(
      &key, 1, map, 2, offsets, 2, pairs.data(), 8, &count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(count, 8u);
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    EXPECT_EQ(pairs[i].first, i / 4);
    EXPECT_EQ(pairs[i].second, 2 + i % 4);
  }

  const c::SelfContactPairKey duplicate[]{key, key};
  EXPECT_EQ(sct::ExpandFacetPairs(
      duplicate, 2, map, 2, offsets, 2,
      pairs.data(), pairs.size(), &count).status,
      c::SelfContactTransactionStatus::IdentityMismatch);
}

}  // namespace
