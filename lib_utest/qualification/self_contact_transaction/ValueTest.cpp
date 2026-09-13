// SPDX-License-Identifier: MIT
#include "lib_src/collision/self_contact_transaction/Storage.h"

#include <gtest/gtest.h>

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
    const c::RepresentedIntervalResult* crossing) {
  sct::CandidateValidationInput input;
  input.canonical_pairs = pair;
  input.pair_count = 1;
  input.features = {nullptr, 0, true};
  input.intersections = {nullptr, 0, true};
  input.crossings = {crossing, 1, true};
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
     CandidateArenaHasExactInclusiveCap) {
  sct::Layout layout;
  ASSERT_TRUE(sct::MakeLayout(15, 4, 8, 6, 7, SIZE_MAX, layout));
  ASSERT_GT(layout.bytes, 0u);
  const auto exact = layout.bytes;
  sct::Layout unchanged = layout;
  EXPECT_FALSE(sct::MakeLayout(
      15, 4, 8, 6, 7, exact - 1, layout));
  EXPECT_EQ(layout.bytes, unchanged.bytes);
  ASSERT_TRUE(sct::MakeLayout(15, 4, 8, 6, 7, exact, layout));
  EXPECT_EQ(layout.bytes, exact);
}

TEST(SelfContactTransactionValues,
     CompleteSeparatedRosterPassesAndUnresolvedNeverPasses) {
  const auto pair = Pair(10, 20);
  c::RepresentedIntervalResult result;
  result.key = pair;
  result.classification =
      c::RepresentedIntervalClassification::CertifiedSeparated;
  EXPECT_EQ(sct::ValidateCandidatePublications(
      Input(&pair, &result)).status,
      c::SelfContactTransactionStatus::Ok);

  result.classification =
      c::RepresentedIntervalClassification::Unresolved;
  EXPECT_EQ(sct::ValidateCandidatePublications(
      Input(&pair, &result)).status,
      c::SelfContactTransactionStatus::UnresolvedCandidate);
  result.key = Pair(10, 30);
  EXPECT_EQ(sct::ValidateCandidatePublications(
      Input(&pair, &result)).status,
      c::SelfContactTransactionStatus::IdentityMismatch);
}

TEST(SelfContactTransactionValues,
     PassThroughCrossingNeedsExactPolicyAndAcceptedVfIdentity) {
  const auto pair = Pair(10, 20);
  c::RepresentedIntervalResult result;
  result.key = pair;
  result.classification =
      c::RepresentedIntervalClassification::CertifiedCrossingContact;
  result.feature.kind = c::RepresentedFeatureKind::VertexFace;
  result.feature.vertex = Vertex(100);
  result.feature.face = Path(20);

  auto input = Input(&pair, &result);
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::UnresolvedCandidate);

  c::SelfContactCrossingDecision decision{
      pair, c::SelfContactCrossingDisposition::RejectCandidate};
  input.decisions = &decision;
  input.decision_count = 1;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);

  decision.disposition =
      c::SelfContactCrossingDisposition::
          RepresentedByAcceptedVertexFace;
  sct::AcceptedEventIdentity event{Feature(), 9};
  input.accepted_events = &event;
  input.accepted_event_count = 1;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::Ok);
  event.feature.vertex_face.vertex.first++;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::IdentityMismatch);
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

  auto input = Input(&pair, &result);
  input.intersections = {&intersection, 1, true};
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::Ok);

  intersection.local_exclusion =
      c::FixedTriangleLocalExclusion::None;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::UnresolvedCandidate);
}

}  // namespace
