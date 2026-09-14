// SPDX-License-Identifier: MIT
#include "lib_src/collision/self_contact_transaction/Storage.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace {
namespace c = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;

constexpr std::size_t DecisionCount = 262144;
constexpr std::size_t ChunkCapacity = 257;
constexpr std::uint64_t ExpectedPolicyDigest =
    7261953295680066653ull;

c::RepresentedTrianglePathKey Path(std::uint64_t parent) {
  return {17, parent, 0, 0};
}

c::RepresentedIntervalPairKey Pair(std::size_t ordinal) {
  return {{Path(1000 + ordinal),
           Path(1000 + DecisionCount + ordinal)}};
}

c::FacetVertexKey Vertex(std::uint64_t id) {
  c::FacetVertexKey result;
  result.source_instance_id = 17;
  result.first = id;
  result.denominator = 1;
  return result;
}

c::FacetEdgeKey Edge(std::uint64_t first, std::uint64_t second,
                     std::uint64_t parent) {
  c::FacetEdgeKey result;
  result.parent_boundary = true;
  result.parent_eid = parent;
  result.endpoints[0] = Vertex(first);
  result.endpoints[1] = Vertex(second);
  if (c::fixed_triangle_features::Compare(
          result.endpoints[1], result.endpoints[0]) < 0)
    std::swap(result.endpoints[0], result.endpoints[1]);
  return result;
}

sct::AcceptedEventCertificate VertexFace(
    std::uint32_t first_owner, std::uint32_t second_owner) {
  sct::AcceptedEventCertificate result;
  result.event.feature.vertex_face.vertex = Vertex(100);
  result.event.feature.vertex_face.target.SetFace({17, 20, 0, 0});
  result.event.vertex_use = 2;
  result.event.facet_use = 3;
  result.event.classification.kind = c::SelfContactPairKind::VertexFace;
  result.event.classification.status =
      c::SelfContactPairStatus::AdmittedVertexFace;
  result.event.classification.parent[0] = first_owner;
  result.event.classification.parent[1] = second_owner;
  result.event.classification.active[0] = true;
  result.event.classification.active[1] = true;
  result.event.classification.candidate_directed_area_m2 = {1, 1, 1, 0};
  result.event.classification.admitted_force_area_m2 = {1, 1, 1, 0};
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
    result.event.endpoints[endpoint].count = 3;
    result.event.endpoints[endpoint].nodes[0] = 1;
    result.event.endpoints[endpoint].nodes[1] = 2;
    result.event.endpoints[endpoint].nodes[2] = 3;
    result.event.endpoints[endpoint].weights[0] = 1;
  }
  result.discovery.key = result.event.feature;
  result.discovery.triangles[0] = {17, 10, 0, 0};
  result.discovery.triangles[1] = {17, 20, 0, 0};
  result.discovery.face_weights[0] = 1;
  result.vertex_facet = 2;
  result.target_facet = 3;
  return result;
}

sct::AcceptedEventCertificate EdgeEdge(
    std::uint32_t first_owner, std::uint32_t second_owner) {
  sct::AcceptedEventCertificate result;
  result.kind = sct::AcceptedEventCertificateKind::EdgeEdge;
  result.event.feature.SetEdgeEdge();
  result.event.feature.edge_edge.edges[0] = Edge(1, 2, 10);
  result.event.feature.edge_edge.edges[1] = Edge(3, 4, 20);
  result.event.edge_use[0] = 4;
  result.event.edge_use[1] = 5;
  result.event.classification.kind = c::SelfContactPairKind::EdgeEdge;
  result.event.classification.edge_edge_case =
      c::SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum;
  result.event.classification.status =
      c::SelfContactPairStatus::AdmittedEdgeEdge;
  result.event.classification.parent[0] = first_owner;
  result.event.classification.parent[1] = second_owner;
  result.event.classification.active[0] = true;
  result.event.classification.active[1] = true;
  result.event.classification.candidate_directed_area_m2 = {1, 1, 1, 0};
  result.event.classification.admitted_force_area_m2 = {1, 1, 1, 0};
  for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
    result.event.endpoints[endpoint].count = 3;
    result.event.endpoints[endpoint].nodes[0] = 3 * endpoint;
    result.event.endpoints[endpoint].nodes[1] = 3 * endpoint + 1;
    result.event.endpoints[endpoint].nodes[2] = 3 * endpoint + 2;
    result.event.endpoints[endpoint].weights[0] = .5;
    result.event.endpoints[endpoint].weights[1] = .5;
  }
  result.discovery.key = result.event.feature;
  result.discovery.triangles[0] = {17, 10, 0, 0};
  result.discovery.triangles[1] = {17, 20, 0, 0};
  result.discovery.edge_parameters[0] = .5;
  result.discovery.edge_parameters[1] = .5;
  result.edge_facet[0] = 1;
  result.edge_facet[1] = 2;
  return result;
}

struct FixedStorage {
  std::array<c::SelfContactCandidatePolicyOutcome, ChunkCapacity> outcomes;
  std::array<sct::AcceptedEventCertificate, 8> ledger;
  std::array<std::uint32_t, 17> hash;
  std::array<c::SelfContactForceEvent, 8> events;
};

static_assert(sizeof(FixedStorage) < 512u * 1024u);

TEST(SelfContactTransactionMediumCoupon,
     CompleteDeterministicDecisionStreamUsesFixedStorage) {
  const c::SelfContactSweptParentBounds inflated{
      {0, 0, 0}, {1, 1, 1}};
  const c::SelfContactSweptParentBounds strictly_separated{
      {std::nextafter(1.0, INFINITY), 0, 0}, {2, 1, 1}};
  const c::SelfContactSweptParentBounds touching{
      {1, 0, 0}, {2, 1, 1}};
  sct::MotionSupport linear;
  sct::MotionSupport rigid;
  rigid.motion = c::SelfContactFacetMotion::CompleteRigidGroup;
  rigid.complete_rigid_group = 7;
  rigid.rigid_groups[0] = 7;
  rigid.rigid_group_count = 1;

  FixedStorage storage{};
  c::SelfContactCandidatePolicySummary summary;
  c::RepresentedIntervalPairKey previous;
  bool have_previous = false;
  std::size_t input_count = 0;
  std::size_t chunk_count = 0;
  for (std::size_t begin = 0; begin < DecisionCount;
       begin += ChunkCapacity) {
    const auto count =
        std::min(ChunkCapacity, DecisionCount - begin);
    for (std::size_t local = 0; local < count; ++local) {
      const auto ordinal = begin + local;
      const auto residue = ordinal % 5;
      auto& outcome = storage.outcomes[local];
      outcome = {};
      outcome.pair = Pair(ordinal);
      if (have_previous)
        ASSERT_LT(sct::Compare(previous, outcome.pair), 0);
      previous = outcome.pair;
      have_previous = true;

      sct::PairMotionAction action;
      if (residue == 0)
        action = sct::ClassifyCandidatePairMotion(
            linear, inflated, linear, strictly_separated);
      else if (residue == 1)
        action = sct::ClassifyCandidatePairMotion(
            linear, inflated, linear, touching);
      else if (residue == 2)
        action = sct::ClassifyCandidatePairMotion(
            linear, inflated, linear, inflated);
      else if (residue == 3)
        action = sct::ClassifyCandidatePairMotion(
            rigid, inflated, rigid, inflated);
      else
        action = sct::ClassifyCandidatePairMotion(
            linear, strictly_separated, linear, inflated);

      if (action == sct::PairMotionAction::CertifiedLinearSeparation) {
        outcome.disposition =
            c::SelfContactCandidateDisposition::CertifiedSeparated;
        ++summary.motion_certified_linear_separated;
      } else if (action ==
                 sct::PairMotionAction::ExcludedSameRigidGroup) {
        outcome.disposition =
            c::SelfContactCandidateDisposition::ExcludedSameRigidGroup;
        ++summary.motion_excluded_same_rigid_group;
      } else {
        ASSERT_EQ(action, sct::PairMotionAction::LinearNodalV1);
        outcome.disposition = residue == 1
            ? c::SelfContactCandidateDisposition::
                RepresentedByAcceptedVertexFace
            : c::SelfContactCandidateDisposition::
                RepresentedByAcceptedEdgeEdge;
        outcome.accepted_event = residue == 1 ? 0 : 2;
        outcome.source_order = outcome.accepted_event;
        ++summary.exact_crossing_pairs;
      }
      ++input_count;
    }
    sct::FoldPolicyOutcomes(
        storage.outcomes.data(), count, &summary);
    ++chunk_count;
  }
  summary.complete = true;

  EXPECT_EQ(input_count, DecisionCount);
  EXPECT_EQ(chunk_count,
            (DecisionCount + ChunkCapacity - 1) / ChunkCapacity);
  EXPECT_EQ(summary.outcomes, DecisionCount);
  EXPECT_EQ(summary.motion_certified_linear_separated, 104857u);
  EXPECT_EQ(summary.motion_excluded_same_rigid_group, 52429u);
  EXPECT_EQ(summary.exact_crossing_pairs, 104858u);
  EXPECT_EQ(summary.certified_separated, 104857u);
  EXPECT_EQ(summary.excluded_same_rigid_group, 52429u);
  EXPECT_EQ(summary.represented_by_accepted_vf, 52429u);
  EXPECT_EQ(summary.represented_by_accepted_ee, 52429u);
  EXPECT_EQ(summary.digest, ExpectedPolicyDigest);
  EXPECT_EQ(summary.outcomes,
            summary.motion_certified_linear_separated +
                summary.motion_excluded_same_rigid_group +
                summary.exact_crossing_pairs);
  EXPECT_LT(sizeof(storage), 512u * 1024u);
}

TEST(SelfContactTransactionMediumCoupon,
     OwnerAwareVfAndEeDeduplicateAcrossChunks) {
  FixedStorage storage{};
  storage.hash.fill(UINT32_MAX);
  std::size_t ledger_count = 0;
  const std::array<sct::AcceptedEventCertificate, 2> first_chunk{
      VertexFace(2, 3), EdgeEdge(6, 7)};
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      first_chunk.data(), first_chunk.size(),
      storage.ledger.data(), storage.ledger.size(),
      storage.hash.data(), storage.hash.size(),
      &ledger_count).status, c::SelfContactTransactionStatus::Ok);
  const std::array<sct::AcceptedEventCertificate, 4> second_chunk{
      VertexFace(2, 3), EdgeEdge(6, 7),
      VertexFace(4, 5), EdgeEdge(8, 9)};
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      second_chunk.data(), second_chunk.size(),
      storage.ledger.data(), storage.ledger.size(),
      storage.hash.data(), storage.hash.size(),
      &ledger_count).status, c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(ledger_count, 4u);
  ASSERT_EQ(sct::FinalizeAcceptedEventLedger(
      storage.ledger.data(), ledger_count,
      storage.events.data(), storage.events.size()).status,
      c::SelfContactTransactionStatus::Ok);

  std::size_t vf = 0;
  std::size_t ee = 0;
  for (std::size_t event = 0; event < ledger_count; ++event) {
    EXPECT_EQ(storage.events[event].source_order, event);
    if (event)
      EXPECT_LT(c::CompareSelfContactForceEventIdentity(
                    storage.events[event - 1],
                    storage.events[event]), 0);
    if (storage.events[event].feature.kind ==
        c::FixedTriangleCandidateKind::VertexFace)
      ++vf;
    else
      ++ee;
  }
  EXPECT_EQ(vf, 2u);
  EXPECT_EQ(ee, 2u);
}

}  // namespace
