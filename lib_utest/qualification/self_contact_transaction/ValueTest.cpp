// SPDX-License-Identifier: MIT
#include "lib_src/collision/self_contact_transaction/Storage.h"
#include "lib_src/collision/SelfContactForceValues.h"

#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <limits>
#include <type_traits>
#include <vector>

namespace {
namespace c = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;

static_assert(sizeof(c::SelfContactFacetFilterCategory) == 1);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetFilterCategory::ExcludedSameRigidGroup) == 0);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetFilterCategory::CoordinateAabbSeparated) == 1);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetFilterCategory::FaceAxisSeparated) == 2);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetFilterCategory::EdgeCrossAxisSeparated) == 3);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetFilterCategory::ExactRemaining) == 4);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetFilterCategory::VertexEdgeAxisSeparated) == 5);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetFilterCategory::VertexVertexAxisSeparated) == 6);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetPrismSeparationAxis::None) == 0);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetPrismSeparationAxis::FaceNormal) == 1);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetPrismSeparationAxis::EdgeCross) == 2);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetPrismSeparationAxis::VertexEdge) == 3);
static_assert(static_cast<unsigned>(
    c::SelfContactFacetPrismSeparationAxis::VertexVertex) == 4);
static_assert(offsetof(
    c::SelfContactCandidatePolicySummary, digest) == 96);
static_assert(offsetof(
    c::SelfContactCandidatePolicySummary, complete) == 104);
static_assert(offsetof(
    c::SelfContactCandidatePolicySummary,
    vertex_edge_axis_separated) == 112);
static_assert(offsetof(
    c::SelfContactCandidatePolicySummary,
    vertex_vertex_axis_separated) == 120);
static_assert(sizeof(c::SelfContactCandidatePolicySummary) == 176);
static_assert(
    std::is_trivially_copyable_v<c::SelfContactForceEventIdentity>);
static_assert(sizeof(c::SelfContactForceEventIdentity) == 240);

using LegacyPrismCertificate = bool (*)(
    const c::CurrentFixedTriangle&, const c::CurrentFixedTriangle&, double,
    const c::CurrentFixedTriangle&, const c::CurrentFixedTriangle&, double,
    bool, c::SelfContactFacetPrismSeparationAxis*, bool*) noexcept;
using ExtendedPrismCertificate = bool (*)(
    const c::CurrentFixedTriangle&, const c::CurrentFixedTriangle&, double,
    const c::CurrentFixedTriangle&, const c::CurrentFixedTriangle&, double,
    c::SelfContactFacetPrismAxisLimit,
    c::SelfContactFacetPrismSeparationAxis*, bool*) noexcept;
constexpr LegacyPrismCertificate LegacyPrismCertificateEntry =
    static_cast<LegacyPrismCertificate>(
        &c::CertifiedLinearFacetPrismSeparation);
constexpr ExtendedPrismCertificate ExtendedPrismCertificateEntry =
    static_cast<ExtendedPrismCertificate>(
        &c::CertifiedLinearFacetPrismSeparation);
static_assert(LegacyPrismCertificateEntry != nullptr);
static_assert(ExtendedPrismCertificateEntry != nullptr);

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

c::FacetEdgeKey Edge(std::uint64_t first,std::uint64_t second,
                     std::uint64_t parent) {
  c::FacetEdgeKey result;
  result.parent_eid=parent;
  result.endpoints[0]=Vertex(first);
  result.endpoints[1]=Vertex(second);
  if (c::fixed_triangle_features::Compare(
          result.endpoints[1],result.endpoints[0]) < 0)
    std::swap(result.endpoints[0],result.endpoints[1]);
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

c::FixedContactFacet TopologyFacet(
    std::uint64_t eid, unsigned local,
    std::array<std::uint64_t, 3> vertices) {
  auto result = Facet(eid, local);
  for (unsigned i = 0; i < 3; ++i)
    result.vertex_keys[i] = Vertex(vertices[i]);
  for (unsigned i = 0; i < 3; ++i) {
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

sct::AcceptedEventCertificate EdgeCertificate() {
  sct::AcceptedEventCertificate result;
  result.kind=sct::AcceptedEventCertificateKind::EdgeEdge;
  result.event.feature.SetEdgeEdge();
  result.event.feature.edge_edge.edges[0]=Edge(1,2,10);
  result.event.feature.edge_edge.edges[1]=Edge(3,4,20);
  result.event.source_order=0;
  result.event.edge_use[0]=4;
  result.event.edge_use[1]=5;
  for (unsigned endpoint=0;endpoint<2;++endpoint) {
    result.event.endpoints[endpoint].count=3;
    result.event.endpoints[endpoint].nodes[0]=3*endpoint;
    result.event.endpoints[endpoint].nodes[1]=3*endpoint+1;
    result.event.endpoints[endpoint].nodes[2]=3*endpoint+2;
    result.event.endpoints[endpoint].weights[0]=.5;
    result.event.endpoints[endpoint].weights[1]=.5;
  }
  auto& pair=result.event.classification;
  pair.kind=c::SelfContactPairKind::EdgeEdge;
  pair.edge_edge_case=
      c::SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum;
  pair.status=c::SelfContactPairStatus::AdmittedEdgeEdge;
  pair.active[0]=pair.active[1]=true;
  pair.reference_half_thickness_m[0]=.1;
  pair.reference_half_thickness_m[1]=.1;
  pair.candidate_directed_area_m2={1,1,1,0};
  pair.admitted_force_area_m2={1,1,1,0};
  result.discovery.key=result.event.feature;
  result.discovery.triangles[0]={17,10,0,0};
  result.discovery.triangles[1]={17,20,0,0};
  result.discovery.local_features[0]=0;
  result.discovery.local_features[1]=1;
  result.discovery.edge_parameters[0]=.5;
  result.discovery.edge_parameters[1]=.5;
  result.discovery.distance_m=.1;
  result.edge_facet[0]=1;
  result.edge_facet[1]=2;
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
  }
  for (unsigned i = 0; i < 3; ++i) {
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

struct PreparedPairFeatures {
  std::array<c::FixedTriangleFeatureCandidate, 15> values{};
  std::size_t count = 0;
  std::array<c::FixedTriangleIntersection, 1> intersections{};
  std::size_t intersection_count = 0;
};

PreparedPairFeatures DiscoverPreparedPair(
    const c::CurrentFixedTriangle& first,
    const c::CurrentFixedTriangle& second) {
  PreparedPairFeatures output;
  c::fixed_triangle_features::PairFeatureResult features;
  EXPECT_EQ(
      c::fixed_triangle_features::EvaluatePairFeaturesOnce(
          first, second, output.values.data(),
          output.values.size(), &features),
      c::FixedTriangleDiscoveryStatus::Ok);
  output.count = features.feature_count;
  bool intersects = false;
  EXPECT_EQ(
      c::fixed_triangle_features::ClassifyPairIntersection(
          first, second, output.intersections.data(),
          &intersects),
      c::FixedTriangleDiscoveryStatus::Ok);
  output.intersection_count = intersects ? 1 : 0;
  return output;
}

sct::LinearResidualSeparationResult ResidualCertificate(
    const c::CurrentFixedTriangle& first_base,
    const c::CurrentFixedTriangle& first_prepared,
    const c::CurrentFixedTriangle& second_base,
    const c::CurrentFixedTriangle& second_prepared,
    double half_thickness = .1) {
  const auto geometry =
      DiscoverPreparedPair(first_prepared, second_prepared);
  return sct::CertifyLinearResidualSeparation(
      first_base, first_prepared, half_thickness,
      second_base, second_prepared, half_thickness,
      {geometry.values.data(), geometry.count, true},
      {geometry.intersections.data(),
       geometry.intersection_count, true});
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
     ResidualTranslationUsesOutwardBoundsAfterCancellation) {
  const double translation = std::ldexp(1.0, -52);
  const auto first_base = Triangle(
      10, {1, 2, 3},
      {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto second_base = Triangle(
      20, {4, 5, 6},
      {{{0, 0, 1}, {2, 0, 1}, {0, 2, 1}}});
  auto first_prepared = first_base;
  auto second_prepared = second_base;
  for (auto* triangle : {&first_prepared, &second_prepared})
    for (auto& vertex : triangle->vertices)
      vertex.x += translation;

  const auto result = ResidualCertificate(
      first_base, first_prepared,
      second_base, second_prepared);
  EXPECT_EQ(
      result.status,
      sct::LinearResidualSeparationStatus::
          CertifiedSeparated);
  EXPECT_EQ(result.reference_translation.x, translation);
  EXPECT_FALSE(result.exact_common_translation);
  EXPECT_GE(result.first_residual_upper_m, translation);
  EXPECT_GE(result.second_residual_upper_m, translation);
  EXPECT_GT(result.prepared_distance_lower_m, .99);
  EXPECT_GT(result.strict_gap_lower_m, .79);
}

TEST(SelfContactTransactionValues,
     ResidualTranslationRetainsSubnormalExactMotion) {
  const double translation =
      std::numeric_limits<double>::denorm_min();
  const auto first_base = Triangle(
      10, {1, 2, 3},
      {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto second_base = Triangle(
      20, {4, 5, 6},
      {{{0, 0, 1}, {2, 0, 1}, {0, 2, 1}}});
  auto first_prepared = first_base;
  auto second_prepared = second_base;
  for (auto* triangle : {&first_prepared, &second_prepared})
    for (auto& vertex : triangle->vertices)
      vertex.x += translation;

  const auto result = ResidualCertificate(
      first_base, first_prepared,
      second_base, second_prepared);
  EXPECT_EQ(
      result.status,
      sct::LinearResidualSeparationStatus::
          CertifiedSeparated);
  EXPECT_FALSE(result.exact_common_translation);
  EXPECT_GT(result.first_residual_upper_m, 0);
  EXPECT_GT(result.second_residual_upper_m, 0);
}

TEST(SelfContactTransactionValues,
     ResidualTranslationIsInvariantToVertexPermutation) {
  const double translation = std::ldexp(1.0, -52);
  const auto first_base = Triangle(
      10, {1, 2, 3},
      {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto second_base = Triangle(
      20, {4, 5, 6},
      {{{0, 0, 1}, {2, 0, 1}, {0, 2, 1}}});
  auto first_prepared = first_base;
  auto second_prepared = second_base;
  for (auto* triangle : {&first_prepared, &second_prepared})
    for (auto& vertex : triangle->vertices)
      vertex.x += translation;
  const auto original = ResidualCertificate(
      first_base, first_prepared,
      second_base, second_prepared);

  const auto permute = [](c::CurrentFixedTriangle value) {
    const auto old = value;
    constexpr unsigned order[3]{1, 2, 0};
    for (unsigned i = 0; i < 3; ++i) {
      value.vertices[i] = old.vertices[order[i]];
      value.vertex_keys[i] = old.vertex_keys[order[i]];
      value.edge_keys[i] = old.edge_keys[order[i]];
    }
    return value;
  };
  const auto permuted = ResidualCertificate(
      permute(first_base), permute(first_prepared),
      permute(second_base), permute(second_prepared));
  EXPECT_EQ(
      original.status,
      sct::LinearResidualSeparationStatus::
          CertifiedSeparated);
  EXPECT_EQ(permuted.status, original.status);
  EXPECT_GT(permuted.strict_gap_lower_m, 0);
  EXPECT_GT(original.strict_gap_lower_m, 0);
}

TEST(SelfContactTransactionValues,
     ResidualTranslationFailsClosedOnOverflow) {
  auto first_base = Triangle(
      10, {1, 2, 3},
      {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  auto first_prepared = first_base;
  const auto second = Triangle(
      20, {4, 5, 6},
      {{{0, 0, 1}, {2, 0, 1}, {0, 2, 1}}});
  first_base.vertices[0].x =
      -std::numeric_limits<double>::max();
  first_prepared.vertices[0].x =
      std::numeric_limits<double>::max();
  const auto result = sct::CertifyLinearResidualSeparation(
      first_base, first_prepared, .1,
      second, second, .1,
      {nullptr, 0, true}, {nullptr, 0, true});
  EXPECT_EQ(
      result.status,
      sct::LinearResidualSeparationStatus::InvalidInput);
}

TEST(SelfContactTransactionValues,
     ResidualTranslationPreservesContactAndUnequalMotion) {
  const auto first = Triangle(
      10, {1, 2, 3},
      {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto close_second = Triangle(
      20, {4, 5, 6},
      {{{0, 0, .15}, {2, 0, .15}, {0, 2, .15}}});
  EXPECT_EQ(
      ResidualCertificate(
          first, first, close_second, close_second)
          .status,
      sct::LinearResidualSeparationStatus::PotentialContact);

  const auto second_base = Triangle(
      20, {4, 5, 6},
      {{{0, 0, 2}, {2, 0, 2}, {0, 2, 2}}});
  const auto second_prepared = Triangle(
      20, {4, 5, 6},
      {{{0, 0, 1}, {2, 0, 1}, {0, 2, 1}}});
  EXPECT_EQ(
      ResidualCertificate(
          first, first, second_base, second_prepared)
          .status,
      sct::LinearResidualSeparationStatus::PotentialContact);
}

TEST(SelfContactTransactionValues,
     ResidualTranslationSubtractsRepresentationErrorStrictly) {
  const auto first = Triangle(
      10, {1, 2, 3},
      {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto second = Triangle(
      20, {4, 5, 6},
      {{{0, 0, 1}, {2, 0, 1}, {0, 2, 1}}});
  auto geometry = DiscoverPreparedPair(first, second);
  ASSERT_EQ(geometry.count, 15u);
  for (auto& feature : geometry.values)
    feature.representation_error_m = .79;
  EXPECT_EQ(
      sct::CertifyLinearResidualSeparation(
          first, first, .1, second, second, .1,
          {geometry.values.data(), geometry.count, true},
          {nullptr, 0, true}).status,
      sct::LinearResidualSeparationStatus::
          CertifiedSeparated);
  for (auto& feature : geometry.values)
    feature.representation_error_m = .8;
  EXPECT_EQ(
      sct::CertifyLinearResidualSeparation(
          first, first, .1, second, second, .1,
          {geometry.values.data(), geometry.count, true},
          {nullptr, 0, true}).status,
      sct::LinearResidualSeparationStatus::PotentialContact);
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
  EXPECT_EQ(layout.chunk_feature_task_masks.count, 3u);
  EXPECT_EQ(layout.chunk_feature_task_masks.bytes,
            3 * sizeof(c::FixedTriangleFeatureTaskMask));
  EXPECT_EQ(layout.accepted_event_identities.count, 8u);
  EXPECT_EQ(layout.accepted_events.count, 7u);
  EXPECT_EQ(layout.accepted_certificates.count, 7u);
  EXPECT_EQ(layout.accepted_event_hash.count, 16u);
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
     AuthenticatedLocalTaskMasksAreCanonicalAndCapacityAtomic) {
  const c::FixedContactFacet facets[]{
      TopologyFacet(100, 0, {1, 2, 3}),
      TopologyFacet(200, 0, {12, 13, 1}),
      TopologyFacet(100, 1, {21, 22, 23})};
  const c::FixedTrianglePair pairs[]{{1, 0}, {0, 2}};
  std::array<c::FixedTriangleFeatureTaskMask, 2> masks{{
      {0x1234u}, {0x5678u}}};

  auto report = sct::BuildLocalFeatureTaskMasks(
      facets, std::size(facets), pairs, std::size(pairs),
      masks.data(), masks.size() - 1);
  EXPECT_EQ(report.status, c::SelfContactTransactionStatus::ResourceLimit);
  EXPECT_EQ(masks[0].local_tasks, 0x1234u);
  EXPECT_EQ(masks[1].local_tasks, 0x5678u);

  report = sct::BuildLocalFeatureTaskMasks(
      facets, std::size(facets), pairs, std::size(pairs),
      masks.data(), masks.size());
  ASSERT_EQ(report.status, c::SelfContactTransactionStatus::Ok);
  const std::uint16_t shared_vertex =
      c::FixedTriangleFeatureTaskBit(0) |
      c::FixedTriangleFeatureTaskBit(5) |
      c::FixedTriangleFeatureTaskBit(7) |
      c::FixedTriangleFeatureTaskBit(8) |
      c::FixedTriangleFeatureTaskBit(13) |
      c::FixedTriangleFeatureTaskBit(14);
  EXPECT_EQ(masks[0].local_tasks, shared_vertex);
  // Same source parent alone is not local incidence.
  EXPECT_EQ(masks[1].local_tasks, 0u);

  const c::FixedTrianglePair reversed[]{{0, 1}};
  c::FixedTriangleFeatureTaskMask orientation;
  ASSERT_EQ(sct::BuildLocalFeatureTaskMasks(
      facets, std::size(facets), reversed, 1, &orientation, 1).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(orientation.local_tasks, masks[0].local_tasks);
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
      96ull << 30, 4, 4);
  EXPECT_EQ(limits.broadphase.max_pairs, 1584464u);
  EXPECT_EQ(limits.max_candidate_pairs, 5989248u);
  EXPECT_EQ(limits.max_facet_pair_chunk, chunk);
  EXPECT_EQ(limits.accepted_discovery.max_raw_feature_candidates,
            15 * chunk);
  EXPECT_EQ(limits.accepted_discovery.worker_count, 4u);
  EXPECT_EQ(limits.candidate_discovery.worker_count, 4u);
  EXPECT_EQ(limits.crossing.max_paths, 2 * chunk);
  EXPECT_EQ(limits.crossing.worker_count, 4u);
  EXPECT_EQ(
      limits.max_nonlinear_subdivision_work_per_pair, 4095u);
  EXPECT_EQ(
      limits.max_nonlinear_subdivision_work_per_chunk,
      per_chunk_work);
  EXPECT_EQ(
      limits.max_stream_nonlinear_subdivision_work,
      complete_work);
  EXPECT_EQ(limits.max_nonlinear_subdivision_depth, 20u);
  EXPECT_EQ(limits.max_policy_outcomes, 0u);
  const auto compatible_default =
      c::SelfContactTransactionLimits::Vehicle(
          census, chunk, 1, 2, 0, 4095, per_chunk_work,
          complete_work, 20, 64ull << 30, 8ull << 30,
          96ull << 30, 4);
  EXPECT_EQ(compatible_default.accepted_discovery.worker_count, 4u);
  EXPECT_EQ(compatible_default.crossing.worker_count, 1u);
  const auto compact_census =
      c::SelfContactTransactionLimits::Vehicle(
          census, chunk, 1, 16000000, 0, 4095,
          per_chunk_work, complete_work, 20,
          64ull << 30, 8ull << 30, 96ull << 30,
          4, 4, 8000000);
  EXPECT_EQ(compact_census.max_global_events, 1u);
  EXPECT_EQ(
      compact_census.max_event_identity_census, 8000000u);
  EXPECT_EQ(compact_census.max_event_hash_slots, 16000000u);
  EXPECT_EQ(compact_census.force.max_events, 1u);
  auto over_force_census = census;
  over_force_census.accepted_events = 2;
  const auto count_before_force =
      c::SelfContactTransactionLimits::Vehicle(
          over_force_census, chunk, 1, 16, 0, 4095,
          per_chunk_work, complete_work, 20,
          64ull << 30, 8ull << 30, 96ull << 30,
          4, 4, 8);
  EXPECT_NE(count_before_force.max_host_bytes, 0u);
  EXPECT_EQ(count_before_force.max_global_events, 1u);
  EXPECT_EQ(count_before_force.max_event_identity_census, 8u);

  auto overflow = census;
  overflow.facet_pairs = SIZE_MAX / 15 + 1;
  const auto rejected = c::SelfContactTransactionLimits::Vehicle(
      overflow, chunk, 1, 2, 0, 4095, per_chunk_work,
      complete_work, 20, 64ull << 30, 8ull << 30,
      96ull << 30);
  EXPECT_EQ(rejected.max_host_bytes, 0u);
  const auto bad_workers = c::SelfContactTransactionLimits::Vehicle(
      census, chunk, 1, 2, 0, 4095, per_chunk_work,
      complete_work, 20, 64ull << 30, 8ull << 30,
      96ull << 30, 9);
  EXPECT_EQ(bad_workers.max_host_bytes, 0u);
  const auto bad_crossing_workers =
      c::SelfContactTransactionLimits::Vehicle(
          census, chunk, 1, 2, 0, 4095, per_chunk_work,
          complete_work, 20, 64ull << 30, 8ull << 30,
          96ull << 30, 4, 9);
  EXPECT_EQ(bad_crossing_workers.max_host_bytes, 0u);

  sct::Layout minimum;
  ASSERT_TRUE(sct::MakeLayout(
      census.nodes, census.surface_parents,
      census.selected_parents, census.facets,
      779, census.parent_pairs, chunk, 1, 1, 2, 0,
      SIZE_MAX, minimum));
  sct::Layout eight_million_identity_census;
  ASSERT_TRUE(sct::MakeLayout(
      census.nodes, census.surface_parents,
      census.selected_parents, census.facets,
      779, census.parent_pairs, chunk,
      1, 8000000, 16000000, 0,
      SIZE_MAX, eight_million_identity_census));
  EXPECT_GT(minimum.bytes, 0u);
  EXPECT_GT(eight_million_identity_census.bytes, minimum.bytes);
  const auto exact_identity_bytes =
      eight_million_identity_census.bytes;
  auto unchanged = eight_million_identity_census;
  EXPECT_FALSE(sct::MakeLayout(
      census.nodes, census.surface_parents,
      census.selected_parents, census.facets,
      779, census.parent_pairs, chunk,
      1, 8000000, 16000000, 0,
      exact_identity_bytes - 1, unchanged));
  EXPECT_EQ(unchanged.bytes, exact_identity_bytes);
  std::cout << "V5_STREAMING_ARENA minimum_event_bytes="
            << minimum.bytes
            << " eight_million_identity_bytes="
            << eight_million_identity_census.bytes
            << " event_identity_size="
            << sizeof(c::SelfContactForceEventIdentity)
            << " parent_key_bytes="
            << census.parent_pairs * sizeof(c::SelfContactPairKey)
            << " cursor_bytes="
            << census.parent_pairs * sizeof(sct::FacetPairCursor)
            << " heap_bytes="
            << census.parent_pairs * sizeof(std::uint32_t)
            << " chunk_pair_bytes="
            << chunk * sizeof(c::FixedTrianglePair)
            << " chunk_task_mask_bytes="
            << chunk * sizeof(c::FixedTriangleFeatureTaskMask)
            << '\n';
}

TEST(SelfContactTransactionValues,
     PairMotionIsScopedAndRigidBoxesOnlyProveSeparation) {
  const c::SelfContactSweptParentBounds near{
      {-1, -1, -1}, {1, 1, 1}};
  const c::SelfContactSweptParentBounds far{
      {3, -1, -1}, {4, 1, 1}};
  const c::SelfContactSweptParentBounds touching{
      {1, -1, -1}, {2, 1, 1}};
  sct::MotionSupport ordinary;
  ordinary.certified_affine = true;
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
  auto affine_partial = partial;
  affine_partial.certified_affine = true;

  // An unrelated rigid facet elsewhere cannot change an ordinary pair.
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      ordinary, near, ordinary, near),
      sct::PairMotionAction::LinearNodalV1);
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      ordinary, near, ordinary, touching),
      sct::PairMotionAction::LinearNodalV1);
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      ordinary, near, ordinary, far),
      sct::PairMotionAction::CertifiedLinearSeparation);
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
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      affine_partial, near, ordinary, near),
      sct::PairMotionAction::LinearNodalV1);
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      affine_partial, near, ordinary, far),
      sct::PairMotionAction::CertifiedLinearSeparation);
  auto affine_same_rigid = rigid_a;
  affine_same_rigid.certified_affine = true;
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      affine_same_rigid, near, affine_same_rigid, near),
      sct::PairMotionAction::ExcludedSameRigidGroup);

  // Numeric source-ID collisions do not merge different actual group rows.
  rigid_b.complete_rigid_group = 8;
  rigid_b.rigid_groups[0] = 8;
  EXPECT_EQ(sct::ClassifyCandidatePairMotion(
      rigid_a, near, rigid_b, near),
      sct::PairMotionAction::UnsupportedRigidArc);
}

TEST(SelfContactTransactionValues,
     AcceptedFacetFilterCategoriesAreDisjointAndFailClosed) {
  constexpr double thickness = 0.01;
  auto first = Triangle(
      10, {1, 2, 3},
      {{{0, 0, 0}, {1, 0, 1}, {0, 1, 1}}});
  auto aabb = first;
  auto face = first;
  auto edge = first;
  auto vertex_edge_first = first;
  auto vertex_edge_second = first;
  auto vertex_vertex_first = first;
  auto vertex_vertex_second = first;
  auto touching = Triangle(
      20, {4, 5, 6},
      {{{0, 0, 0.02}, {1, 0, 0.02}, {0, 1, 0.02}}});
  for (auto& vertex : aabb.vertices)
    vertex.x += 2;
  for (auto& vertex : face.vertices) {
    vertex.x -= 0.0625;
    vertex.y -= 0.0625;
    vertex.z += 0.0625;
  }
  edge.vertices[0] = {-0.25, -1, -0.5};
  edge.vertices[1] = {0.25, -0.5, -0.5};
  edge.vertices[2] = {1, 1, 0.25};
  const c::Vec3 vertex_edge_first_points[3]{
      {-0.7617271175557994, -0.9854918156065067,
       -0.5687134555287552},
      {-0.7158959051133422, 0.4952591442811507,
       -0.938635023308946},
      {-0.6630144249741934, -0.3375527983865929,
       -0.4822709565245622},
  };
  const c::Vec3 vertex_edge_second_points[3]{
      {-0.699936476378623, 0.07336418345847506,
       -1.4577573753456794},
      {-0.5297783238983161, -1.0961427534549402,
       -0.9422850565170009},
      {-0.7545209025989622, -0.7210003196383497,
       -0.6793528800202036},
  };
  const c::Vec3 vertex_vertex_first_points[3]{
      {0, 0, 0},
      {-0.05620322369963638, 0.016125484251891486,
       0.10471122170959381},
      {-0.054441030886089786, -0.15764697103506448,
       -0.16959380631479207},
  };
  const c::Vec3 vertex_vertex_second_points[3]{
      {0.022402691787493437, 0.020240403023466674,
       0.0057644742154427343},
      {0.0016407555274286429, 0.10513512125221264,
       0.17366994826653179},
      {0.16934954727284365, 0.12577252251265236,
       0.0056955278101172291},
  };
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    vertex_edge_first.vertices[vertex] =
        vertex_edge_first_points[vertex];
    vertex_edge_second.vertices[vertex] =
        vertex_edge_second_points[vertex];
    vertex_vertex_first.vertices[vertex] =
        vertex_vertex_first_points[vertex];
    vertex_vertex_second.vertices[vertex] =
        vertex_vertex_second_points[vertex];
  }
  auto planar = Triangle(
      30, {7, 8, 9},
      {{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}});

  const std::array<c::SelfContactFacetFilterResult, 7> results{{
      c::ClassifyAcceptedFacetPair(
          first, thickness, 7, first, thickness, 7),
      c::ClassifyAcceptedFacetPair(
          first, thickness, UINT32_MAX, aabb, thickness, UINT32_MAX),
      c::ClassifyAcceptedFacetPair(
          first, thickness, UINT32_MAX, face, thickness, UINT32_MAX),
      c::ClassifyAcceptedFacetPair(
          first, thickness, UINT32_MAX, edge, thickness, UINT32_MAX),
      c::ClassifyAcceptedFacetPair(
          vertex_edge_first, thickness, UINT32_MAX,
          vertex_edge_second, thickness, UINT32_MAX),
      c::ClassifyAcceptedFacetPair(
          vertex_vertex_first, thickness, UINT32_MAX,
          vertex_vertex_second, thickness, UINT32_MAX),
      c::ClassifyAcceptedFacetPair(
          planar, thickness, UINT32_MAX,
          touching, thickness, UINT32_MAX),
  }};
  std::array<std::size_t, 7> counts{};
  for (const auto result : results) {
    ASSERT_EQ(result.status, c::SelfContactFacetFilterStatus::Ok);
    ++counts[static_cast<unsigned>(result.category)];
  }
  for (const auto count : counts)
    EXPECT_EQ(count, 1u);

  auto nonfinite = first;
  nonfinite.vertices[0].x =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(c::ClassifyAcceptedFacetPair(
      nonfinite, thickness, UINT32_MAX,
      first, thickness, UINT32_MAX).status,
      c::SelfContactFacetFilterStatus::InvalidInput);
  EXPECT_EQ(c::ClassifyAcceptedFacetPair(
      first, std::numeric_limits<double>::infinity(), UINT32_MAX,
      first, thickness, UINT32_MAX).status,
      c::SelfContactFacetFilterStatus::InvalidInput);
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
  event.discovery.triangles[0] = {17,10,0,0};
  event.discovery.triangles[1] = {17,20,0,0};
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

  auto unrelated = event;
  unrelated.event.classification.parent[0] = 1;
  unrelated.event.classification.parent[1] = 2;
  unrelated.discovery.triangles[0].parent_eid = 30;
  event.event.classification.parent[0] = 3;
  event.event.classification.parent[1] = 4;
  event.event.source_order = 1;
  std::array<sct::AcceptedEventCertificate, 2> owners{
      unrelated, event};
  input.accepted_events = owners.data();
  input.accepted_event_count = owners.size();
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(outcome.accepted_event, 1u);
  EXPECT_EQ(outcome.source_order, 1u);
  event.event.classification.parent[0] = 0;
  event.event.classification.parent[1] = 0;
  event.event.source_order = 0;
  input.accepted_events = &event;
  input.accepted_event_count = 1;

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

  // Parent-pair equality is insufficient for boundary-EE deduplication by an
  // admitted VF: its producing fixed-facet pair must also be exact.
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
     EeCrossingRequiresExactAcceptedCanonicalFeature) {
  const auto pair=Pair(10,20);
  auto certificate=EdgeCertificate();
  c::RepresentedIntervalResult result;
  result.key=pair;
  result.classification=
      c::RepresentedIntervalClassification::CertifiedCrossingContact;
  result.feature.kind=c::RepresentedFeatureKind::EdgeEdge;
  result.feature.edges[0]=certificate.event.feature.edge_edge.edges[0];
  result.feature.edges[1]=certificate.event.feature.edge_edge.edges[1];
  c::SelfContactCandidatePolicyOutcome outcome;
  std::size_t count=0;
  auto input=Input(&pair,1,&result,&outcome,&count);
  input.accepted_events=&certificate;
  input.accepted_event_count=1;
  ASSERT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(outcome.disposition,
      c::SelfContactCandidateDisposition::RepresentedByAcceptedEdgeEdge);
  EXPECT_EQ(outcome.accepted_event,0u);

  auto unrelated=certificate;
  unrelated.event.classification.parent[0]=1;
  unrelated.event.classification.parent[1]=2;
  unrelated.discovery.triangles[0].parent_eid=30;
  unrelated.discovery.triangles[1].parent_eid=40;
  certificate.event.classification.parent[0]=3;
  certificate.event.classification.parent[1]=4;
  certificate.event.source_order=1;
  std::array<sct::AcceptedEventCertificate,2> owners{
      unrelated,certificate};
  input.accepted_events=owners.data();
  input.accepted_event_count=owners.size();
  ASSERT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(outcome.accepted_event,1u);
  certificate.event.classification.parent[0]=0;
  certificate.event.classification.parent[1]=0;
  certificate.event.source_order=0;
  input.accepted_events=&certificate;
  input.accepted_event_count=1;

  certificate.discovery.triangles[0].local_facet=1;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::Ok);
  certificate.discovery.triangles[0].local_facet=0;
  const auto edge=certificate.event.feature.edge_edge.edges[0];
  certificate.event.feature.edge_edge.edges[0].endpoints[0].first++;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);
  certificate.event.feature.edge_edge.edges[0]=edge;
  certificate.kind=sct::AcceptedEventCertificateKind::VertexFace;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);
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

  result.classification =
      c::RepresentedIntervalClassification::Unresolved;
  result.reason = c::RepresentedIntervalReason::UnsupportedMotion;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(outcome.disposition,
      c::SelfContactCandidateDisposition::ExcludedLocalIntersection);

  result.reason = c::RepresentedIntervalReason::WorkExhausted;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::UnresolvedCandidate);

  result.classification =
      c::RepresentedIntervalClassification::CertifiedCrossingContact;
  result.reason = c::RepresentedIntervalReason::None;
  intersection.local_exclusion =
      c::FixedTriangleLocalExclusion::None;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::CandidateRejected);

  result.classification =
      c::RepresentedIntervalClassification::Unresolved;
  result.reason = c::RepresentedIntervalReason::UnsupportedMotion;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
      c::SelfContactTransactionStatus::UnresolvedCandidate);
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
     CompactIdentityCensusDeduplicatesOwnersAndResolvesHashCollisions) {
  auto event = Certificate(100).event;
  event.classification.parent[0] = 2;
  event.classification.parent[1] = 3;
  const auto identity = c::SelfContactForceEventIdentityOf(event);
  EXPECT_EQ(c::CompareSelfContactForceEventIdentity(
                identity,
                c::SelfContactForceEventIdentityOf(event)),
            0);

  std::array<int, 3> first_owner{{-1, -1, -1}};
  c::SelfContactForceEventIdentity collision[2];
  bool found = false;
  for (std::uint32_t owner = 0; owner < 100 && !found; ++owner) {
    auto candidate = identity;
    candidate.parent[0] = owner;
    std::uint64_t value = 1469598103934665603ull;
    c::HashSelfContactForceEventIdentity(candidate, &value);
    const auto bucket = value % first_owner.size();
    if (first_owner[bucket] >= 0) {
      collision[0] = identity;
      collision[0].parent[0] =
          static_cast<std::uint32_t>(first_owner[bucket]);
      collision[1] = candidate;
      found = true;
    } else {
      first_owner[bucket] = static_cast<int>(owner);
    }
  }
  ASSERT_TRUE(found);
  ASSERT_NE(collision[0].parent[0], collision[1].parent[0]);

  std::array<c::SelfContactForceEventIdentity, 2> census;
  std::array<std::uint32_t, 3> hash;
  hash.fill(UINT32_MAX);
  std::size_t count = 0;
  ASSERT_EQ(sct::MergeAcceptedEventIdentityChunk(
      collision, 1, census.data(), census.size(),
      hash.data(), hash.size(), &count).status,
      c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(sct::MergeAcceptedEventIdentityChunk(
      collision + 1, 1, census.data(), census.size(),
      hash.data(), hash.size(), &count).status,
      c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(sct::MergeAcceptedEventIdentityChunk(
      collision, 1, census.data(), census.size(),
      hash.data(), hash.size(), &count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(count, 2u);
}

TEST(SelfContactTransactionValues,
     CensusLowerBoundAndFullVerificationKeepDifferentSemantics) {
  std::array<c::SelfContactForceEventIdentity, 2> census;
  std::array<std::uint32_t, 3> identity_hash;
  identity_hash.fill(UINT32_MAX);
  std::size_t count = 0;
  auto certificate = Certificate(100);
  for (std::uint32_t owner = 0; owner < 2; ++owner) {
    certificate.event.classification.parent[0] = owner;
    const auto identity =
        c::SelfContactForceEventIdentityOf(certificate.event);
    ASSERT_EQ(sct::MergeAcceptedEventIdentityChunk(
        &identity, 1, census.data(), census.size(),
        identity_hash.data(), identity_hash.size(), &count).status,
        c::SelfContactTransactionStatus::Ok);
  }
  certificate.event.classification.parent[0] = 2;
  const auto overflow_identity =
      c::SelfContactForceEventIdentityOf(certificate.event);
  const auto overflow = sct::MergeAcceptedEventIdentityChunk(
      &overflow_identity, 1, census.data(), census.size(),
      identity_hash.data(), identity_hash.size(), &count);
  EXPECT_EQ(overflow.status,
            c::SelfContactTransactionStatus::ResourceLimit);
  EXPECT_EQ(overflow.candidate, 3u);
  EXPECT_EQ(overflow.count_kind,
            c::SelfContactTransactionCountKind::
                AcceptedEventsLowerBound);
  EXPECT_EQ(count, 2u);

  certificate = Certificate(100);
  certificate.event.classification.parent[0] = 7;
  certificate.event.classification.parent[1] = 8;
  auto forged = certificate;
  forged.event.classification.admitted_force_area_m2.value = 3;
  auto one_identity =
      c::SelfContactForceEventIdentityOf(certificate.event);
  ASSERT_EQ(sct::CanonicalizeAcceptedEventIdentityCensus(
      &one_identity, 1).status,
      c::SelfContactTransactionStatus::Ok);
  const std::array<sct::AcceptedEventCertificate, 2> verification{
      certificate, forged};
  EXPECT_EQ(sct::VerifyAcceptedEventIdentityChunk(
      verification.data(), verification.size(), &one_identity, 1).status,
      c::SelfContactTransactionStatus::Ok);

  std::array<sct::AcceptedEventCertificate, 1> full_ledger;
  std::array<std::uint32_t, 2> full_hash;
  full_hash.fill(UINT32_MAX);
  std::size_t full_count = 0;
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &certificate, 1, full_ledger.data(), full_ledger.size(),
      full_hash.data(), full_hash.size(), &full_count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(sct::MergeAcceptedEventChunk(
      &forged, 1, full_ledger.data(), full_ledger.size(),
      full_hash.data(), full_hash.size(), &full_count).status,
      c::SelfContactTransactionStatus::IdentityMismatch);
}

TEST(SelfContactTransactionValues,
     MillionSyntheticIdentitiesUseBoundedCompactStorage) {
  constexpr std::size_t identity_count = 1000000;
  constexpr std::size_t hash_count = 2000000;
  std::vector<c::SelfContactForceEventIdentity> census(identity_count);
  std::vector<std::uint32_t> hash(hash_count, UINT32_MAX);
  c::SelfContactForceEventIdentity identity;
  identity.feature = Feature();
  identity.parent[1] = 1000001;
  std::size_t count = 0;
  c::SelfContactTransactionReport report;
  for (std::uint32_t owner = 0; owner < identity_count; ++owner) {
    identity.parent[0] = owner;
    report = sct::MergeAcceptedEventIdentityChunk(
        &identity, 1, census.data(), census.size(),
        hash.data(), hash.size(), &count);
    if (report.status != c::SelfContactTransactionStatus::Ok)
      break;
  }
  ASSERT_EQ(report.status, c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(count, identity_count);
  EXPECT_LT(census.size() * sizeof(census[0]) +
                hash.size() * sizeof(hash[0]),
            256ull << 20);
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
  EXPECT_EQ(short_cap.count_kind,
            c::SelfContactTransactionCountKind::
                ExactAcceptedEvents);
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
     EventIdentityRetainsOwnersAndDedupsOnlyExactOwnerEvents) {
  auto high=EdgeCertificate();
  high.event.classification.parent[0]=4;
  high.event.classification.parent[1]=5;
  auto low=high;
  low.event.classification.parent[0]=2;
  low.event.classification.parent[1]=3;
  low.event.classification.candidate_directed_area_m2={2,2,2,0};
  low.event.classification.admitted_force_area_m2={2,2,2,0};
  std::array<sct::AcceptedEventCertificate,2> ledger{};
  std::array<std::uint32_t,7> hash;
  hash.fill(UINT32_MAX);
  std::size_t count=0;
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &high,1,ledger.data(),ledger.size(),
      hash.data(),hash.size(),&count).status,
      c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &low,1,ledger.data(),ledger.size(),
      hash.data(),hash.size(),&count).status,
      c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(count,2u);
  std::array<c::SelfContactForceEvent,2> events;
  ASSERT_EQ(sct::FinalizeAcceptedEventLedger(
      ledger.data(),count,events.data(),events.size()).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(events[0].classification.parent[0],2u);
  EXPECT_EQ(events[0].classification.admitted_force_area_m2.value,2);
  EXPECT_EQ(events[0].source_order,0u);
  EXPECT_EQ(events[1].classification.parent[0],4u);
  EXPECT_EQ(events[1].source_order,1u);

  std::array<sct::AcceptedEventCertificate,2> reversed_ledger{};
  std::array<std::uint32_t,7> reversed_hash;
  reversed_hash.fill(UINT32_MAX);
  std::size_t reversed_count=0;
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &low,1,reversed_ledger.data(),reversed_ledger.size(),
      reversed_hash.data(),reversed_hash.size(),
      &reversed_count).status,
      c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &high,1,reversed_ledger.data(),reversed_ledger.size(),
      reversed_hash.data(),reversed_hash.size(),
      &reversed_count).status,
      c::SelfContactTransactionStatus::Ok);
  std::array<c::SelfContactForceEvent,2> reversed_events;
  ASSERT_EQ(sct::FinalizeAcceptedEventLedger(
      reversed_ledger.data(),reversed_count,
      reversed_events.data(),reversed_events.size()).status,
      c::SelfContactTransactionStatus::Ok);
  for (unsigned event=0;event<events.size();++event) {
    EXPECT_EQ(c::CompareSelfContactForceEventIdentity(
        events[event],reversed_events[event]),0);
    EXPECT_EQ(events[event].source_order,
              reversed_events[event].source_order);
    EXPECT_EQ(events[event].classification.
                  admitted_force_area_m2.value,
              reversed_events[event].classification.
                  admitted_force_area_m2.value);
  }

  auto seam=high;
  seam.discovery.triangles[0].local_facet++;
  seam.edge_facet[0]++;
  hash.fill(UINT32_MAX);
  count=0;
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &high,1,ledger.data(),ledger.size(),
      hash.data(),hash.size(),&count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(sct::MergeAcceptedEventChunk(
      &seam,1,ledger.data(),ledger.size(),
      hash.data(),hash.size(),&count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(count,1u);

  auto boundary=Certificate(100);
  boundary.event.feature.vertex_face.target.SetEdge(Edge(6,7,20));
  boundary.discovery.key=boundary.event.feature;
  boundary.event.classification.parent[0]=6;
  boundary.event.classification.parent[1]=7;
  boundary.event.classification.feature[1]=3;
  auto boundary_seam=boundary;
  boundary_seam.event.facet_use=4;
  boundary_seam.event.classification.feature[1]=4;
  boundary_seam.discovery.triangles[1].local_facet=1;
  boundary_seam.target_facet=4;
  hash.fill(UINT32_MAX);
  count=0;
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &boundary,1,ledger.data(),ledger.size(),
      hash.data(),hash.size(),&count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(sct::MergeAcceptedEventChunk(
      &boundary_seam,1,ledger.data(),ledger.size(),
      hash.data(),hash.size(),&count).status,
      c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(count,1u);

  hash.fill(UINT32_MAX);
  count=0;
  ASSERT_EQ(sct::MergeAcceptedEventChunk(
      &high,1,ledger.data(),ledger.size(),
      hash.data(),hash.size(),&count).status,
      c::SelfContactTransactionStatus::Ok);
  auto mismatched=high;
  mismatched.event.classification.admitted_force_area_m2.value=3;
  EXPECT_EQ(sct::MergeAcceptedEventChunk(
      &mismatched,1,ledger.data(),ledger.size(),
      hash.data(),hash.size(),&count).status,
      c::SelfContactTransactionStatus::IdentityMismatch);
  mismatched=high;
  mismatched.event.endpoints[0].weights[0]=.25;
  EXPECT_EQ(sct::MergeAcceptedEventChunk(
      &mismatched,1,ledger.data(),ledger.size(),
      hash.data(),hash.size(),&count).status,
      c::SelfContactTransactionStatus::IdentityMismatch);
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
  std::array<c::SelfContactCandidatePolicyOutcome, 4> values{};
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i].pair = Pair(10 + i, 20 + i);
    values[i].disposition = i == 0
        ? c::SelfContactCandidateDisposition::CertifiedSeparated
        : (i == 1
            ? c::SelfContactCandidateDisposition::
                ExcludedLocalIntersection
            : (i == 2
                ? c::SelfContactCandidateDisposition::
                    RepresentedByAcceptedVertexFace
                : c::SelfContactCandidateDisposition::
                    RepresentedByAcceptedEdgeEdge));
    values[i].accepted_event = i >= 2 ? 4+i : SIZE_MAX;
    values[i].source_order = i >= 2 ? 4+i : UINT64_MAX;
  }
  c::SelfContactCandidatePolicySummary whole;
  c::SelfContactCandidatePolicySummary chunks;
  sct::FoldPolicyOutcomes(values.data(), values.size(), &whole);
  sct::FoldPolicyOutcomes(values.data(), 1, &chunks);
  sct::FoldPolicyOutcomes(
      values.data() + 1, values.size() - 1, &chunks);
  EXPECT_EQ(whole.outcomes, chunks.outcomes);
  EXPECT_EQ(whole.certified_separated, 1u);
  EXPECT_EQ(whole.excluded_local_intersection, 1u);
  EXPECT_EQ(whole.represented_by_accepted_vf, 1u);
  EXPECT_EQ(whole.represented_by_accepted_ee, 1u);
  EXPECT_EQ(whole.digest, chunks.digest);
}

}  // namespace
