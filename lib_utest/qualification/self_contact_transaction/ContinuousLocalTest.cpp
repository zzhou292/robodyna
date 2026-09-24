// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/self_contact_transaction/Storage.h"
#include "lib_src/collision/self_contact_transaction/SharedVertexProofQualification.h"
#include "NonlinearResultAssertions.h"
#include "lib_src/collision/self_contact_transaction/ConeDirections.h"
#include "ConeDirectionOracle.h"
#include "lib_src/collision/self_contact_transaction/LocalContact.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace {
namespace c = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;
namespace fe = tl::fea;

constexpr double Epsilon = 1.0 / 4096;
constexpr double HalfThickness = 1.0 / 32768;
constexpr double Duration = 1;

c::CurrentFixedTriangle Triangle(
    std::uint64_t eid, std::array<std::uint64_t, 3> ids,
    std::array<c::Vec3, 3> positions) {
  c::CurrentFixedTriangle value;
  value.key = {17, eid, 0, 0};
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    value.vertices[vertex] = positions[vertex];
    value.vertex_keys[vertex].source_instance_id = 17;
    value.vertex_keys[vertex].first = ids[vertex];
    value.vertex_keys[vertex].denominator = 1;
  }
  for (unsigned edge = 0; edge < 3; ++edge) {
    auto first = value.vertex_keys[edge];
    auto second = value.vertex_keys[(edge + 1) % 3];
    if (c::fixed_triangle_features::Compare(second, first) < 0)
      std::swap(first, second);
    value.edge_keys[edge].endpoints[0] = first;
    value.edge_keys[edge].endpoints[1] = second;
    value.edge_keys[edge].parent_boundary = true;
  }
  return value;
}

struct CurvedSharedVertex {
  c::CurrentFixedTriangle first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {4, 0, 0}, {0, 4, 0}}});
  c::CurrentFixedTriangle second = Triangle(
      20, {1, 4, 5}, {{{0, 0, 0}, {1, 1, Epsilon}, {2, 1, Epsilon}}});
  sct::FacetQuadraticCoefficients stationary;
  sct::FacetQuadraticCoefficients curved;

  void DeriveActualRigidCoefficient() {
    // A mixed facet: the common vertex and last vertex are ordinary nodes;
    // its middle vertex belongs to one rotating/translating rigid body.
    // The owner recurrence x1=c1+r+h(w x r)+h^2(w x (w x r))/2
    // gives x1=x0 exactly for r=(0,0,-1), w=(1/16,0,0), h=1.
    // Rotation is 1/16 rad, below the normal 0.1-rad owner screen.
    const double positions[]{
        0, 0, 0, 4, 0, 0, 0, 4, 0,
        1, 1, Epsilon, 2, 1, Epsilon};
    const std::uint32_t groups[]{
        UINT32_MAX, UINT32_MAX, UINT32_MAX, 0, UINT32_MAX};
    fe::NodalRigidGroupSnapshot accepted;
    accepted.source_group_id = 41;
    accepted.source_node_set_id = 43;
    accepted.source_kind = fe::RigidBindingSourceKind::Part;
    accepted.state.center = {1, 1, 1 + Epsilon};
    auto prepared = accepted;
    prepared.state.center = {1, 15.0 / 16, 1 + Epsilon - 1.0 / 512};
    prepared.state.omega = {1.0 / 16, 0, 0};
    const auto& center = prepared.state.center;
    EXPECT_EQ(center.x, second.vertices[1].x);
    EXPECT_EQ(center.y + 1.0 / 16, second.vertices[1].y);
    EXPECT_EQ(center.z - 1 + 1.0 / 512, second.vertices[1].z);

    c::FixedContactFacet facet;
    const std::uint32_t nodes[]{0, 3, 4};
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      facet.vertices[vertex].count = 3;
      for (unsigned slot = 0; slot < 3; ++slot)
        facet.vertices[vertex].nodes[slot] = nodes[slot];
      facet.vertices[vertex].weights[vertex] = 1;
    }
    bool affine = true;
    ASSERT_EQ(sct::BuildRigidFacetQuadraticCoefficients(
        facet, {positions, 5, 3, 1}, {positions, 5, 3, 1},
        groups, &accepted, &prepared, 1,
        fe::NodalRigidMemberTrajectory::EndpointCorrectedSecondOrderDriftV1,
        Duration, &curved, &affine), sct::RigidMemberSweepStatus::Ok);
    ASSERT_TRUE(curved.complete);
    EXPECT_FALSE(affine);
    EXPECT_EQ(curved.q[1][2].lower, 1.0 / 256);
    EXPECT_EQ(curved.q[1][2].upper, 1.0 / 256);
    stationary.complete = true;
  }
};

TEST(SelfContactContinuousLocal,
     ActualSmallAngleMixedRigidPathCrossesOnlyBetweenLocalEndpoints) {
  CurvedSharedVertex geometry;
  ASSERT_NO_FATAL_FAILURE(geometry.DeriveActualRigidCoefficient());
  c::FixedTriangleFeatureTaskMask mask;
  ASSERT_EQ(c::BuildFixedTriangleFeatureTaskMask(
      geometry.first, geometry.second, &mask), c::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_NE(mask.local_tasks, 0u);
  std::array<c::FixedTriangleFeatureCandidate, 15> features;
  c::fixed_triangle_features::PairFeatureResult feature_result;
  ASSERT_EQ(c::fixed_triangle_features::EvaluatePairFeaturesMaskedOnce(
      geometry.first, geometry.second, mask, features.data(), features.size(),
      &feature_result), c::FixedTriangleDiscoveryStatus::Ok);
  std::size_t expected = 15;
  for (unsigned task = 0; task < 15; ++task)
    expected -= bool(mask.local_tasks & c::FixedTriangleFeatureTaskBit(task));
  ASSERT_EQ(feature_result.feature_count, expected);
  for (std::size_t feature = 0; feature < feature_result.feature_count; ++feature)
    ASSERT_GT((features[feature].distance_m - HalfThickness) - HalfThickness,
              features[feature].representation_error_m);

  c::FixedTriangleIntersection endpoint;
  bool intersects = false;
  ASSERT_EQ(c::fixed_triangle_features::ClassifyPairIntersection(
      geometry.first, geometry.second, &endpoint, &intersects),
      c::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_TRUE(intersects);
  ASSERT_EQ(endpoint.local_exclusion, c::FixedTriangleLocalExclusion::SharedVertexOnly);
  ASSERT_FALSE(c::RequiresIntersectionAdmission(endpoint));

  // x(u)=x0+(x1-x0)u+h^2*q*u*(u-1)/2. Every value here is dyadic:
  // at u=1/2 the moving arm has z=-epsilon while the other has +epsilon.
  // Their edge crosses the interior of the first triangle at (1.5,1,0).
  auto middle = geometry.second;
  middle.vertices[1].z -= geometry.curved.q[1][2].lower / 8;
  ASSERT_EQ(middle.vertices[1].z, -Epsilon);
  c::FixedTriangleIntersection interior;
  ASSERT_EQ(c::fixed_triangle_features::ClassifyPairIntersection(
      geometry.first, middle, &interior, &intersects),
      c::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_TRUE(intersects);
  ASSERT_TRUE(c::RequiresIntersectionAdmission(interior));
  ASSERT_EQ(interior.local_exclusion, c::FixedTriangleLocalExclusion::None);
  const auto continuous = sct::CertifyQuadraticLocalContact(
      geometry.first, geometry.first, geometry.stationary, HalfThickness,
      geometry.second, geometry.second, geometry.curved, HalfThickness,
      Duration, 255, 12);
  EXPECT_NE(continuous.status,
            sct::NonlinearSeparationStatus::CertifiedLocalIntersection);

  // These are precisely the endpoint facts used by the old unsupported-motion
  // shortcut. Exercise the real final publication validator, not a model of
  // its decision: unknown continuous geometry must never become local policy.
  const c::RepresentedIntervalPairKey pair{{{17, 10, 0, 0}, {17, 20, 0, 0}}};
  c::RepresentedIntervalResult crossing;
  crossing.key = pair;
  crossing.classification = c::RepresentedIntervalClassification::Unresolved;
  crossing.reason = c::RepresentedIntervalReason::UnsupportedMotion;
  c::SelfContactCandidatePolicyOutcome outcome;
  std::size_t count = 0;
  sct::CandidateValidationInput input;
  input.canonical_pairs = &pair;
  input.pair_count = 1;
  input.features = {features.data(), feature_result.feature_count, true};
  input.intersections = {&endpoint, 1, true};
  input.crossings = {&crossing, 1, true};
  input.outcomes = &outcome;
  input.outcome_capacity = 1;
  input.outcome_count = &count;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
            c::SelfContactTransactionStatus::UnresolvedCandidate);
  EXPECT_EQ(count, 0u);
}

sct::FacetQuadraticCoefficients ZeroQuadratic() {
  sct::FacetQuadraticCoefficients result;
  result.complete = true;
  return result;
}

sct::NonlinearSeparationResult LocalPolicy(
    const c::CurrentFixedTriangle& first,
    const c::CurrentFixedTriangle& second,
    const c::CurrentFixedTriangle& prepared,
    const sct::FacetQuadraticCoefficients& curved,
    std::size_t work = 255, unsigned depth = 12,
    double thickness = HalfThickness) {
  return sct::CertifyQuadraticFacetPolicyCoverage(
      first, first, ZeroQuadratic(), thickness,
      second, prepared, curved, thickness, Duration,
      nullptr, 0, nullptr, 0, work, depth);
}

#include "ConeDiagonalCases.h"
#include "ProofOrderCases.h"
#include "AffineConeCases.h"

TEST(SelfContactContinuousLocal, CurvedSharedVertexNeedsNoInventedOwner) {
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}});
  const auto second = Triangle(
      20, {1, 4, 5}, {{{0, 0, 0}, {0, -1, 1}, {0, -1, -1}}});
  auto curved = ZeroQuadratic();
  curved.q[1][2] = {1.0 / 16, 1.0 / 16};
  const auto result = LocalPolicy(first, second, second, curved);
  EXPECT_EQ(result.status,
            sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_GT(result.work, 0u);
  EXPECT_EQ(result.accepted_certificate, SIZE_MAX);
  EXPECT_EQ(result.covered_cells, 0u);
  EXPECT_TRUE(result.has_intersection);
  EXPECT_EQ(result.intersection_feature.kind,
            c::RepresentedFeatureKind::TriangleIntersection);
  EXPECT_FALSE(result.work_exhausted);
  EXPECT_FALSE(result.depth_exhausted);
}

TEST(SelfContactContinuousLocal, AffineDifferentialMotionKeepsSharedVertexLocal) {
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}});
  const auto second = Triangle(
      20, {1, 4, 5}, {{{0, 0, 0}, {0, -1, 1}, {0, -1, -1}}});
  auto prepared = second;
  prepared.vertices[1].z += 1.0 / 64;
  EXPECT_EQ(LocalPolicy(first, second, prepared, ZeroQuadratic()).status,
            sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
}

TEST(SelfContactContinuousLocal, CurvedSharedEdgeRetainsLocalDisposition) {
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto second = Triangle(
      20, {2, 1, 4}, {{{2, 0, 0}, {0, 0, 0}, {0, -2, 0}}});
  auto curved = ZeroQuadratic();
  curved.q[2][2] = {1.0 / 16, 1.0 / 16};
  const auto result = LocalPolicy(first, second, second, curved);
  ASSERT_EQ(result.status,
            sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  c::FixedTriangleIntersection endpoint;
  bool intersects = false;
  ASSERT_EQ(c::fixed_triangle_features::ClassifyPairIntersection(
      first, second, &endpoint, &intersects), c::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_TRUE(intersects);
  const c::RepresentedIntervalPairKey pair{{{17, 10, 0, 0}, {17, 20, 0, 0}}};
  c::RepresentedIntervalResult crossing;
  crossing.key = pair;
  crossing.classification = c::RepresentedIntervalClassification::CertifiedCrossingContact;
  crossing.reason = c::RepresentedIntervalReason::None;
  crossing.geometry = c::RepresentedIntersectionGeometry::CertifiedLocalTopology;
  crossing.feature.kind = c::RepresentedFeatureKind::TriangleIntersection;
  c::SelfContactCandidatePolicyOutcome outcome;
  std::size_t count = 0;
  sct::CandidateValidationInput input;
  input.canonical_pairs = &pair;
  input.pair_count = 1;
  input.features.complete = true;
  input.intersections = {&endpoint, 1, true};
  input.crossings = {&crossing, 1, true};
  input.outcomes = &outcome;
  input.outcome_capacity = 1;
  input.outcome_count = &count;
  ASSERT_EQ(sct::ValidateCandidatePublications(input).status,
            c::SelfContactTransactionStatus::Ok);
  EXPECT_EQ(outcome.disposition,
            c::SelfContactCandidateDisposition::ExcludedLocalIntersection);
  EXPECT_EQ(count, 1u);
  crossing.accepted_event = 0;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
            c::SelfContactTransactionStatus::CandidateRejected);
}

TEST(SelfContactContinuousLocal, RejectsPreexistingNonlocalIntersection) {
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {2, 1, 0}, {2, -1, 0}}});
  const auto second = Triangle(
      20, {1, 4, 5}, {{{0, 0, 0}, {3, 0, 1}, {3, 0, -1}}});
  const auto result = sct::CertifyQuadraticLocalTopology(
      first, first, ZeroQuadratic(), second, second, ZeroQuadratic(),
      Duration, 255, 12);
  EXPECT_EQ(result.status, sct::NonlinearSeparationStatus::PotentialContact);
  EXPECT_EQ(result.work, 0u);
  EXPECT_NE(LocalPolicy(first, second, second, ZeroQuadratic()).status,
            sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
}

TEST(SelfContactContinuousLocal, RejectsUnownedThicknessAndChangedSharedPath) {
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}});
  const auto second = Triangle(
      20, {1, 4, 5}, {{{0, 0, 0}, {0, -1, 1}, {0, -1, -1}}});
  EXPECT_NE(LocalPolicy(first, second, second, ZeroQuadratic(), 255, 12, 2).status,
            sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  auto changed = ZeroQuadratic();
  changed.q[0][0] = {1.0 / 32, 1.0 / 32};
  EXPECT_NE(LocalPolicy(first, second, second, changed).status,
            sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_EQ(LocalPolicy(first, second, second, ZeroQuadratic(), 0).status,
            sct::NonlinearSeparationStatus::InvalidInput);
  EXPECT_EQ(LocalPolicy(first, second, second, ZeroQuadratic(), 1, 53).status,
            sct::NonlinearSeparationStatus::InvalidInput);
  const auto capped = sct::CertifyQuadraticLocalTopology(
      first, first, ZeroQuadratic(), second, second, changed, Duration, 1, 12);
  EXPECT_EQ(capped.status, sct::NonlinearSeparationStatus::WorkExhausted);
  EXPECT_EQ(capped.work, 1u);
  EXPECT_TRUE(capped.work_exhausted);
}

}  // namespace
