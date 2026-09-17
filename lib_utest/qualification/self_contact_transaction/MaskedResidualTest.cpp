// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/self_contact_transaction/ResidualTasks.h"
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>

namespace {
namespace c = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;
using Status = sct::LinearResidualSeparationStatus;

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

sct::FacetQuadraticCoefficients ZeroCurvature() {
  sct::FacetQuadraticCoefficients value;
  value.complete = true;
  return value;
}

struct Roster {
  c::FixedTriangleFeatureTaskMask mask;
  std::array<c::FixedTriangleFeatureCandidate, 15> values{};
  std::size_t count = 0;
  c::FixedTriangleFeatureView view() const {
    return {values.data(), count, true};
  }
};

Roster Discover(const c::CurrentFixedTriangle& first,
                const c::CurrentFixedTriangle& second) {
  Roster roster;
  EXPECT_EQ(c::BuildFixedTriangleFeatureTaskMask(first, second, &roster.mask),
            c::FixedTriangleDiscoveryStatus::Ok);
  c::fixed_triangle_features::PairFeatureResult result;
  EXPECT_EQ(c::fixed_triangle_features::EvaluatePairFeaturesMaskedOnce(
                first, second, roster.mask, roster.values.data(),
                roster.values.size(), &result),
            c::FixedTriangleDiscoveryStatus::Ok);
  roster.count = result.feature_count;
  return roster;
}

struct SharedVertexPair {
  c::CurrentFixedTriangle first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  c::CurrentFixedTriangle second = Triangle(
      20, {1, 4, 5}, {{{0, 0, 0}, {-2, 0, 0}, {0, -2, 0}}});
};

struct ParallelPair {
  c::CurrentFixedTriangle first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  c::CurrentFixedTriangle second = Triangle(
      20, {4, 5, 6}, {{{0, 0, 1}, {2, 0, 1}, {0, 2, 1}}});
};

sct::LinearResidualSeparationResult StaticCertificate(
    const c::CurrentFixedTriangle& first,
    const c::CurrentFixedTriangle& second, const Roster& roster,
    double half = .125) {
  return sct::CertifyQuadraticUnmaskedSeparation(
      first, first, ZeroCurvature(), half,
      second, second, ZeroCurvature(), half, 1,
      roster.view(), roster.mask);
}

void ExpectSame(const sct::LinearResidualSeparationResult& actual,
                const sct::LinearResidualSeparationResult& expected) {
  EXPECT_EQ(actual.status, expected.status);
  EXPECT_EQ(actual.reference_translation.x, expected.reference_translation.x);
  EXPECT_EQ(actual.reference_translation.y, expected.reference_translation.y);
  EXPECT_EQ(actual.reference_translation.z, expected.reference_translation.z);
  EXPECT_EQ(actual.first_residual_upper_m, expected.first_residual_upper_m);
  EXPECT_EQ(actual.second_residual_upper_m, expected.second_residual_upper_m);
  EXPECT_EQ(actual.prepared_distance_lower_m, expected.prepared_distance_lower_m);
  EXPECT_EQ(actual.strict_gap_lower_m, expected.strict_gap_lower_m);
  EXPECT_EQ(actual.exact_common_translation, expected.exact_common_translation);
}

TEST(SelfContactMaskedResidual, CompleteNativeTasksPermitLocalSharedVertexGap) {
  const SharedVertexPair pair;
  auto roster = Discover(pair.first, pair.second);
  ASSERT_NE(roster.mask.local_tasks, 0u);
  ASSERT_GT(roster.count, 1u);
  ASSERT_LT(roster.count, 15u);
  const auto result = StaticCertificate(pair.first, pair.second, roster);
  ASSERT_EQ(result.status, Status::CertifiedSeparated);
  EXPECT_GT(result.strict_gap_lower_m, 0);
  std::reverse(roster.values.begin(), roster.values.begin() + roster.count);
  ExpectSame(StaticCertificate(pair.first, pair.second, roster), result);
  ExpectSame(StaticCertificate(pair.second, pair.first, roster), result);
}

TEST(SelfContactMaskedResidual, EndpointIdentityAndExactLocalMaskAuthenticate) {
  const SharedVertexPair pair;
  const auto roster = Discover(pair.first, pair.second);
  auto modified = roster;
  modified.mask.local_tasks ^= 1;
  EXPECT_EQ(StaticCertificate(pair.first, pair.second, modified).status,
            Status::InvalidInput);
  modified = roster;
  modified.mask.local_tasks |= 0x8000;
  EXPECT_EQ(StaticCertificate(pair.first, pair.second, modified).status,
            Status::InvalidInput);
  auto changed = Triangle(
      20, {1, 9, 5}, {{{0, 0, 0}, {-2, 0, 0}, {0, -2, 0}}});
  EXPECT_EQ(sct::CertifyQuadraticUnmaskedSeparation(
      pair.first, pair.first, ZeroCurvature(), .125,
      changed, pair.second, ZeroCurvature(), .125, 1,
      roster.view(), roster.mask).status, Status::InvalidInput);
  changed = pair.second;
  changed.edge_keys[0].parent_boundary = false;
  changed.edge_keys[0].parent_eid = changed.key.parent_eid;
  EXPECT_EQ(sct::CertifyQuadraticUnmaskedSeparation(
      pair.first, pair.first, ZeroCurvature(), .125,
      pair.second, changed, ZeroCurvature(), .125, 1,
      roster.view(), roster.mask).status, Status::InvalidInput);
}

TEST(SelfContactMaskedResidual, MissingDuplicateForeignAndMaskedTasksReject) {
  const SharedVertexPair pair;
  const auto roster = Discover(pair.first, pair.second);
  ASSERT_GT(roster.count, 1u);
  auto modified = roster;
  --modified.count;
  EXPECT_EQ(StaticCertificate(pair.first, pair.second, modified).status,
            Status::IncompleteFeatureRoster);
  modified = roster;
  modified.values[1] = modified.values[0];
  EXPECT_EQ(StaticCertificate(pair.first, pair.second, modified).status,
            Status::IncompleteFeatureRoster);
  modified = roster;
  ++modified.values[0].triangles[1].parent_eid;
  EXPECT_EQ(StaticCertificate(pair.first, pair.second, modified).status,
            Status::IncompleteFeatureRoster);
  modified = roster;
  modified.values[0].local_features[0] = 99;
  modified.values[0].local_features[1] = 99;
  EXPECT_EQ(StaticCertificate(pair.first, pair.second, modified).status,
            Status::IncompleteFeatureRoster);
  // Native discovery suppresses exact local incidence. Deliberately inject
  // the omitted shared-vertex VF task with its real immutable source keys:
  // this malformed roster must reject before inspecting the zero distance.
  ASSERT_NE(roster.mask.local_tasks & c::FixedTriangleFeatureTaskBit(
      c::FixedTriangleVertexFaceTaskSlot(0, 0)), 0u);
  c::FixedTriangleFeatureCandidate local;
  local.key.vertex_face.vertex = pair.first.vertex_keys[0];
  local.key.vertex_face.target.SetVertex(pair.second.vertex_keys[0]);
  local.triangles[0] = pair.first.key;
  local.triangles[1] = pair.second.key;
  local.local_features[0] = 0;
  local.local_features[1] = 3;
  local.points[0] = pair.first.vertices[0];
  local.points[1] = pair.second.vertices[0];
  modified = roster;
  modified.values[0] = local;
  EXPECT_EQ(StaticCertificate(pair.first, pair.second, modified).status,
            Status::IncompleteFeatureRoster);
}

TEST(SelfContactMaskedResidual, NoMaskedTasksPreserveLegacyArithmeticExactly) {
  const ParallelPair pair;
  const auto roster = Discover(pair.first, pair.second);
  ASSERT_EQ(roster.mask.local_tasks, 0u);
  ASSERT_EQ(roster.count, 15u);
  const auto result = StaticCertificate(pair.first, pair.second, roster);
  ASSERT_EQ(result.status, Status::CertifiedSeparated);
  const auto legacy = sct::CertifyQuadraticResidualSeparation(
      pair.first, pair.first, ZeroCurvature(), .125,
      pair.second, pair.second, ZeroCurvature(), .125, 1,
      roster.view(), {nullptr, 0, true});
  ExpectSame(result, legacy);
  EXPECT_EQ(StaticCertificate(pair.first, pair.second, roster, .5).status,
            Status::PotentialContact);
  auto uncertain = roster;
  uncertain.values[0].representation_error_m = .75;
  EXPECT_EQ(StaticCertificate(pair.first, pair.second, uncertain).status,
            Status::PotentialContact);
}

TEST(SelfContactMaskedResidual, RelativeEndpointMotionAndCurvatureConsumeGap) {
  const ParallelPair pair;
  const auto roster = Discover(pair.first, pair.second);
  auto crossing_base = pair.second;
  for (auto& vertex : crossing_base.vertices) vertex.z = -1;
  EXPECT_EQ(sct::CertifyQuadraticUnmaskedSeparation(
      pair.first, pair.first, ZeroCurvature(), .125,
      crossing_base, pair.second, ZeroCurvature(), .125, 1,
      roster.view(), roster.mask).status, Status::PotentialContact);
  auto curved = ZeroCurvature();
  for (auto& vertex : curved.q) vertex[2] = {16, 16};
  EXPECT_EQ(sct::CertifyQuadraticUnmaskedSeparation(
      pair.first, pair.first, ZeroCurvature(), .125,
      pair.second, pair.second, curved, .125, 1,
      roster.view(), roster.mask).status, Status::PotentialContact);
  for (auto& vertex : curved.q) vertex[2] = {.0625, .0625};
  EXPECT_EQ(sct::CertifyQuadraticUnmaskedSeparation(
      pair.first, pair.first, ZeroCurvature(), .125,
      pair.second, pair.second, curved, .125, 1,
      roster.view(), roster.mask).status, Status::CertifiedSeparated);
}

TEST(SelfContactMaskedResidual, SmallAngleRigidCoefficientCannotHideMidpathCrossing) {
  constexpr double epsilon = 1.0 / 4096;
  constexpr double half = 1.0 / 32768;
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {4, 0, 0}, {0, 4, 0}}});
  const auto second = Triangle(
      20, {1, 4, 5}, {{{0, 0, 0}, {1, 1, epsilon}, {2, 1, epsilon}}});
  const auto roster = Discover(first, second);
  ASSERT_EQ(StaticCertificate(first, second, roster, half).status,
            Status::CertifiedSeparated);
  auto curved = ZeroCurvature();
  // ContinuousLocalTest independently derives this exact coefficient from
  // the real rigid recurrence at omega=1/16, below its angular step screen.
  curved.q[1][2] = {1.0 / 256, 1.0 / 256};
  ASSERT_GT(curved.q[1][2].lower / 8, epsilon - 2 * half);
  EXPECT_EQ(sct::CertifyQuadraticUnmaskedSeparation(
      first, first, ZeroCurvature(), half,
      second, second, curved, half, 1,
      roster.view(), roster.mask).status, Status::PotentialContact);
}

TEST(SelfContactMaskedResidual, InvalidCurvatureTimeAndGeometryFailClosed) {
  const ParallelPair pair;
  const auto roster = Discover(pair.first, pair.second);
  const auto certify = [&](const sct::FacetQuadraticCoefficients& q, double dt) {
    return sct::CertifyQuadraticUnmaskedSeparation(
        pair.first, pair.first, ZeroCurvature(), .125,
        pair.second, pair.second, q, .125, dt,
        roster.view(), roster.mask).status;
  };
  auto q = ZeroCurvature();
  EXPECT_EQ(certify(q, 0), Status::InvalidInput);
  EXPECT_EQ(certify(q, -1), Status::InvalidInput);
  EXPECT_EQ(certify(q, std::numeric_limits<double>::infinity()),
            Status::InvalidInput);
  EXPECT_EQ(certify(q, std::numeric_limits<double>::denorm_min()),
            Status::CertifiedSeparated);
  q.q[1][2] = {1, 1};
  EXPECT_EQ(certify(q, std::numeric_limits<double>::denorm_min()),
            Status::CertifiedSeparated);
  EXPECT_EQ(certify(q, std::numeric_limits<double>::max()),
            Status::InvalidInput);
  q.complete = false;
  EXPECT_EQ(certify(q, 1), Status::InvalidInput);
  q = ZeroCurvature();
  q.q[1][2] = {1, -1};
  EXPECT_EQ(certify(q, 1), Status::InvalidInput);
  q.q[1][2] = {0, std::numeric_limits<double>::infinity()};
  EXPECT_EQ(certify(q, 1), Status::InvalidInput);
  q.q[1][2] = {std::numeric_limits<double>::quiet_NaN(), 0};
  EXPECT_EQ(certify(q, 1), Status::InvalidInput);
  auto degenerate = pair.second;
  degenerate.vertices[1] = degenerate.vertices[0];
  EXPECT_EQ(StaticCertificate(pair.first, degenerate, roster).status,
            Status::InvalidInput);
  auto invalid = roster;
  invalid.values[0].points[0].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(StaticCertificate(pair.first, pair.second, invalid).status,
            Status::InvalidInput);
}

TEST(SelfContactMaskedResidual, EmptyUnmaskedRosterCannotCertifyTopology) {
  const SharedVertexPair pair;
  auto same_support = pair.first;
  same_support.key = pair.second.key;
  const auto roster = Discover(pair.first, same_support);
  ASSERT_EQ(roster.mask.local_tasks, c::FixedTriangleFeatureTaskBits);
  ASSERT_EQ(roster.count, 0u);
  EXPECT_EQ(StaticCertificate(pair.first, same_support, roster).status,
            Status::IncompleteFeatureRoster);
}
}  // namespace
