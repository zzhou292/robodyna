// SPDX-License-Identifier: MIT
#include "Oracle.h"

namespace represented_interval_test {
namespace {

std::array<ct::Vec3, 3> Translated(
    std::array<ct::Vec3, 3> points, ct::Vec3 displacement) {
  for (auto& point : points) {
    point.x += displacement.x;
    point.y += displacement.y;
    point.z += displacement.z;
  }
  return points;
}

ct::RepresentedTrianglePath WithKeys(
    ct::RepresentedTrianglePath path,
    std::array<std::uint64_t, 3> source_vertices) {
  for (unsigned vertex = 0; vertex < 3; ++vertex)
    path.vertices[vertex].key = Vertex(source_vertices[vertex]);
  for (unsigned edge = 0; edge < 3; ++edge)
    path.edge_keys[edge] = Edge(
        path.vertices[edge].key, path.vertices[(edge + 1) % 3].key);
  return path;
}

void CheckInvariantWitness(
    const ct::RepresentedTrianglePath& first,
    const ct::RepresentedTrianglePath& second,
    const ct::RepresentedIntervalResult& result, G shape) {
  ASSERT_EQ(result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.reason, R::None);
  EXPECT_TRUE(ct::HasExactCommonTranslationProof(result.geometry));
  EXPECT_EQ(ct::BaseIntersectionGeometry(result.geometry), shape);
  EXPECT_NE(result.feature.kind, K::None);
  EXPECT_EQ(result.accepted_event, SIZE_MAX);
  EXPECT_EQ(result.witness_time_numerator, 0u);
  EXPECT_EQ(result.witness_time_depth, 0u);
  EXPECT_EQ(result.work, 1u);
  for (std::uint64_t numerator = 0; numerator <= 8; ++numerator) {
    const auto oracle = ExactOracleAt(first, second, numerator, 8);
    ASSERT_TRUE(oracle.valid);
    EXPECT_TRUE(oracle.intersects);
    EXPECT_EQ(oracle.coplanar, shape == G::Coplanar);
  }
}

}  // namespace

TEST(RepresentedIntervalCrossing,
     TranslationProofPreservesPriorEnumOrdinalsAndWitnessShape) {
  static_assert(sizeof(G) == sizeof(std::uint8_t));
  static_assert(static_cast<unsigned>(G::None) == 0);
  static_assert(static_cast<unsigned>(G::Transverse) == 1);
  static_assert(static_cast<unsigned>(G::Coplanar) == 2);
  static_assert(static_cast<unsigned>(G::PersistentPhysicalContact) == 3);
  static_assert(static_cast<unsigned>(G::PersistentAcceptedLedgerCoverage) == 4);
  static_assert(static_cast<unsigned>(G::CertifiedLocalTopology) == 5);
  static_assert(static_cast<unsigned>(G::ExactCommonTranslationTransverse) == 6);
  static_assert(static_cast<unsigned>(G::ExactCommonTranslationCoplanar) == 7);
  for (const auto geometry : {G::None, G::Transverse, G::Coplanar,
                             G::PersistentPhysicalContact,
                             G::PersistentAcceptedLedgerCoverage,
                             G::CertifiedLocalTopology}) {
    EXPECT_FALSE(ct::HasExactCommonTranslationProof(geometry));
    EXPECT_EQ(ct::BaseIntersectionGeometry(geometry), geometry);
  }
  EXPECT_EQ(ct::BaseIntersectionGeometry(G::ExactCommonTranslationTransverse),
            G::Transverse);
  EXPECT_EQ(ct::BaseIntersectionGeometry(G::ExactCommonTranslationCoplanar),
            G::Coplanar);
}

TEST(RepresentedIntervalCrossing,
     ExactTranslationProofRetainsStaticAndMovingSharedTopologyWitnesses) {
  ct::RepresentedIntervalLimits limits;
  limits.max_work_per_pair = 1;
  limits.max_total_work = 1;
  auto owner = Owner(limits);
  const auto first_base = BaseTriangle();
  const std::array<ct::Vec3, 3> shared_edge{{
      {0, 0, 0}, {2, 0, 0}, {0, -2, 0}}};
  const std::array<ct::Vec3, 3> shared_vertex{{
      {0, 0, 0}, {-2, -1, 1}, {-2, -1, -1}}};
  for (const auto displacement : {ct::Vec3{0, 0, 0}, ct::Vec3{4, -3, 2}}) {
    const auto first = Path(
        10, first_base, Translated(first_base, displacement), 100);
    const auto edge = WithKeys(Path(
        20, shared_edge, Translated(shared_edge, displacement), 200),
        {100, 101, 202});
    const auto vertex = WithKeys(Path(
        30, shared_vertex, Translated(shared_vertex, displacement), 300),
        {100, 301, 302});
    ASSERT_NO_FATAL_FAILURE(CheckInvariantWitness(
        first, edge, One(owner, {first, edge}), G::Coplanar));
    ASSERT_NO_FATAL_FAILURE(CheckInvariantWitness(
        first, vertex, One(owner, {first, vertex}), G::Transverse));
  }
}

TEST(RepresentedIntervalCrossing,
     TranslationProofDoesNotConvertNonlocalOverlapIntoLocalAuthority) {
  ct::RepresentedIntervalLimits limits;
  limits.max_work_per_pair = 1;
  limits.max_total_work = 1;
  auto owner = Owner(limits);
  const ct::Vec3 displacement{4, -3, 2};
  const auto first_base = BaseTriangle();
  const auto first = Path(
      10, first_base, Translated(first_base, displacement), 100);
  const std::array<ct::Vec3, 3> contained{{
      {.25, .25, 0}, {.75, .25, 0}, {.25, .75, 0}}};
  const std::array<ct::Vec3, 3> transverse{{
      {.5, .25, -1}, {.5, 1.25, -1}, {.5, .75, 1}}};
  for (unsigned kind = 0; kind < 2; ++kind) {
    const auto& base = kind ? transverse : contained;
    const auto second = Path(20, base, Translated(base, displacement), 200);
    const auto result = One(owner, {first, second});
    ASSERT_NO_FATAL_FAILURE(CheckInvariantWitness(
        first, second, result, kind ? G::Transverse : G::Coplanar));
    EXPECT_NE(result.geometry, G::CertifiedLocalTopology);
    EXPECT_NE(result.classification, C::CertifiedExactExclusion);
  }
}

TEST(RepresentedIntervalCrossing,
     EqualRoundedDisplacementsCannotForgeExactTranslationProof) {
  auto owner = Owner();
  const auto first_base = BaseTriangle();
  const auto first = Path(
      10, first_base, Translated(first_base, {4, 0, 0}), 100);
  const double tiny = std::numeric_limits<double>::denorm_min();
  const std::array<ct::Vec3, 3> second_base{{
      {tiny, .25, 0}, {.75, .25, 0}, {.25, .75, 0}}};
  const auto second = Path(
      20, second_base, Translated(second_base, {4, 0, 0}), 200);
  ASSERT_GT(tiny, 0);
  ASSERT_EQ(second.vertices[0].endpoint[1].x, 4);
  // All rounded subtractions are four. The first second-path displacement
  // is the exact real 4-denorm_min, unlike the exact reference displacement.
  for (const auto* path : {&first, &second})
    for (const auto& vertex : path->vertices) {
      EXPECT_EQ(vertex.endpoint[1].x - vertex.endpoint[0].x, 4);
      EXPECT_EQ(vertex.endpoint[1].y - vertex.endpoint[0].y, 0);
      EXPECT_EQ(vertex.endpoint[1].z - vertex.endpoint[0].z, 0);
    }
  const auto result = One(owner, {first, second});
  EXPECT_EQ(result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.geometry, G::Coplanar);
  EXPECT_FALSE(ct::HasExactCommonTranslationProof(result.geometry));
  EXPECT_EQ(result.witness_time_numerator, 0u);
}

TEST(RepresentedIntervalCrossing,
     UnsupportedCurvedMotionCannotBorrowTranslationProofFromEndpoints) {
  auto owner = Owner();
  const auto base = BaseTriangle();
  const auto next = Translated(base, {4, -3, 2});
  const auto first = Path(10, base, next, 100);
  // RigidArc/Nonlinear can have nonzero interior curvature despite identical
  // endpoint displacements. This native linear module must reject their
  // declared path, rather than interpreting the endpoints as an affine proof.
  for (const auto motion : {ct::RepresentedMotion::RigidArc,
                            ct::RepresentedMotion::Nonlinear}) {
    const auto second = Path(20, base, next, 200, motion);
    const auto result = One(owner, {first, second});
    EXPECT_EQ(result.classification, C::Unresolved);
    EXPECT_EQ(result.reason, R::UnsupportedMotion);
    EXPECT_EQ(result.work, 0u);
    EXPECT_FALSE(ct::HasExactCommonTranslationProof(result.geometry));
  }
}

TEST(RepresentedIntervalCrossing,
     TranslationProofPreservesSeparatedAndDegenerateClassifications) {
  auto owner = Owner();
  const auto first = Static(10, BaseTriangle(), 100);
  const auto separated = One(owner, {first, Static(20, BaseTriangle(1), 200)});
  EXPECT_EQ(separated.classification, C::CertifiedSeparated);
  EXPECT_EQ(separated.geometry, G::None);
  EXPECT_FALSE(ct::HasExactCommonTranslationProof(separated.geometry));
  const std::array<ct::Vec3, 3> line{{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}}};
  const auto degenerate = One(owner, {first, Static(20, line, 200)});
  EXPECT_EQ(degenerate.classification, C::Unresolved);
  EXPECT_EQ(degenerate.reason, R::DegenerateGeometry);
  EXPECT_FALSE(ct::HasExactCommonTranslationProof(degenerate.geometry));
}

}  // namespace represented_interval_test
