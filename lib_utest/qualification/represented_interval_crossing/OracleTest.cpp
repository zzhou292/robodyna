// SPDX-License-Identifier: MIT
#include "Oracle.h"

namespace represented_interval_test {
namespace {

void WitnessAgrees(const ct::RepresentedTrianglePath& a,
                   const ct::RepresentedTrianglePath& b,
                   const ct::RepresentedIntervalResult& result) {
  ASSERT_EQ(result.classification, C::CertifiedCrossingContact);
  const std::uint64_t denominator =
      std::uint64_t{1} << result.witness_time_depth;
  const auto oracle = ExactOracleAt(
      a, b, result.witness_time_numerator, denominator);
  ASSERT_TRUE(oracle.valid);
  EXPECT_TRUE(oracle.intersects);
  EXPECT_EQ(oracle.coplanar, result.geometry == G::Coplanar);
  if (result.feature.kind == K::VertexFace)
    EXPECT_TRUE(oracle.vertex_face);
  if (result.feature.kind == K::EdgeEdge)
    EXPECT_TRUE(oracle.edge_edge);
}

}  // namespace

TEST(RepresentedIntervalCrossing,
     IndependentExactRationalOracleChecksStaticSatAndFeatures) {
  const auto a = Static(10, BaseTriangle(), 100);
  const std::array<ct::Vec3, 3> contained{
      ct::Vec3{.25, .25, 0}, {.75, .25, 0}, {.25, .75, 0}};
  auto oracle = ExactOracleAt(a, Static(20, contained, 200), 0, 1);
  ASSERT_TRUE(oracle.valid);
  EXPECT_TRUE(oracle.intersects);
  EXPECT_TRUE(oracle.coplanar);
  EXPECT_TRUE(oracle.vertex_face);

  const std::array<ct::Vec3, 3> boundary{
      ct::Vec3{2, 0, 0}, {3, 0, 0}, {2, -1, 0}};
  oracle = ExactOracleAt(a, Static(20, boundary, 200), 0, 1);
  ASSERT_TRUE(oracle.valid);
  EXPECT_TRUE(oracle.intersects);
  EXPECT_TRUE(oracle.coplanar);
  EXPECT_TRUE(oracle.vertex_face);
  EXPECT_TRUE(oracle.edge_edge);

  oracle = ExactOracleAt(a, Static(20, BaseTriangle(1), 200), 0, 1);
  ASSERT_TRUE(oracle.valid);
  EXPECT_FALSE(oracle.intersects);
}

TEST(RepresentedIntervalCrossing,
     ProductionCrossingWitnessesPassIndependentExactOracle) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle(), 100);
  const auto pass =
      Path(20, BaseTriangle(1), BaseTriangle(-3), 200);
  WitnessAgrees(a, pass, One(owner, {a, pass}));

  const std::array<ct::Vec3, 3> edge_first{
      ct::Vec3{.5, -.5, 1}, {.5, 2, 1}, {.5, 2.5, 2}};
  const std::array<ct::Vec3, 3> edge_second{
      ct::Vec3{.5, -.5, -1}, {.5, 2, -1}, {.5, 2.5, 2}};
  const auto edge = Path(30, edge_first, edge_second, 300);
  WitnessAgrees(a, edge, One(owner, {a, edge}));
}

TEST(RepresentedIntervalCrossing,
     CoplanarContainmentAndBoundaryAreClosedContactNotSeparation) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle(), 100);
  const std::array<ct::Vec3, 3> contained{
      ct::Vec3{.25, .25, 0}, {.75, .25, 0}, {.25, .75, 0}};
  const auto inside = Static(20, contained, 200);
  const auto inside_result = One(owner, {a, inside});
  EXPECT_EQ(inside_result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(inside_result.geometry, G::Coplanar);
  WitnessAgrees(a, inside, inside_result);

  const std::array<ct::Vec3, 3> boundary{
      ct::Vec3{2, 0, 0}, {3, 0, 0}, {2, -1, 0}};
  const auto touch = Static(30, boundary, 300);
  const auto touch_result = One(owner, {a, touch});
  EXPECT_EQ(touch_result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(touch_result.geometry, G::Coplanar);
  WitnessAgrees(a, touch, touch_result);
}

}  // namespace represented_interval_test
