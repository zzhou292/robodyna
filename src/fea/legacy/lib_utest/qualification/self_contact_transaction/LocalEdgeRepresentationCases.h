// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Exact prepared coordinates captured from the unconstrained original
// adjacent T3/Q4 CUDA coupon in continuous-local-transaction-tests-2.log.
// Preserve its current fail-closed representation limit independently of the
// source-authenticated CUDA rejection/rollback/retry regression.
TEST(SelfContactLocalRepresentation,
     RealPreparedEdgeInteriorWithRoundedUnitComplementFailsClosed) {
  const c::Vec3 node10{
      0x1.11ea8f9c5a9ddp-75, 0x1.1e9973f2d1014p-75, 0x1.b639dc4204a53p-77};
  const c::Vec3 node11{
      0x1.47ae147ae147bp-5, -0x1.5030928fcede8p-84, -0x1.caea008d1ef56p-77};
  const c::Vec3 node12{0x1.47ae147ae147bp-5, 0x1.47ae147ae147bp-6, 0};
  const c::Vec3 node14{0x1.999999999999ap-5, 0x1.47ae147ae147bp-7, 0};
  const auto first = Triangle(102, {11, 14, 12}, {{node11, node14, node12}});
  const auto second = Triangle(103, {10, 11, 12}, {{node10, node11, node12}});
  const c::Vec3 canonical_t3[]{node11, node12, node14};
  c::fixed_triangle_features::exact::ClosestTriangleStratum stratum;
  ASSERT_TRUE(c::fixed_triangle_features::exact::ClosestStratum(
      node10, canonical_t3, &stratum));
  ASSERT_EQ(stratum.kind, c::fixed_triangle_features::exact::ClosestStratumKind::Edge);
  ASSERT_EQ(stratum.local, 0u);
  c::SegmentGeometry edge;
  edge.vertices[0] = node11; edge.vertices[1] = node12;
  edge.vertex_ids[0] = 11; edge.vertex_ids[1] = 12;
  c::SegmentPointGeometry closest;
  ASSERT_EQ(c::ClosestPointOnSegment(node10, edge, &closest), c::Status::kOk);
  ASSERT_GT(closest.parameter, 0);
  ASSERT_LT(closest.parameter, 1);
  EXPECT_EQ(1 - closest.parameter, 1);
  // Rounding the complementary weight to one cannot silently relabel this
  // exact edge stratum as a vertex or authorize the candidate interval.
  c::FixedTriangleFeatureTaskMask mask;
  ASSERT_EQ(c::BuildFixedTriangleFeatureTaskMask(first, second, &mask),
            c::FixedTriangleDiscoveryStatus::Ok);
  c::FixedTriangleFeatureCandidate features[15];
  c::fixed_triangle_features::PairFeatureResult result;
  EXPECT_EQ(c::fixed_triangle_features::EvaluatePairFeaturesMaskedOnce(
      first, second, mask, features, 15, &result),
      c::FixedTriangleDiscoveryStatus::NonFiniteResult);
  EXPECT_EQ(result.input_task, 1u);
  EXPECT_EQ(result.arithmetic_reason,
            c::FixedTriangleArithmeticReason::EdgeInteriorRepresentation);
}
