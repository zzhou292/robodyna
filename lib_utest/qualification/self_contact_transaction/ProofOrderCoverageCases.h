// SPDX-License-Identifier: MIT
#pragma once

TEST(SelfContactProofOrderCoverage, AcceptedOwnerReportsAndFailureBudgetsRemainIdentical) {
  const auto first = Triangle(10, {1, 2, 3}, {{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}});
  const auto second = Triangle(20, {1, 4, 5}, {{{0, 0, 0}, {0, -1, 1}, {0, -1, -1}}});
  const auto geometry = DiscoverPreparedPair(first, second);
  const auto owner = AcceptedCertificate(FirstEdgeEdge(geometry));
  const sct::AcceptedEventCertificate duplicate[]{owner, owner};
  for (unsigned count : {0u, 1u, 2u})
    for (std::size_t work : {0u, 1u, 255u})
      for (unsigned depth : {0u, 8u, 53u}) {
        const auto compared = sct::CompareSharedVertexCoverageOrders(
            first, first, Quadratic(0), .75,
            second, second, Quadratic(0), .75, 1,
            duplicate, count, work, depth);
        sct::test::ExpectResult(compared.cone_first.report, compared.polynomial_first.report);
        const auto actual = sct::CertifyQuadraticFacetCoverage(
            first, first, Quadratic(0), .75,
            second, second, Quadratic(0), .75, 1,
            duplicate, count, work, depth);
        sct::test::ExpectResult(actual, compared.cone_first.report);
      }
}

TEST(SelfContactProofOrderCoverage, RealNonlocalGeometryCannotGainAcceptedCoverage) {
  const auto first = Triangle(10, {1, 2, 3}, {{{0, 0, 0}, {4, 0, 0}, {0, 4, 0}}});
  const auto second = Triangle(20, {1, 4, 5}, {{{0, 0, 0}, {1, 1, 1}, {1, 2, -1}}});
  const auto geometry = DiscoverPreparedPair(first, second);
  ASSERT_EQ(geometry.intersection_count, 1u);
  ASSERT_TRUE(c::RequiresIntersectionAdmission(geometry.intersections[0]));
  const auto owner = AcceptedCertificate(FirstEdgeEdge(geometry));
  const auto compared = sct::CompareSharedVertexCoverageOrders(
      first, first, Quadratic(0), .75,
      second, second, Quadratic(0), .75, 1,
      &owner, 1, 255, 8);
  sct::test::ExpectResult(compared.cone_first.report, compared.polynomial_first.report);
  EXPECT_NE(compared.cone_first.report.status,
            sct::NonlinearSeparationStatus::CertifiedAcceptedCoverage);
}
