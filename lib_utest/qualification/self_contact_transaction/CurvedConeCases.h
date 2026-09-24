// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Included after the existing affine/source-path fixtures and result oracle.
namespace curved_cone_test {
using ExactControls = std::array<cone_direction_test::V, 12>;

struct Geometry : AffineConeGeometry {
  sct::FacetQuadraticCoefficients coefficients[2]{ZeroQuadratic(), ZeroQuadratic()};
  c::Vec3 controls[12]{};

  Geometry() {
    // Endpoint signed arms lie on epsilon*v + span(u), u=(1,2), v=(2,-1).
    // Interior controls lie on eta*v + span(u-alpha*v). Their feasible cone
    // is narrower and tilted: the endpoint-only direction v is now invalid.
    constexpr double alpha = 1.0 / 1024, eta = 1.0 / 1048576;
    constexpr double along[2][2]{{1, -2}, {-1, 3}};
    const c::CurrentFixedTriangle* triangles[]{&first, &second};
    unsigned count = 0;
    for (unsigned side = 0; side < 2; ++side)
      for (unsigned arm = 0; arm < 2; ++arm) {
        const double a = along[side][arm];
        const double offset = eta - alpha * a;
        const c::Vec3 middle{a + 2 * offset, 2 * a - offset, 0};
        const auto endpoint = c::Scale(triangles[side]->vertices[arm + 1], side ? -1 : 1);
        controls[count++] = endpoint;
        controls[count++] = middle;
        controls[count++] = endpoint;
        const auto raw_middle = c::Scale(middle, side ? -1 : 1);
        const auto q = c::Scale(c::Subtract(triangles[side]->vertices[arm + 1], raw_middle), 4);
        coefficients[side].q[arm + 1][0] = {q.x, q.x};
        coefficients[side].q[arm + 1][1] = {q.y, q.y};
      }
  }

  ExactControls Exact() const {
    ExactControls result;
    for (unsigned i = 0; i < 12; ++i) result[i] = cone_direction_test::Exact(controls[i]);
    return result;
  }
  sct::CurvedConeSearchComparison Compare(std::size_t work = 1, unsigned depth = 0) const {
    return sct::CompareCurvedConeSearch(first, first, coefficients[0],
        second, second, coefficients[1], 1, work, depth);
  }
};

bool StreamedStrict(const c::Vec3 (&rays)[12], unsigned* visited) {
  ExactControls exact;
  for (unsigned i = 0; i < 12; ++i) exact[i] = cone_direction_test::Exact(rays[i]);
  sct::CurvedConeDirections stream(rays);
  c::Vec3 axis;
  while (stream.Next(&axis))
    if (c::IsFinite(axis) && cone_direction_test::Strict(exact, cone_direction_test::Exact(axis))) {
      *visited = stream.emitted();
      return true;
    }
  *visited = stream.emitted();
  return false;
}
}  // namespace curved_cone_test

TEST(SelfContactCurvedCone, IteratorStreamsTwelveSingletonsSixtySixEdgesAndTwoHundredTwentyFaces) {
  const curved_cone_test::Geometry geometry;
  sct::CurvedConeDirections stream(geometry.controls);
  c::Vec3 actual;
  EXPECT_FALSE(stream.Next(nullptr));
  EXPECT_EQ(stream.emitted(), 0u);
  const auto same = [&](c::Vec3 expected) {
    ASSERT_TRUE(stream.Next(&actual));
    EXPECT_EQ(actual.x, expected.x);
    EXPECT_EQ(actual.y, expected.y);
    EXPECT_EQ(actual.z, expected.z);
  };
  const auto& rays = geometry.controls;
  for (const auto ray : rays) same(ray);
  for (unsigned i = 0; i < 12; ++i) for (unsigned j = i + 1; j < 12; ++j) {
    const auto e = c::Subtract(rays[j], rays[i]);
    same(c::geometry_detail::Cross(e, c::geometry_detail::Cross(rays[i], e)));
  }
  for (unsigned i = 0; i < 12; ++i) for (unsigned j = i + 1; j < 12; ++j)
    for (unsigned k = j + 1; k < 12; ++k)
      same(c::geometry_detail::Cross(c::Subtract(rays[j], rays[i]), c::Subtract(rays[k], rays[i])));
  EXPECT_EQ(stream.emitted(), 298u);
  EXPECT_FALSE(stream.Next(&actual));
  EXPECT_FALSE(stream.Next(&actual));
}

TEST(SelfContactCurvedCone, InteriorControlsFindWholeCurveProofThatEndpointDirectionsCannotSupply) {
  const curved_cone_test::Geometry geometry;
  const auto exact = geometry.Exact();
  ASSERT_TRUE(cone_direction_test::Feasible(exact));
  // Independent exact normal to u-alpha*v; all twelve controls must be strict.
  ASSERT_TRUE(cone_direction_test::Strict(exact, cone_direction_test::Exact(
      c::Vec3{2 + 1.0 / 1024, -1 + 2.0 / 1024, 0})));
  c::Vec3 endpoints[8];
  unsigned count = 0;
  for (unsigned endpoint : {0u, 2u})
    for (unsigned arm = 0; arm < 4; ++arm) endpoints[count++] = geometry.controls[3 * arm + endpoint];
  sct::ConeDirections endpoint_search(endpoints);
  c::Vec3 direction;
  while (endpoint_search.Next(&direction))
    if (c::IsFinite(direction))
      EXPECT_FALSE(cone_direction_test::Strict(exact, cone_direction_test::Exact(direction)));
  const auto result = geometry.Compare();
  EXPECT_NE(result.original.report.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_EQ(result.current.report.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_EQ(result.current.report.work, 1u);
  EXPECT_EQ(result.current.counters.affine_searches, 0u);
  EXPECT_EQ(result.current.counters.curved_searches, 1u);
  EXPECT_GT(result.current.counters.curved_directions, 12u);
  EXPECT_LE(result.current.counters.curved_directions, 298u);
  EXPECT_FALSE(result.current.counters.saturated);
  sct::test::ExpectResult(sct::CertifyQuadraticLocalTopology(
      geometry.first, geometry.first, geometry.coefficients[0],
      geometry.second, geometry.second, geometry.coefficients[1], 1, 1, 0), result.current.report);
}

TEST(SelfContactCurvedCone, LocalGeometryCannotOverridePositiveThicknessAndSmallCurvatureCanPassBoth) {
  const curved_cone_test::Geometry geometry;
  const auto contact = sct::CertifyQuadraticLocalContact(
      geometry.first, geometry.first, geometry.coefficients[0], .001,
      geometry.second, geometry.second, geometry.coefficients[1], .001, 1, 255, 8);
  EXPECT_NE(contact.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  auto small = ZeroQuadratic();
  small.q[1][0] = {-std::ldexp(geometry.first.vertices[1].x, -30),
                   -std::ldexp(geometry.first.vertices[1].x, -30)};
  small.q[1][1] = {-std::ldexp(geometry.first.vertices[1].y, -30),
                   -std::ldexp(geometry.first.vertices[1].y, -30)};
  const auto admitted = sct::CertifyQuadraticLocalContact(
      geometry.first, geometry.first, small, .001,
      geometry.second, geometry.second, ZeroQuadratic(), .001, 1, 255, 8);
  ASSERT_EQ(admitted.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  const auto policy = sct::CertifyQuadraticFacetPolicyCoverage(
      geometry.first, geometry.first, small, .001,
      geometry.second, geometry.second, ZeroQuadratic(), .001, 1,
      nullptr, 0, nullptr, 0, 255, 8);
  sct::test::ExpectResult(policy, admitted);
}

TEST(SelfContactCurvedCone, IdenticalLocalEndpointsDoNotHideARealInteriorNonlocalIntersection) {
  const AffineConeGeometry geometry;
  auto crossing = ZeroQuadratic();
  // x(1/2)=x0-q/8. At the midpoint, a distinct B source vertex exactly
  // reaches A's nonshared vertex, although both endpoint geometries are local.
  const auto q = c::Scale(c::Subtract(geometry.second.vertices[1], geometry.first.vertices[1]), 8);
  crossing.q[1][0] = {q.x, q.x};
  crossing.q[1][1] = {q.y, q.y};
  auto midpoint = geometry.second;
  midpoint.vertices[1] = geometry.first.vertices[1];
  c::FixedTriangleIntersection intersection;
  bool intersects = false;
  ASSERT_EQ(c::fixed_triangle_features::ClassifyPairIntersection(
      geometry.first, midpoint, &intersection, &intersects), c::FixedTriangleDiscoveryStatus::Ok);
  ASSERT_TRUE(intersects);
  ASSERT_TRUE(c::RequiresIntersectionAdmission(intersection));
  for (unsigned depth : {0u, 4u, 12u}) {
    const auto result = sct::CompareCurvedConeSearch(geometry.first, geometry.first, ZeroQuadratic(),
        geometry.second, geometry.second, crossing, 1, 255, depth);
    sct::test::ExpectResult(result.current.report, result.original.report);
    EXPECT_NE(result.current.report.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
    EXPECT_LE(result.current.counters.curved_searches, 1u);
    EXPECT_LE(result.current.counters.curved_directions, 298u);
  }
}

TEST(SelfContactCurvedCone, DirectedCoefficientUncertaintyCannotBorrowRepresentativeAuthority) {
  curved_cone_test::Geometry geometry;
  ASSERT_EQ(geometry.Compare().current.report.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  // Same representatives, but uncertain coefficients admit both signs of the
  // narrow whole-curve gap. A midpoint-generated direction grants no authority.
  for (auto& coefficients : geometry.coefficients)
    for (unsigned vertex = 1; vertex < 3; ++vertex)
      for (unsigned component = 0; component < 2; ++component) {
        coefficients.q[vertex][component].lower -= .25;
        coefficients.q[vertex][component].upper += .25;
      }
  const auto uncertain = geometry.Compare();
  sct::test::ExpectResult(uncertain.current.report, uncertain.original.report);
  EXPECT_NE(uncertain.current.report.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_LE(uncertain.current.counters.curved_directions, 298u);
}

TEST(SelfContactCurvedCone, GenericFeasibilitySurvivesRotationsAndExtremeDirectionsFailClosed) {
  const curved_cone_test::Geometry geometry;
  for (int exponent : {-512, -100, 0, 100, 512}) for (const auto& axes : ConePermutations) {
    SCOPED_TRACE(exponent);
    c::Vec3 rays[12];
    curved_cone_test::ExactControls exact;
    for (unsigned i = 0; i < 12; ++i) {
      const double values[]{geometry.controls[i].x, geometry.controls[i].y, geometry.controls[i].z};
      rays[i] = {std::ldexp(values[axes[0]], exponent), std::ldexp(values[axes[1]], exponent),
                 std::ldexp(values[axes[2]], exponent)};
      exact[i] = cone_direction_test::Exact(rays[i]);
    }
    ASSERT_TRUE(cone_direction_test::Feasible(exact));
    unsigned visited = 0;
    EXPECT_EQ(curved_cone_test::StreamedStrict(rays, &visited), std::abs(exponent) < 512);
    EXPECT_LE(visited, 298u);
    if (std::abs(exponent) == 512) EXPECT_EQ(visited, 298u);
  }
}

TEST(SelfContactCurvedCone, InfeasibleRootConsumesOneBoundedSearchAndKeepsRecursiveFallback) {
  const AffineConeGeometry geometry;
  sct::FacetQuadraticCoefficients q[2]{ZeroQuadratic(), ZeroQuadratic()};
  const c::CurrentFixedTriangle* triangles[]{&geometry.first, &geometry.second};
  curved_cone_test::ExactControls exact;
  unsigned count = 0;
  for (unsigned side = 0; side < 2; ++side)
    for (unsigned vertex = 1; vertex < 3; ++vertex) {
      const auto point = triangles[side]->vertices[vertex];
      const c::Vec3 middle{point.x - point.y, point.x + point.y, point.z};
      q[side].q[vertex][0] = {4 * point.y, 4 * point.y};
      q[side].q[vertex][1] = {-4 * point.x, -4 * point.x};
      for (const auto control : {point, middle, point})
        exact[count++] = cone_direction_test::Exact(c::Scale(control, side ? -1 : 1));
    }
  // The common map I+s*J has determinant 1+s*s for all times; a constant
  // root direction still cannot separate the union of the rotated controls.
  ASSERT_FALSE(cone_direction_test::Feasible(exact));
  const auto result = sct::CompareCurvedConeSearch(geometry.first, geometry.first, q[0],
      geometry.second, geometry.second, q[1], 1, 255, 8);
  sct::test::ExpectResult(result.current.report, result.original.report);
  EXPECT_EQ(result.current.counters.curved_searches, 1u);
  EXPECT_EQ(result.current.counters.curved_directions, 298u);
  EXPECT_GT(result.current.counters.cells, 1u);
}

TEST(SelfContactCurvedCone, AffineAndExistingCurvedWinnersKeepTheirReportsAndSearchOrder) {
  const AffineConeGeometry affine;
  const auto old = sct::CompareAffineConeSearch(affine.first, affine.first, ZeroQuadratic(),
      affine.second, affine.second, ZeroQuadratic(), 1, 255, 8);
  const auto current = sct::CompareCurvedConeSearch(affine.first, affine.first, ZeroQuadratic(),
      affine.second, affine.second, ZeroQuadratic(), 1, 255, 8);
  sct::test::ExpectResult(current.current.report, old.current.report);
  sct::test::ExpectResult(current.current.report, current.original.report);
  EXPECT_EQ(current.current.counters.affine_directions, old.current.counters.affine_directions);
  EXPECT_EQ(current.current.counters.curved_searches, 0u);
  const auto first = Triangle(10, {1, 2, 3}, {{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}});
  const auto second = Triangle(20, {1, 4, 5}, {{{0, 0, 0}, {0, -1, 1}, {0, -1, -1}}});
  auto q = ZeroQuadratic();
  q.q[1][2] = {1.0 / 16, 1.0 / 16};
  const auto winner = sct::CompareCurvedConeSearch(first, first, ZeroQuadratic(), second, second, q, 1, 1, 0);
  ASSERT_EQ(winner.original.report.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  sct::test::ExpectResult(winner.current.report, winner.original.report);
  EXPECT_EQ(winner.current.counters.curved_searches, 0u);
}

TEST(SelfContactCurvedCone, SourceAndBudgetPremisesAndGeneralLedgerNeverGainSearchAuthority) {
  const curved_cone_test::Geometry geometry;
  for (unsigned mutation = 0; mutation < 4; ++mutation) {
    auto prepared = geometry.second;
    auto q = geometry.coefficients[1];
    if (mutation == 0) prepared.vertex_keys[0].source_instance_id++;
    if (mutation == 1) prepared.vertices[0].x = .01;
    if (mutation == 2) q.q[0][0] = {.01, .01};
    if (mutation == 3) q.complete = false;
    const auto result = sct::CompareCurvedConeSearch(geometry.first, geometry.first, geometry.coefficients[0],
        geometry.second, prepared, q, 1, 255, 8);
    sct::test::ExpectResult(result.current.report, result.original.report);
    EXPECT_EQ(result.current.counters.curved_searches, 0u);
  }
  EXPECT_EQ(geometry.Compare(0, 0).current.counters.curved_searches, 0u);
  EXPECT_EQ(geometry.Compare(1, 53).current.counters.curved_searches, 0u);
  const auto ledger = sct::CompareSharedVertexCoverageOrders(
      geometry.first, geometry.first, geometry.coefficients[0], .001,
      geometry.second, geometry.second, geometry.coefficients[1], .001, 1, nullptr, 0, 255, 8);
  EXPECT_EQ(ledger.polynomial_first.counters.curved_searches, 0u);
  EXPECT_EQ(ledger.cone_first.counters.curved_searches, 0u);
}
