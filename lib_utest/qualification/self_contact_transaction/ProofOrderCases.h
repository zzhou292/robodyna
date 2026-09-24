// SPDX-License-Identifier: MIT
#pragma once

// Uses ContinuousLocalTest's native fixtures and the captured cone coordinates.
sct::SharedVertexProofOrderComparison CompareTopologyOrder(
    const c::CurrentFixedTriangle& first,
    const c::CurrentFixedTriangle& first_prepared,
    const sct::FacetQuadraticCoefficients& first_coefficients,
    const c::CurrentFixedTriangle& second,
    const c::CurrentFixedTriangle& second_prepared,
    const sct::FacetQuadraticCoefficients& second_coefficients,
    double duration = Duration, std::size_t work = 255, unsigned depth = 12) {
  const auto compared = sct::CompareSharedVertexTopologyOrders(
      first, first_prepared, first_coefficients,
      second, second_prepared, second_coefficients, duration, work, depth);
  sct::test::ExpectResult(compared.cone_first.report, compared.polynomial_first.report);
  const auto actual = sct::CertifyQuadraticLocalTopology(
      first, first_prepared, first_coefficients,
      second, second_prepared, second_coefficients, duration, work, depth);
  sct::test::ExpectResult(actual, compared.cone_first.report);
  EXPECT_FALSE(compared.polynomial_first.counters.saturated);
  EXPECT_FALSE(compared.cone_first.counters.saturated);
  EXPECT_EQ(compared.polynomial_first.counters.cells, compared.cone_first.counters.cells);
  EXPECT_EQ(compared.polynomial_first.counters.endpoint_classifications,
            compared.cone_first.counters.endpoint_classifications);
  return compared;
}

TEST(SelfContactProofOrder, CapturedConeSuccessSkipsThirteenPolynomialTasksAtSameProofWork) {
  const CapturedConeGeometry geometry;
  const auto zero = ZeroQuadratic();
  const auto result = CompareTopologyOrder(
      geometry.first, geometry.first_prepared, zero,
      geometry.second, geometry.second_prepared, zero, 2e-7, 1, 0);
  ASSERT_EQ(result.cone_first.report.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_EQ(result.cone_first.report.work, 1u);
  EXPECT_EQ(result.polynomial_first.counters.vertex_face_tasks, 4u);
  EXPECT_EQ(result.polynomial_first.counters.nonincident_edge_tasks, 5u);
  EXPECT_EQ(result.polynomial_first.counters.incident_edge_tasks, 4u);
  EXPECT_EQ(result.cone_first.counters.vertex_face_tasks, 0u);
  EXPECT_EQ(result.cone_first.counters.nonincident_edge_tasks, 0u);
  EXPECT_EQ(result.cone_first.counters.incident_edge_tasks, 0u);
  EXPECT_EQ(result.polynomial_first.counters.cone_calls, 1u);
  EXPECT_EQ(result.cone_first.counters.cone_calls, 1u);
  EXPECT_EQ(result.cone_first.counters.cone_axes, result.polynomial_first.counters.cone_axes);
}

TEST(SelfContactProofOrder, ExistingRootAndConeProofsChooseTheSameReport) {
  const auto first = Triangle(10, {1, 2, 3}, {{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}});
  const auto second = Triangle(20, {1, 4, 5}, {{{0, 0, 0}, {0, -1, 1}, {0, -1, -1}}});
  const auto result = CompareTopologyOrder(first, first, ZeroQuadratic(),
      second, second, ZeroQuadratic(), 1, 1, 0);
  ASSERT_EQ(result.cone_first.report.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_EQ(result.cone_first.counters.vertex_face_tasks, 0u);
  EXPECT_EQ(result.polynomial_first.counters.vertex_face_tasks, 4u);
  EXPECT_EQ(result.polynomial_first.counters.cone_calls, 0u);
}

TEST(SelfContactProofOrder, CapturedGeometryVertexAndSignedAxisPermutationsAreFieldwiseEqual) {
  const CapturedConeGeometry source;
  for (const auto& axes : ConePermutations) {
    for (unsigned signs = 0; signs < 8; ++signs) {
      const ConeAxisTransform transform{axes, {{signs & 1 ? -1 : 1,
                                               signs & 2 ? -1 : 1,
                                               signs & 4 ? -1 : 1}}};
      for (const auto& order : ConePermutations) {
        const auto first = TransformConeTriangle(source.first, transform, order);
        const auto first_next = TransformConeTriangle(source.first_prepared, transform, order);
        const auto second = TransformConeTriangle(source.second, transform, order);
        const auto second_next = TransformConeTriangle(source.second_prepared, transform, order);
        CompareTopologyOrder(first, first_next, ZeroQuadratic(),
                             second, second_next, ZeroQuadratic(), 2e-7, 1, 0);
      }
    }
  }
}

TEST(SelfContactProofOrder, ActualRigidInteriorCrossingsAndClosedBoundariesRetainFailures) {
  CurvedSharedVertex source;
  source.DeriveActualRigidCoefficient();
  for (const auto& transform : ConeRotations) {
    const auto first = TransformConeTriangle(source.first, transform);
    const auto second = TransformConeTriangle(source.second, transform);
    const auto curved = TransformConeQuadratic(source.curved, transform);
    for (std::size_t work : {0u, 1u, 255u})
      for (unsigned depth : {0u, 4u, 12u, 53u}) {
        const auto result = CompareTopologyOrder(first, first, ZeroQuadratic(),
            second, second, curved, Duration, work, depth);
        EXPECT_NE(result.cone_first.report.status,
                  sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
      }
  }
}

TEST(SelfContactProofOrder, DegeneracySharedPathNonfiniteAndOverflowRemainFailClosed) {
  const CapturedConeGeometry source;
  for (unsigned mutation = 0; mutation < 7; ++mutation) {
    auto first = source.first;
    auto first_next = source.first_prepared;
    auto second = source.second;
    auto second_next = source.second_prepared;
    auto quadratic = ZeroQuadratic();
    if (mutation == 0) second_next.vertices[0].x += 1;
    if (mutation == 1) first_next.vertices[1] = first_next.vertices[0];
    if (mutation == 2) quadratic.q[0][0] = {1, 1};
    if (mutation == 3) quadratic.q[0][0] = {NAN, NAN};
    if (mutation == 4) second_next.vertices[1].x = INFINITY;
    if (mutation == 5) {
      for (auto* triangle : {&first, &first_next, &second, &second_next})
        for (auto& vertex : triangle->vertices) vertex.x *= 1e308;
    }
    if (mutation == 6) second_next.vertex_keys[0].source_instance_id++;
    SCOPED_TRACE(mutation);
    CompareTopologyOrder(first, first_next, ZeroQuadratic(),
        second, second_next, quadratic, 2e-7, 31, 4);
  }
}

TEST(SelfContactProofOrder, DyadicStaticAndCurvedFamiliesPreserveEveryReportField) {
  const auto first = Triangle(10, {1, 2, 3}, {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  for (int x : {-2, -1, 1, 2})
    for (int y : {-2, -1, 1, 2})
      for (int z : {-1, 0, 1}) {
        const auto second = Triangle(20, {1, 4, 5},
            {{{0, 0, 0}, {double(x), double(y), double(z)},
              {double(x + 1), double(y), double(z - 1)}}});
        for (double curve : {-1.0 / 16, 0.0, 1.0 / 16}) {
          auto quadratic = ZeroQuadratic();
          quadratic.q[1][2] = {curve, curve};
          CompareTopologyOrder(first, first, ZeroQuadratic(),
              second, second, quadratic, 1, 63, 4);
        }
      }
}

TEST(SelfContactProofOrder, NarrowConeMissPreservesTheIndependentPolynomialProof) {
  // u=(1,2,3), v=(2,-1,0), w=(3,6,-5) are mutually orthogonal.
  // A arms v+e*u,-2v+e*u lie strictly on +u; B arms w-e*u,-3w-e*u
  // lie on -u. Their cones meet only at the origin. Unequal arm lengths
  // keep the existing finite axis search from discovering the narrow u cone;
  // the unchanged exact polynomial alternative must still certify the cell.
  constexpr double e = 1.0 / 1024;
  const auto first = Triangle(10, {1, 2, 3}, {{{0, 0, 0},
      {2+e, -1+2*e, 3*e}, {-4+e, 2+2*e, 3*e}}});
  const auto second = Triangle(20, {1, 4, 5}, {{{0, 0, 0},
      {3-e, 6-2*e, -5-3*e}, {-9-e, -18-2*e, 15-3*e}}});
  const auto compared = CompareTopologyOrder(first, first, ZeroQuadratic(),
      second, second, ZeroQuadratic(), 1, 1, 0);
  ASSERT_EQ(compared.cone_first.report.status,
            sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  EXPECT_EQ(compared.polynomial_first.counters.cone_calls, 0u);
  EXPECT_EQ(compared.cone_first.counters.cone_calls, 1u);
  EXPECT_EQ(compared.cone_first.counters.cone_axes, 20u);
  EXPECT_EQ(compared.cone_first.counters.vertex_face_tasks, 4u);
  EXPECT_EQ(compared.cone_first.counters.nonincident_edge_tasks, 5u);
  EXPECT_EQ(compared.cone_first.counters.incident_edge_tasks, 4u);
}
