// SPDX-License-Identifier: AGPL-3.0-or-later
// Included inside ContinuousLocalTest.cpp after the native fixture helpers.

struct ConeAxisTransform {
  std::array<unsigned, 3> axes;
  std::array<int, 3> signs;
};

constexpr std::array<std::array<unsigned, 3>, 6> ConePermutations{{
    {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
    {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};

constexpr std::array<ConeAxisTransform, 6> ConeRotations{{
    {{{0, 1, 2}}, {{1, 1, 1}}},
    {{{1, 0, 2}}, {{-1, 1, 1}}},
    {{{2, 1, 0}}, {{1, 1, -1}}},
    {{{0, 2, 1}}, {{1, -1, 1}}},
    {{{1, 2, 0}}, {{1, 1, 1}}},
    {{{0, 1, 2}}, {{-1, -1, 1}}}}};

c::Vec3 TransformConePoint(c::Vec3 point, const ConeAxisTransform& transform) {
  const double values[]{point.x, point.y, point.z};
  return {transform.signs[0] * values[transform.axes[0]],
          transform.signs[1] * values[transform.axes[1]],
          transform.signs[2] * values[transform.axes[2]]};
}

c::CurrentFixedTriangle TransformConeTriangle(
    const c::CurrentFixedTriangle& triangle,
    const ConeAxisTransform& transform,
    const std::array<unsigned, 3>& order = ConePermutations[0]) {
  std::array<c::Vec3, 3> points;
  std::array<std::uint64_t, 3> ids;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    points[vertex] = TransformConePoint(triangle.vertices[order[vertex]], transform);
    ids[vertex] = triangle.vertex_keys[order[vertex]].first;
  }
  return Triangle(triangle.key.parent_eid, ids, points);
}

sct::FacetQuadraticCoefficients TransformConeQuadratic(
    const sct::FacetQuadraticCoefficients& coefficients,
    const ConeAxisTransform& transform) {
  auto result = coefficients;
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      const auto value = coefficients.q[vertex][transform.axes[axis]];
      result.q[vertex][axis] = transform.signs[axis] > 0
          ? value : sct::DirectedInterval{-value.upper, -value.lower};
    }
  }
  return result;
}

struct CapturedConeGeometry {
  // Exact coordinate_bits from failure8, manifest SHA-256
  // 3b2ab9990ad1dbf5c98a27c28420d2fb9e0847b2226337eb519a69e8d3559da9.
  // Source IDs are deliberately generic; only shared source identity matters.
  c::CurrentFixedTriangle first = Triangle(10, {1, 2, 3}, {{{
      -0x1.d4d332ed6822dp-2, -0x1.71fbe83ab3d28p-9, 0x1.c3315e1bea167p-2}, {
      -0x1.d4da9f734d54ap-2, -0x1.a4189261dbac0p-7, 0x1.c2fcd4d254cc1p-2}, {
      -0x1.cd807f4153b26p-2, -0x1.ab3332650aa45p-7, 0x1.cd00439cb7dc3p-2}}});
  c::CurrentFixedTriangle first_prepared = Triangle(10, {1, 2, 3}, {{{
      -0x1.d4d260ed1ad8dp-2, -0x1.71fbed07667a2p-9, 0x1.c3315e1b70748p-2}, {
      -0x1.d4d9cd72ae582p-2, -0x1.a4189261dbac0p-7, 0x1.c2fcd4d254cc1p-2}, {
      -0x1.cd7fad40b4b5ep-2, -0x1.ab3332650aa45p-7, 0x1.cd00439cb7dc3p-2}}});
  c::CurrentFixedTriangle second = Triangle(20, {2, 4, 5}, {{{
      -0x1.d4da9f734d54ap-2, -0x1.a4189261dbac0p-7, 0x1.c2fcd4d254cc1p-2}, {
      -0x1.d4e207f267bc0p-2, -0x1.75db2269d118dp-6, 0x1.c2c84b88bf81cp-2}, {
      -0x1.cd87e7c06e19cp-2, -0x1.7968726b68950p-6, 0x1.cccbba532291dp-2}}});
  c::CurrentFixedTriangle second_prepared = Triangle(20, {2, 4, 5}, {{{
      -0x1.d4d9cd72ae582p-2, -0x1.a4189261dbac0p-7, 0x1.c2fcd4d254cc1p-2}, {
      -0x1.d4e135f1c8bf8p-2, -0x1.75db2269d118dp-6, 0x1.c2c84b88bf81cp-2}, {
      -0x1.cd8715bfcf1d4p-2, -0x1.7968726b68950p-6, 0x1.cccbba532291dp-2}}});
};

void ExpectCapturedConeRootProof(const CapturedConeGeometry& geometry) {
  const auto zero = ZeroQuadratic();
  const auto local = sct::CertifyQuadraticLocalContact(
      geometry.first, geometry.first_prepared, zero, 0.0005,
      geometry.second, geometry.second_prepared, zero, 0.0005, 2e-7, 1, 0);
  const auto policy = sct::CertifyQuadraticFacetPolicyCoverage(
      geometry.first, geometry.first_prepared, zero, 0.0005,
      geometry.second, geometry.second_prepared, zero, 0.0005, 2e-7,
      nullptr, 0, nullptr, 0, 1, 0);
  for (const auto& result : {local, policy}) {
    EXPECT_EQ(result.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
    EXPECT_EQ(result.work, 1u);
    EXPECT_EQ(result.deepest, 0u);
    EXPECT_EQ(result.accepted_certificate, SIZE_MAX);
    EXPECT_FALSE(result.work_exhausted);
    EXPECT_FALSE(result.depth_exhausted);
  }
}

TEST(SelfContactConeDiagonals, CapturedAffineCoordinatesNeedOneRootProofWithoutSourceSpecialCases) {
  ExpectCapturedConeRootProof(CapturedConeGeometry{});
}

TEST(SelfContactConeDiagonals, AllVertexOrdersRetainSharedIdentityAndRootProof) {
  const CapturedConeGeometry source;
  for (std::size_t index = 0; index < ConePermutations.size(); ++index) {
    SCOPED_TRACE(index);
    const auto& first_order = ConePermutations[index];
    const auto& second_order = ConePermutations[ConePermutations.size() - 1 - index];
    CapturedConeGeometry transformed;
    transformed.first = TransformConeTriangle(source.first, ConeRotations[0], first_order);
    transformed.first_prepared = TransformConeTriangle(source.first_prepared, ConeRotations[0], first_order);
    transformed.second = TransformConeTriangle(source.second, ConeRotations[0], second_order);
    transformed.second_prepared = TransformConeTriangle(source.second_prepared, ConeRotations[0], second_order);
    ExpectCapturedConeRootProof(transformed);
  }
}

TEST(SelfContactConeDiagonals, SignedCoordinatePermutationsAndQuarterTurnsRetainRootProof) {
  const CapturedConeGeometry source;
  const auto check = [&](const ConeAxisTransform& transform) {
    CapturedConeGeometry transformed;
    transformed.first = TransformConeTriangle(source.first, transform);
    transformed.first_prepared = TransformConeTriangle(source.first_prepared, transform);
    transformed.second = TransformConeTriangle(source.second, transform);
    transformed.second_prepared = TransformConeTriangle(source.second_prepared, transform);
    ExpectCapturedConeRootProof(transformed);
  };
  for (std::size_t index = 0; index < ConePermutations.size(); ++index) {
    for (unsigned signs = 0; signs < 8; ++signs) {
      SCOPED_TRACE(::testing::Message() << "axis permutation=" << index << " signs=" << signs);
      check({ConePermutations[index], {{signs & 1 ? -1 : 1,
                                      signs & 2 ? -1 : 1,
                                      signs & 4 ? -1 : 1}}});
    }
  }
  for (const auto& rotation : ConeRotations) check(rotation);
}

TEST(SelfContactConeDiagonals, DyadicBoundaryRequiresStrictConeSeparation) {
  for (double delta : {0x1p-30, 0.0, -0x1p-30}) {
    SCOPED_TRACE(delta);
    const auto first = Triangle(10, {1, 2, 3}, {{{0, 0, 0}, {0, 1, 0}, {1, -1 + delta, 0}}});
    const auto second = Triangle(20, {1, 4, 5}, {{{0, 0, 0}, {-1, 0, 0}, {1, -1 - delta, 0}}});
    const auto result = sct::CertifyQuadraticLocalTopology(
        first, first, ZeroQuadratic(), second, second, ZeroQuadratic(), 2e-7, 1, 0);
    c::FixedTriangleIntersection intersection;
    bool intersects = false;
    ASSERT_EQ(c::fixed_triangle_features::ClassifyPairIntersection(
        first, second, &intersection, &intersects), c::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_TRUE(intersects);
    if (delta > 0) {
      EXPECT_FALSE(c::RequiresIntersectionAdmission(intersection));
      EXPECT_EQ(result.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
      EXPECT_EQ(result.work, 1u);
    } else {
      EXPECT_TRUE(c::RequiresIntersectionAdmission(intersection));
      EXPECT_NE(result.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
    }
  }
}

TEST(SelfContactConeDiagonals, RotatedActualRigidMidintervalCrossingRemainsRejected) {
  CurvedSharedVertex source;
  ASSERT_NO_FATAL_FAILURE(source.DeriveActualRigidCoefficient());
  for (std::size_t index = 0; index < ConeRotations.size(); ++index) {
    SCOPED_TRACE(index);
    const auto first = TransformConeTriangle(source.first, ConeRotations[index]);
    const auto second = TransformConeTriangle(source.second, ConeRotations[index]);
    const auto curved = TransformConeQuadratic(source.curved, ConeRotations[index]);
    auto middle = source.second;
    middle.vertices[1].z -= source.curved.q[1][2].lower / 8;
    middle = TransformConeTriangle(middle, ConeRotations[index]);
    c::FixedTriangleIntersection intersection;
    bool intersects = false;
    ASSERT_EQ(c::fixed_triangle_features::ClassifyPairIntersection(
        first, middle, &intersection, &intersects), c::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_TRUE(intersects);
    ASSERT_TRUE(c::RequiresIntersectionAdmission(intersection));
    EXPECT_NE(sct::CertifyQuadraticLocalTopology(
        first, first, ZeroQuadratic(), second, second, curved, Duration, 255, 12).status,
        sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
    EXPECT_NE(LocalPolicy(first, second, second, curved).status,
              sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
  }
}

TEST(SelfContactConeDiagonals, MalformedSharedTrajectoryDegeneracyAndZeroWorkFailClosed) {
  const CapturedConeGeometry source;
  auto malformed = source.second_prepared;
  malformed.vertices[0].x += 0x1p-30;
  auto degenerate = source.second;
  degenerate.vertices[2] = degenerate.vertices[1];
  const auto zero = ZeroQuadratic();
  const auto shared = sct::CertifyQuadraticLocalTopology(
      source.first, source.first_prepared, zero,
      source.second, malformed, zero, 2e-7, 255, 12);
  const auto collapsed = sct::CertifyQuadraticLocalTopology(
      source.first, source.first, zero, degenerate, degenerate, zero, 2e-7, 255, 12);
  const auto no_work = sct::CertifyQuadraticLocalTopology(
      source.first, source.first_prepared, zero,
      source.second, source.second_prepared, zero, 2e-7, 0, 0);
  for (const auto& result : {shared, collapsed, no_work})
    EXPECT_NE(result.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
}
