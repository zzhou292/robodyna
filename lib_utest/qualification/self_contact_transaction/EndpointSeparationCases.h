// SPDX-License-Identifier: MIT
#pragma once
// Included in the existing rigid-sweep fixture namespace. Reuse its actual
// TriangleAt/Coefficients construction and complete fieldwise result checks.

TEST(SelfContactRigidSweepBounds, ClosedEndpointWitnessStopsOnlyAnImpossibleSeparationSearch) {
  const auto fixed = TriangleAt({});
  const auto apart = TriangleAt({0, 0, 1});
  const auto zero = Coefficients({});
  const auto curved = Coefficients({0, .5, .25});
  for (bool at_end : {false, true}) {
    auto first = at_end ? apart : fixed;
    auto last = at_end ? fixed : apart;
    for (unsigned permutation = 0; permutation < 3; ++permutation) {
      const auto comparison = sct::CompareRigidSeparationEndpointWitness(
          fixed, fixed, zero, .01, first, last, curved, .01, 1, 64, 20);
      EXPECT_NE(comparison.original.status, sct::NonlinearSeparationStatus::CertifiedSeparated);
      EXPECT_GT(comparison.original.work, 1u);
      sct::NonlinearSeparationResult expected;
      expected.status = sct::NonlinearSeparationStatus::PotentialContact;
      expected.work = 1;
      sct::test::ExpectResult(comparison.current, expected);
      const auto reversed = sct::CertifyQuadraticFacetSeparation(
          first, last, curved, .01, fixed, fixed, zero, .01, 1, 64, 20);
      sct::test::ExpectResult(reversed, expected);
      std::rotate(first.vertices, first.vertices + 1, first.vertices + 3);
      std::rotate(last.vertices, last.vertices + 1, last.vertices + 3);
    }
  }
}

TEST(SelfContactRigidSweepBounds, CoincidenceAtDifferentTimesDoesNotPreventRealSeparation) {
  const auto a0 = TriangleAt({-1, 0, 0});
  const auto a1 = TriangleAt({1, 0, 0});
  const auto result = sct::CompareRigidSeparationEndpointWitness(
      a0, a1, Coefficients({0, -8, 0}), .01,
      a1, a0, Coefficients({0, 8, 0}), .01, 1, 4095, 20);
  ASSERT_EQ(result.original.status, sct::NonlinearSeparationStatus::CertifiedSeparated);
  EXPECT_GT(result.original.work, 1u);
  sct::test::ExpectResult(result.current, result.original);
}

TEST(SelfContactRigidSweepBounds, OverlappingInteriorEnclosuresAreNotSingletonEndpointWitnesses) {
  const auto fixed = TriangleAt({});
  const auto moving = TriangleAt({0, 0, 1});
  // The middle control enclosures overlap z=0, but the actual quadratic stays
  // above z=.5 and needs both child certificates. Reusing that overlap as an
  // exact endpoint witness would incorrectly block this separated case.
  const auto result = sct::CompareRigidSeparationEndpointWitness(
      fixed, fixed, Coefficients({}), .01,
      moving, moving, Coefficients({0, 0, 4}), .01, 1, 4095, 20);
  ASSERT_EQ(result.original.status, sct::NonlinearSeparationStatus::CertifiedSeparated);
  ASSERT_EQ(result.original.work, 3u);
  sct::test::ExpectResult(result.current, result.original);
}

TEST(SelfContactRigidSweepBounds, SignedZeroMatchesButAdjacentRepresentableEndpointsDoNot) {
  const auto first = TriangleAt({});
  auto signed_zero = first;
  for (auto& vertex : signed_zero.vertices) vertex.z = -0.0;
  const auto zero = Coefficients({});
  const auto matched = sct::CompareRigidSeparationEndpointWitness(
      first, first, zero, .01, signed_zero, signed_zero, zero, .01, 1, 64, 8);
  EXPECT_EQ(matched.current.status, sct::NonlinearSeparationStatus::PotentialContact);
  EXPECT_EQ(matched.current.work, 1u);
  for (double gap : {std::numeric_limits<double>::denorm_min(), std::nextafter(1.0, 2.0) - 1.0}) {
    auto near = first;
    for (auto& vertex : near.vertices) vertex.z = gap;
    const auto result = sct::CompareRigidSeparationEndpointWitness(
        first, first, zero, .01, near, near, zero, .01, 1, 64, 8);
    EXPECT_NE(result.current.status, sct::NonlinearSeparationStatus::PotentialContact);
    sct::test::ExpectResult(result.current, result.original);
  }
#if defined(__SSE2__)
  struct RestoreMxcsr {
    unsigned value = _mm_getcsr();
    ~RestoreMxcsr() { _mm_setcsr(value); }
  } restore;
  auto near = first;
  for (auto& vertex : near.vertices) vertex.z = std::numeric_limits<double>::denorm_min();
  _mm_setcsr(restore.value | 0x8040u);
  const auto result = sct::CompareRigidSeparationEndpointWitness(
      first, first, zero, .01, near, near, zero, .01, 1, 64, 8);
  EXPECT_NE(result.current.status, sct::NonlinearSeparationStatus::PotentialContact);
  sct::test::ExpectResult(result.current, result.original);
#endif
}

TEST(SelfContactRigidSweepBounds, EndpointWitnessKeepsInputAndRootArithmeticErrorsAheadOfIt) {
  const auto triangle = TriangleAt({});
  for (unsigned invalid = 0; invalid < 8; ++invalid) {
    auto coefficients = Coefficients({});
    double thickness = .01, duration = 1;
    std::size_t work = 64;
    unsigned depth = 20;
    if (invalid == 0) coefficients.complete = false;
    if (invalid == 1) coefficients.q[0][0] = {1, -1};
    if (invalid == 2) coefficients.q[0][0].upper = std::numeric_limits<double>::infinity();
    if (invalid == 3) thickness = 0;
    if (invalid == 4) duration = 0;
    if (invalid == 5) work = 0;
    if (invalid == 6) depth = 53;
    if (invalid == 7) thickness = std::numeric_limits<double>::max();
    const auto result = sct::CompareRigidSeparationEndpointWitness(
        triangle, triangle, coefficients, thickness,
        triangle, triangle, coefficients, thickness, duration, work, depth);
    ASSERT_EQ(result.original.status, sct::NonlinearSeparationStatus::InvalidInput) << invalid;
    sct::test::ExpectResult(result.current, result.original);
  }
}

TEST(SelfContactRigidSweepBounds, EndpointWitnessCountsOneRealRootVisitAtMinimalLimits) {
  const auto triangle = TriangleAt({});
  for (unsigned depth : {0u, 20u}) {
    const auto result = sct::CompareRigidSeparationEndpointWitness(
        triangle, triangle, Coefficients({}), .01,
        triangle, triangle, Coefficients({}), .01, 1, 1, depth);
    EXPECT_EQ(result.original.work, 1u);
    EXPECT_EQ(result.current.status, sct::NonlinearSeparationStatus::PotentialContact);
    EXPECT_EQ(result.current.work, 1u);
    EXPECT_EQ(result.current.deepest, 0u);
    EXPECT_FALSE(result.current.has_intersection);
    EXPECT_EQ(result.current.accepted_certificate, SIZE_MAX);
  }
}
