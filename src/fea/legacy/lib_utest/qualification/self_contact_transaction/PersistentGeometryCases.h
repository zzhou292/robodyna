// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Included after ValueTest.cpp's native discovery and accepted-owner helpers.
// Persistent thickness coverage is an owner/distance statement. These cases
// require the full policy's independent continuous midsurface safety proof.

TEST(SelfContactPersistentGeometry,
     AffineThicknessPersistenceDoesNotAuthorizeMidsurfaceCrossing) {
  constexpr double gap = 1.0 / 32;
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto second_at = [](double z) {
    return Triangle(
        20, {4, 5, 6}, {{{0, 0, z}, {2, 0, z}, {0, 2, z}}});
  };
  const auto accepted_second = second_at(gap);
  const auto prepared_second = second_at(-gap);
  const auto base_geometry = DiscoverPreparedPair(first, accepted_second);
  const auto next_geometry = DiscoverPreparedPair(first, prepared_second);
  ASSERT_EQ(base_geometry.intersection_count, 0u);
  ASSERT_EQ(next_geometry.intersection_count, 0u);
  const auto owner = AcceptedCertificate(FirstEdgeEdge(base_geometry));
  const auto persistent = sct::CertifyPersistentLinearContact(
      first, first, .1, accepted_second, prepared_second, .1,
      {next_geometry.values.data(), next_geometry.count, true}, &owner, 1);
  ASSERT_EQ(persistent.status,
            sct::PersistentLinearContactStatus::CertifiedContact);
  ASSERT_GT(persistent.strict_thickness_margin_lower_m, 0);

  // The dyadic midpoint is a real nonlocal coplanar intersection. Source
  // vertices are distinct, so coordinate coincidence grants no local mask.
  const auto midpoint_geometry = DiscoverPreparedPair(first, second_at(0));
  ASSERT_EQ(midpoint_geometry.intersection_count, 1u);
  ASSERT_TRUE(c::RequiresIntersectionAdmission(midpoint_geometry.intersections[0]));
  const auto policy = sct::CertifyQuadraticFacetPolicyCoverage(
      first, first, Quadratic(0), .1,
      accepted_second, prepared_second, Quadratic(0), .1, 1,
      &owner, 1, nullptr, 0, 255, 8);
  EXPECT_EQ(policy.status,
            sct::NonlinearSeparationStatus::PossibleGeometricCrossing);
  EXPECT_TRUE(policy.depth_exhausted);
  EXPECT_TRUE(policy.has_unresolved_cell);
}

TEST(SelfContactPersistentGeometry,
     QuadraticThicknessPersistenceDoesNotAuthorizeInteriorExcursion) {
  constexpr double gap = 3.0 / 128;
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto second_at = [](double z) {
    return Triangle(
        20, {4, 5, 6}, {{{0, 0, z}, {2, 0, z}, {0, 2, z}}});
  };
  const auto second = second_at(gap);
  const auto geometry = DiscoverPreparedPair(first, second);
  ASSERT_EQ(geometry.intersection_count, 0u);
  const auto owner = AcceptedCertificate(FirstEdgeEdge(geometry));
  const auto curved = Quadratic(.25);
  const auto persistent = sct::CertifyPersistentQuadraticContact(
      first, first, Quadratic(0), .1,
      second, second, curved, .1, 1,
      {geometry.values.data(), geometry.count, true}, &owner, 1);
  ASSERT_EQ(persistent.status,
            sct::PersistentLinearContactStatus::CertifiedContact);
  ASSERT_GT(persistent.strict_thickness_margin_lower_m, 0);

  // z(u)=gap-q*u*(1-u)/2 has exact roots at 1/4 and 3/4, and
  // z(1/2)=-1/128. The final endpoint returns to its accepted position.
  constexpr double quarter_z = gap - .25 * .25 * .75 / 2;
  static_assert(quarter_z == 0);
  static_assert(gap - .25 / 8 < 0);
  const auto quarter = DiscoverPreparedPair(first, second_at(quarter_z));
  ASSERT_EQ(quarter.intersection_count, 1u);
  ASSERT_TRUE(c::RequiresIntersectionAdmission(quarter.intersections[0]));
  const auto policy = sct::CertifyQuadraticFacetPolicyCoverage(
      first, first, Quadratic(0), .1, second, second, curved, .1, 1,
      &owner, 1, nullptr, 0, 255, 8);
  EXPECT_EQ(policy.status,
            sct::NonlinearSeparationStatus::PossibleGeometricCrossing);
  EXPECT_TRUE(policy.depth_exhausted);
  EXPECT_TRUE(policy.has_unresolved_cell);
}

TEST(SelfContactPersistentGeometry,
     LegitimateMovingAcceptedContactRetainsContinuousPolicyCoverage) {
  constexpr double gap = 3.0 / 128;
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto accepted_second = Triangle(
      20, {4, 5, 6}, {{{0, 0, gap}, {2, 0, gap}, {0, 2, gap}}});
  auto prepared_second = accepted_second;
  for (auto& vertex : prepared_second.vertices) vertex.z += 1.0 / 64;
  const auto base_geometry = DiscoverPreparedPair(first, accepted_second);
  const auto next_geometry = DiscoverPreparedPair(first, prepared_second);
  const auto owner = AcceptedCertificate(FirstEdgeEdge(base_geometry));
  for (const double q : {0.0, 1.0 / 16}) {
    SCOPED_TRACE(q);
    const auto curved = Quadratic(q);
    const auto persistent = sct::CertifyPersistentQuadraticContact(
        first, first, Quadratic(0), .1,
        accepted_second, prepared_second, curved, .1, 1,
        {next_geometry.values.data(), next_geometry.count, true}, &owner, 1);
    ASSERT_EQ(persistent.status,
              sct::PersistentLinearContactStatus::CertifiedContact);
    const auto policy = sct::CertifyQuadraticFacetPolicyCoverage(
        first, first, Quadratic(0), .1,
        accepted_second, prepared_second, curved, .1, 1,
        &owner, 1, nullptr, 0, 255, 8);
    EXPECT_EQ(policy.status,
              sct::NonlinearSeparationStatus::CertifiedAcceptedCoverage);
    EXPECT_EQ(policy.accepted_source_order, owner.event.source_order);
    EXPECT_GT(policy.covered_cells, 0u);
    EXPECT_FALSE(policy.depth_exhausted);
    EXPECT_FALSE(policy.work_exhausted);
  }
}

TEST(SelfContactPersistentGeometry,
     SharedVertexCannotHidePreexistingNonlocalIntersectionBehindAcceptedOwner) {
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {4, 0, 0}, {0, 4, 0}}});
  const auto second = Triangle(
      20, {1, 4, 5}, {{{0, 0, 0}, {1, 1, 1}, {1, 2, -1}}});
  const auto geometry = DiscoverPreparedPair(first, second);
  ASSERT_EQ(geometry.intersection_count, 1u);
  ASSERT_TRUE(c::RequiresIntersectionAdmission(geometry.intersections[0]));
  ASSERT_EQ(geometry.intersections[0].local_exclusion,
            c::FixedTriangleLocalExclusion::None);
  const auto found = std::find_if(
      geometry.values.begin(), geometry.values.begin() + geometry.count,
      [](const auto& feature) {
        return feature.key.kind == c::FixedTriangleCandidateKind::VertexFace &&
            feature.key.vertex_face.vertex.first == 4 &&
            feature.key.vertex_face.target.kind == c::FixedTriangleStratumKind::Face &&
            feature.key.vertex_face.target.face.parent_eid == 10;
      });
  ASSERT_NE(found, geometry.values.begin() + geometry.count);
  ASSERT_EQ(found->distance_m, 1);
  auto owner = AcceptedCertificate(*found);
  owner.event.classification.reference_half_thickness_m[0] = .75;
  owner.event.classification.reference_half_thickness_m[1] = .75;
  const auto ledger = sct::CertifyQuadraticFacetCoverage(
      first, first, Quadratic(0), .75, second, second, Quadratic(0), .75, 1,
      &owner, 1, 255, 8);
  EXPECT_EQ(ledger.status,
            sct::NonlinearSeparationStatus::PossibleGeometricCrossing);
  const auto policy = sct::CertifyQuadraticFacetPolicyCoverage(
      first, first, Quadratic(0), .75, second, second, Quadratic(0), .75, 1,
      &owner, 1, nullptr, 0, 255, 8);
  EXPECT_EQ(policy.status,
            sct::NonlinearSeparationStatus::PossibleGeometricCrossing);
  EXPECT_TRUE(ledger.depth_exhausted);
  EXPECT_TRUE(policy.depth_exhausted);
}
