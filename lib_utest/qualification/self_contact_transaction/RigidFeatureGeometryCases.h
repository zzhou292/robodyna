// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Include after ValueTest.cpp's native geometry/accepted-owner helpers.
// A source-derived same-rigid feature exclusion is not a whole-facet
// exclusion: the second facet below has one independently moving member.
TEST(SelfContactRigidFeatureGeometry,
     MixedFacetCannotHideInteriorCrossingBehindStationaryRigidVertexFace) {
  namespace fe = tl::fea;
  constexpr double epsilon = 1.0 / 4096;
  const auto first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {4, 0, 0}, {0, 4, 0}}});
  const auto second = Triangle(
      20, {4, 5, 6},
      {{{1, 1, epsilon}, {2, 1, epsilon}, {1, 2, epsilon}}});
  const double positions[]{
      0, 0, 0, 4, 0, 0, 0, 4, 0,
      1, 1, epsilon, 2, 1, epsilon, 1, 2, epsilon};
  const std::uint32_t groups[]{0, 0, 0, 0, 1, 0};
  std::array<fe::NodalRigidGroupSnapshot, 2> accepted_groups;
  for (unsigned group = 0; group < 2; ++group) {
    accepted_groups[group].source_kind = fe::RigidBindingSourceKind::Part;
    accepted_groups[group].source_group_id = 7 + group;
    accepted_groups[group].source_node_set_id = 17 + group;
  }
  accepted_groups[1].state.center = {2, 1, 1 + epsilon};
  auto prepared_groups = accepted_groups;
  prepared_groups[1].state.center =
      {2, 15.0 / 16, 1 + epsilon - 1.0 / 512};
  prepared_groups[1].state.omega = {1.0 / 16, 0, 0};
  // The endpoint-corrected owner recurrence exactly returns the moving
  // member to its accepted endpoint; its interior path is still curved.
  EXPECT_EQ(prepared_groups[1].state.center.y + 1.0 / 16,
            second.vertices[1].y);
  EXPECT_EQ(prepared_groups[1].state.center.z - 1 + 1.0 / 512,
            second.vertices[1].z);
  sct::FacetQuadraticCoefficients coefficients[2];
  bool affine[2]{false, true};
  for (unsigned side = 0; side < 2; ++side) {
    c::FixedContactFacet facet;
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      facet.vertices[vertex].count = 3;
      for (unsigned node = 0; node < 3; ++node)
        facet.vertices[vertex].nodes[node] = 3 * side + node;
      facet.vertices[vertex].weights[vertex] = 1;
    }
    ASSERT_EQ(sct::BuildRigidFacetQuadraticCoefficients(
        facet, {positions, 6, 3, 1}, {positions, 6, 3, 1}, groups,
        accepted_groups.data(), prepared_groups.data(), accepted_groups.size(),
        fe::NodalRigidMemberTrajectory::EndpointCorrectedSecondOrderDriftV1,
        1, &coefficients[side], &affine[side]), sct::RigidMemberSweepStatus::Ok);
  }
  EXPECT_TRUE(affine[0]);
  EXPECT_FALSE(affine[1]);
  ASSERT_EQ(coefficients[1].q[1][2].lower, 1.0 / 256);
  ASSERT_EQ(coefficients[1].q[1][2].upper, 1.0 / 256);

  const auto geometry = DiscoverPreparedPair(first, second);
  ASSERT_EQ(geometry.intersection_count, 0u);
  const auto found = std::find_if(
      geometry.values.begin(), geometry.values.begin() + geometry.count,
      [](const auto& feature) {
        return feature.key.kind == c::FixedTriangleCandidateKind::VertexFace &&
            feature.key.vertex_face.vertex.first == 4 &&
            feature.key.vertex_face.target.kind == c::FixedTriangleStratumKind::Face &&
            feature.key.vertex_face.target.face.parent_eid == 10;
      });
  ASSERT_NE(found, geometry.values.begin() + geometry.count);
  ASSERT_EQ(found->distance_m, epsilon);
  ASSERT_EQ(found->face_weights[0], .5);
  ASSERT_EQ(found->face_weights[1], .25);
  ASSERT_EQ(found->face_weights[2], .25);
  // Both weighted endpoint supports are exactly group 0. The unmapped
  // second-facet vertex 1 has group 1, so its whole-parent support is mixed.
  EXPECT_EQ(groups[3], 0u);
  for (unsigned node = 0; node < 3; ++node)
    if (found->face_weights[node] != 0) EXPECT_EQ(groups[node], groups[3]);
  EXPECT_NE(groups[4], groups[3]);
  sct::AcceptedFeatureExclusionCertificate exclusion{*found, 0};

  auto middle = second;
  middle.vertices[1].z -= coefficients[1].q[1][2].lower / 8;
  ASSERT_EQ(middle.vertices[1].z, -epsilon);
  const auto middle_geometry = DiscoverPreparedPair(first, middle);
  ASSERT_EQ(middle_geometry.intersection_count, 1u);
  ASSERT_TRUE(c::RequiresIntersectionAdmission(middle_geometry.intersections[0]));
  ASSERT_EQ(middle_geometry.intersections[0].local_exclusion,
            c::FixedTriangleLocalExclusion::None);

  // The ordinary owner path already requires geometry and rejects this
  // crossing. An otherwise valid rigid-feature exclusion must do the same.
  auto owner = AcceptedCertificate(*found);
  owner.event.classification.reference_half_thickness_m[0] = epsilon;
  owner.event.classification.reference_half_thickness_m[1] = epsilon;
  const auto ledger = sct::CertifyQuadraticFacetCoverage(
      first, first, coefficients[0], epsilon,
      second, second, coefficients[1], epsilon, 1, &owner, 1, 4095, 12);
  ASSERT_EQ(ledger.status,
            sct::NonlinearSeparationStatus::PossibleGeometricCrossing);
  const auto policy = sct::CertifyQuadraticFacetPolicyCoverage(
      first, first, coefficients[0], epsilon,
      second, second, coefficients[1], epsilon, 1,
      nullptr, 0, &exclusion, 1, 4095, 12);
  EXPECT_EQ(policy.status,
            sct::NonlinearSeparationStatus::PossibleGeometricCrossing);
  EXPECT_NE(policy.status,
            sct::NonlinearSeparationStatus::CertifiedExactExclusion);

  // The same native feature remains a valid exclusion for safe differential
  // affine motion parallel to the first facet, with positive midsurface gap.
  auto safe_prepared = second;
  safe_prepared.vertices[1].x += 1.0 / 64;
  const auto safe = sct::CertifyQuadraticFacetPolicyCoverage(
      first, first, coefficients[0], epsilon,
      second, safe_prepared, Quadratic(0), epsilon, 1,
      nullptr, 0, &exclusion, 1, 4095, 12);
  EXPECT_EQ(safe.status, sct::NonlinearSeparationStatus::CertifiedExactExclusion);
  EXPECT_EQ(safe.excluded_rigid_group, 0u);
}
