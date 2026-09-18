// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Value-level composition tests use actual native geometry publications.
// Physical assembly/activity/regularity authority is exercised separately by
// the CUDA transaction coupons; this helper cannot manufacture that authority.
std::vector<c::RepresentedIntervalResult> NativeLocalResults(
    const c::CurrentFixedTriangle& first,
    const c::CurrentFixedTriangle& first_next,
    const c::CurrentFixedTriangle& second,
    const c::CurrentFixedTriangle& second_next,
    c::RepresentedMotion second_motion = c::RepresentedMotion::LinearNodalV1,
    c::RepresentedIntervalStatus expected_status = c::RepresentedIntervalStatus::Ok,
    c::RepresentedMotion first_motion = c::RepresentedMotion::LinearNodalV1) {
  c::RepresentedTrianglePath paths[2];
  const c::CurrentFixedTriangle* base[]{&first, &second};
  const c::CurrentFixedTriangle* next[]{&first_next, &second_next};
  for (unsigned row = 0; row < 2; ++row) {
    auto& path = paths[row];
    const auto& key = base[row]->key;
    path.key = {key.source_instance_id, key.parent_eid, key.level, key.local_facet};
    path.motion = row ? second_motion : first_motion;
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      path.vertices[vertex].key = base[row]->vertex_keys[vertex];
      path.vertices[vertex].endpoint[0] = base[row]->vertices[vertex];
      path.vertices[vertex].endpoint[1] = next[row]->vertices[vertex];
      path.edge_keys[vertex] = base[row]->edge_keys[vertex];
    }
  }
  c::RepresentedIntervalLimits limits;
  limits.max_paths = 2;
  limits.max_input_pairs = 1;
  limits.max_results = 1;
  limits.max_work_per_pair = 1;
  limits.max_total_work = 1;
  c::RepresentedIntervalCrossing native;
  EXPECT_EQ(native.Initialize(limits).status, c::RepresentedIntervalStatus::Ok);
  const c::RepresentedTrianglePair pair{0, 1};
  const auto report = native.Certify(paths, 2, &pair, 1);
  EXPECT_EQ(report.status, expected_status);
  const auto results = native.results();
  if (expected_status != c::RepresentedIntervalStatus::Ok) {
    EXPECT_FALSE(results.complete);
    EXPECT_EQ(results.count, 0u);
    return {};
  }
  EXPECT_TRUE(results.complete);
  EXPECT_EQ(results.count, 1u);
  EXPECT_NE(results.data, nullptr);
  if (report.status != c::RepresentedIntervalStatus::Ok ||
      !results.complete || results.count != 1 || !results.data)
    return {};
  return {results.data[0]};
}

void CheckTranslatedLocalUnchanged(
    const c::RepresentedIntervalResult& before,
    const c::RepresentedIntervalResult& after) {
  EXPECT_EQ(sct::Compare(before.key, after.key), 0);
  EXPECT_EQ(before.classification, after.classification);
  EXPECT_EQ(before.reason, after.reason);
  EXPECT_EQ(before.geometry, after.geometry);
  EXPECT_EQ(before.feature.kind, after.feature.kind);
  EXPECT_EQ(c::fixed_triangle_features::Compare(before.feature.vertex, after.feature.vertex), 0);
  EXPECT_EQ(sct::Compare(before.feature.face, after.feature.face), 0);
  for (unsigned edge = 0; edge < 2; ++edge)
    EXPECT_EQ(c::fixed_triangle_features::Compare(before.feature.edges[edge], after.feature.edges[edge]), 0);
  EXPECT_EQ(before.witness_time_numerator, after.witness_time_numerator);
  EXPECT_EQ(before.witness_time_depth, after.witness_time_depth);
  EXPECT_EQ(before.accepted_event, after.accepted_event);
  EXPECT_EQ(before.work, after.work);
}

TEST(SelfContactTransactionValues,
     NativeStaticAndTranslatedLocalProofPreservesWorkAndPublishesLocalGeometry) {
  const auto first = Triangle(10, {1, 2, 3}, {{{0,0,0}, {2,0,0}, {0,2,0}}});
  for (const auto second : {
      Triangle(20, {1,2,4}, {{{0,0,0}, {2,0,0}, {0,-2,0}}}),
      Triangle(20, {1,4,5}, {{{0,0,0}, {-2,-1,1}, {-2,-1,-1}}})}) {
    for (const double shift : {0., 4.}) {
      auto first_next = first, second_next = second;
      for (auto* triangle : {&first_next, &second_next})
        for (auto& point : triangle->vertices) point.z += shift;
      const auto geometry = DiscoverPreparedPair(first_next, second_next);
      ASSERT_EQ(geometry.intersection_count, 1u);
      ASSERT_FALSE(c::RequiresIntersectionAdmission(geometry.intersections[0]));
      const c::FixedTriangleIntersectionView intersections{
          geometry.intersections.data(), geometry.intersection_count, true};
      const auto native = NativeLocalResults(first, first_next, second, second_next);
      ASSERT_EQ(native.size(), 1u);
      auto value = native.front();
      ASSERT_TRUE(c::HasExactCommonTranslationProof(value.geometry));
      const auto native_work = value.work;
      ASSERT_EQ(sct::NormalizeExactTranslatedLocal(intersections, &value),
                sct::TranslatedLocalStatus::Certified);
      EXPECT_EQ(value.geometry, c::RepresentedIntersectionGeometry::CertifiedLocalTopology);
      EXPECT_EQ(value.feature.kind, c::RepresentedFeatureKind::TriangleIntersection);
      EXPECT_EQ(value.accepted_event, SIZE_MAX);
      EXPECT_EQ(value.witness_time_numerator, 0u);
      EXPECT_EQ(value.witness_time_depth, 0u);
      EXPECT_EQ(value.work, native_work);
      c::SelfContactCandidatePolicyOutcome outcome;
      std::size_t count = 9;
      auto input = Input(&value.key, 1, &value, &outcome, &count);
      input.intersections = intersections;
      ASSERT_EQ(sct::ValidateCandidatePublications(input).status,
                c::SelfContactTransactionStatus::Ok);
      ASSERT_EQ(count, 1u);
      EXPECT_EQ(outcome.disposition, c::SelfContactCandidateDisposition::ExcludedLocalIntersection);
      EXPECT_EQ(outcome.accepted_event, SIZE_MAX);
      EXPECT_EQ(outcome.source_order, UINT64_MAX);
    }
  }
}

TEST(SelfContactTransactionValues,
     TranslationProofDoesNotWeakenStandaloneUnownedThicknessContract) {
  const auto first = Triangle(10, {1,2,3}, {{{0,0,0}, {2,0,0}, {0,2,0}}});
  const auto second = Triangle(20, {1,4,5}, {{{0,0,0}, {.5,.5,.125}, {.5,1,.125}}});
  const auto geometry = DiscoverPreparedPair(first, second);
  ASSERT_EQ(geometry.intersection_count, 1u);
  ASSERT_EQ(geometry.intersections[0].local_exclusion,
            c::FixedTriangleLocalExclusion::SharedVertexOnly);
  // A real nonincident vertex-face task lies strictly inside summed physical
  // thickness. The geometry-only normalizer never claims to own its force.
  bool near_unmasked_face = false;
  for (std::size_t index = 0; index < geometry.count; ++index) {
    const auto& feature = geometry.values[index];
    if (feature.key.kind == c::FixedTriangleCandidateKind::VertexFace &&
        feature.key.vertex_face.vertex.first == 4 &&
        feature.distance_m > 0 && feature.distance_m < .25)
      near_unmasked_face = true;
  }
  ASSERT_TRUE(near_unmasked_face);
  const auto standalone = sct::CertifyQuadraticLocalContact(
      first, first, Quadratic(0), .125,
      second, second, Quadratic(0), .125, 1, 127, 8);
  EXPECT_EQ(standalone.status, sct::NonlinearSeparationStatus::PotentialContact);
  const auto native = NativeLocalResults(first, first, second, second);
  ASSERT_EQ(native.size(), 1u);
  auto value = native.front();
  ASSERT_TRUE(c::HasExactCommonTranslationProof(value.geometry));
  EXPECT_EQ(sct::NormalizeExactTranslatedLocal(
      {geometry.intersections.data(),1,true}, &value),
      sct::TranslatedLocalStatus::Certified);
  EXPECT_EQ(value.accepted_event, SIZE_MAX);
  EXPECT_EQ(value.work, 1u);
}

TEST(SelfContactTransactionValues,
     TranslatedNonlocalGeometryCannotBorrowLocalPublication) {
  const auto first = Triangle(10, {1,2,3}, {{{0,0,0}, {2,0,0}, {0,2,0}}});
  for (const auto second : {
      Triangle(20, {4,5,6}, {{{.25,.25,0}, {.75,.25,0}, {.25,.75,0}}}),
      Triangle(20, {4,5,6}, {{{.5,.25,-1}, {.5,1.25,-1}, {.5,.75,1}}})}) {
    const auto geometry = DiscoverPreparedPair(first, second);
    ASSERT_EQ(geometry.intersection_count, 1u);
    ASSERT_TRUE(c::RequiresIntersectionAdmission(geometry.intersections[0]));
    const auto native = NativeLocalResults(first, first, second, second);
    ASSERT_EQ(native.size(), 1u);
    auto value = native.front();
    ASSERT_TRUE(c::HasExactCommonTranslationProof(value.geometry));
    const auto before = value;
    const c::FixedTriangleIntersectionView intersections{geometry.intersections.data(), 1, true};
    EXPECT_EQ(sct::NormalizeExactTranslatedLocal(intersections, &value),
              sct::TranslatedLocalStatus::NotApplicable);
    CheckTranslatedLocalUnchanged(before, value);
    c::SelfContactCandidatePolicyOutcome outcome;
    std::size_t count = 9;
    auto input = Input(&value.key, 1, &value, &outcome, &count);
    input.intersections = intersections;
    EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
              c::SelfContactTransactionStatus::CandidateRejected);
    EXPECT_EQ(count, 9u);  // Failed publication preserves the caller's count.
  }
}

TEST(SelfContactTransactionValues,
     LocalNormalizationCannotPromoteOrdinaryWitnessOrUnsupportedMotion) {
  const auto first = Triangle(10, {1,2,3}, {{{0,0,0}, {2,0,0}, {0,2,0}}});
  const auto second = Triangle(20, {1,2,4}, {{{0,0,0}, {2,0,0}, {0,-2,0}}});
  const auto geometry = DiscoverPreparedPair(first, second);
  ASSERT_EQ(geometry.intersection_count, 1u);
  const c::FixedTriangleIntersectionView intersections{geometry.intersections.data(), 1, true};
  const auto native = NativeLocalResults(first, first, second, second);
  ASSERT_EQ(native.size(), 1u);
  auto value = native.front();
  value.geometry = c::BaseIntersectionGeometry(value.geometry);
  const auto before = value;
  EXPECT_EQ(sct::NormalizeExactTranslatedLocal(intersections, &value),
            sct::TranslatedLocalStatus::NotApplicable);
  CheckTranslatedLocalUnchanged(before, value);
  c::SelfContactCandidatePolicyOutcome outcome;
  std::size_t count = 9;
  auto input = Input(&value.key, 1, &value, &outcome, &count);
  input.intersections = intersections;
  EXPECT_EQ(sct::ValidateCandidatePublications(input).status,
            c::SelfContactTransactionStatus::CandidateRejected);
  EXPECT_EQ(count, 9u);  // No successful publication replaced the caller's count.

  // A shared source vertex cannot simultaneously declare a linear trajectory
  // in one facet and rigid/nonlinear motion in another. The native identity
  // check rejects the entire call before any result exists to normalize.
  for (const auto motion : {c::RepresentedMotion::RigidArc,
                            c::RepresentedMotion::Nonlinear}) {
    const auto rejected = NativeLocalResults(first, first, second, second,
        motion, c::RepresentedIntervalStatus::IdentityMismatch);
    EXPECT_TRUE(rejected.empty());

    // Consistent source declarations are a distinct negative case: native
    // publication succeeds with an explicit unsupported-motion result. Equal
    // endpoints do not give curved paths an exact affine-translation proof.
    const auto consistent = NativeLocalResults(first, first, second, second,
        motion, c::RepresentedIntervalStatus::Ok, motion);
    ASSERT_EQ(consistent.size(), 1u);
    auto unsupported = consistent.front();
    EXPECT_EQ(unsupported.classification, c::RepresentedIntervalClassification::Unresolved);
    EXPECT_EQ(unsupported.reason, c::RepresentedIntervalReason::UnsupportedMotion);
    EXPECT_EQ(unsupported.work, 0u);
    EXPECT_FALSE(c::HasExactCommonTranslationProof(unsupported.geometry));
    EXPECT_EQ(sct::NormalizeExactTranslatedLocal(intersections, &unsupported),
              sct::TranslatedLocalStatus::NotApplicable);
    CheckTranslatedLocalUnchanged(consistent.front(), unsupported);
    auto unsupported_input = Input(&unsupported.key, 1, &unsupported, &outcome, &count);
    unsupported_input.intersections = intersections;
    EXPECT_EQ(sct::ValidateCandidatePublications(unsupported_input).status,
              c::SelfContactTransactionStatus::UnresolvedCandidate);
    EXPECT_EQ(count, 9u);
  }
}

TEST(SelfContactTransactionValues,
     LocalNormalizationRequiresCompleteMatchingNativeInputsWithoutPartialWrite) {
  const auto first = Triangle(10, {1,2,3}, {{{0,0,0}, {2,0,0}, {0,2,0}}});
  const auto second = Triangle(20, {1,2,4}, {{{0,0,0}, {2,0,0}, {0,-2,0}}});
  const auto geometry = DiscoverPreparedPair(first, second);
  ASSERT_EQ(geometry.intersection_count, 1u);
  const auto native = NativeLocalResults(first, first, second, second);
  ASSERT_EQ(native.size(), 1u);
  const auto original = native.front();
  for (unsigned mode = 0; mode < 10; ++mode) {
    auto value = original;
    auto intersection = geometry.intersections[0];
    c::FixedTriangleIntersectionView intersections{&intersection, 1, true};
    switch (mode) {
      case 0: intersections.complete = false; break;
      case 1: intersections.data = nullptr; break;
      case 2: std::swap(value.key.paths[0], value.key.paths[1]); break;
      case 3: value.classification = c::RepresentedIntervalClassification::Unresolved; break;
      case 4: value.reason = c::RepresentedIntervalReason::UnsupportedMotion; break;
      case 5: value.witness_time_numerator = 1; break;
      case 6: value.witness_time_depth = 1; break;
      case 7: value.work = 0; break;
      case 8: value.work = 2; break;
      case 9: intersection.local_exclusion = static_cast<c::FixedTriangleLocalExclusion>(255); break;
    }
    const auto before = value;
    EXPECT_EQ(sct::NormalizeExactTranslatedLocal(intersections, &value),
              sct::TranslatedLocalStatus::InvalidInput) << mode;
    CheckTranslatedLocalUnchanged(before, value);
  }
  auto value = original;
  auto foreign = geometry.intersections[0];
  ++foreign.triangles[0].source_instance_id;
  EXPECT_EQ(sct::NormalizeExactTranslatedLocal({&foreign,1,true}, &value),
            sct::TranslatedLocalStatus::NotApplicable);
  CheckTranslatedLocalUnchanged(original, value);
  EXPECT_EQ(sct::NormalizeExactTranslatedLocal({nullptr,0,true}, &value),
            sct::TranslatedLocalStatus::NotApplicable);
  CheckTranslatedLocalUnchanged(original, value);
  EXPECT_EQ(sct::NormalizeExactTranslatedLocal({nullptr,0,true}, nullptr),
            sct::TranslatedLocalStatus::InvalidInput);
  // Lookup preserves producer permutation and the historical first match.
  auto reversed = geometry.intersections[0];
  std::swap(reversed.triangles[0], reversed.triangles[1]);
  EXPECT_EQ(sct::FindPairIntersection({&reversed,1,true}, original.key), &reversed);
}
