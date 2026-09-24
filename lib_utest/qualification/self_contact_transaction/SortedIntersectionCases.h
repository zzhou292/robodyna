// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

namespace sorted_intersection_test {
c::RepresentedIntervalPairKey Key(const c::FixedTriangleIntersection& value) {
  c::RepresentedIntervalPairKey key;
  for (unsigned side = 0; side < 2; ++side) {
    const auto& source = value.triangles[side];
    key.paths[side] = {source.source_instance_id, source.parent_eid, source.level, source.local_facet};
  }
  if (sct::Compare(key.paths[1], key.paths[0]) < 0) std::swap(key.paths[0], key.paths[1]);
  return key;
}
c::FixedTriangleIntersection Row(std::uint64_t id) {
  c::FixedTriangleIntersection result;
  result.triangles[0] = {17, 2 * id, 0, 0};
  result.triangles[1] = {17, 2 * id + 1, 0, 0};
  result.local_exclusion = c::FixedTriangleLocalExclusion::SharedVertexOnly;
  return result;
}
void SameOutcome(const c::SelfContactCandidatePolicyOutcome& a,
                 const c::SelfContactCandidatePolicyOutcome& b) {
  EXPECT_EQ(sct::Compare(a.pair, b.pair), 0);
  EXPECT_EQ(a.disposition, b.disposition);
  EXPECT_EQ(a.accepted_event, b.accepted_event);
  EXPECT_EQ(a.source_order, b.source_order);
}
void ComparePolicy(sct::CandidateValidationInput input, const sct::SortedIntersections& index) {
  std::vector<c::SelfContactCandidatePolicyOutcome> original(input.pair_count), indexed(input.pair_count);
  for (auto* values : {&original, &indexed}) for (auto& value : *values) value.source_order = 333;
  std::size_t first_count = 999, second_count = 999;
  input.outcomes = original.data(); input.outcome_count = &first_count;
  const auto raw = sct::ValidateCandidatePublications(input);
  input.outcomes = indexed.data(); input.outcome_count = &second_count;
  const auto fast = sct::ValidateCandidatePublications(input, index);
  sct::test::ExpectTransactionReport(fast, raw);
  EXPECT_EQ(first_count, second_count);
  for (std::size_t i = 0; i < original.size(); ++i) SameOutcome(original[i], indexed[i]);
  if (raw.status == c::SelfContactTransactionStatus::Ok) {
    c::SelfContactCandidatePolicySummary a, b;
    sct::FoldPolicyOutcomes(original.data(), first_count, &a);
    sct::FoldPolicyOutcomes(indexed.data(), second_count, &b);
    EXPECT_EQ(a.digest, b.digest); EXPECT_EQ(a.outcomes, b.outcomes);
    EXPECT_EQ(a.excluded_local_intersection, b.excluded_local_intersection);
  }
}
}  // namespace sorted_intersection_test

TEST(SelfContactSortedIntersections, OrderedCohortPreservesOrdinalsAndAvoidsRepeatedFullScans) {
  std::vector<c::FixedTriangleIntersection> rows;
  for (unsigned i = 1; i <= 4096; ++i) rows.push_back(sorted_intersection_test::Row(i));
  const c::FixedTriangleIntersectionView view{rows.data(), rows.size(), true};
  const auto index = sct::SortedIntersectionsTestAccess::FromRaw(view);
  ASSERT_TRUE(index.ordered());
  for (std::size_t ordinal : {std::size_t{0}, std::size_t{2000}, rows.size() - 1}) {
    const auto compared = sct::CompareIntersectionLookup(view, sorted_intersection_test::Key(rows[ordinal]), index);
    EXPECT_EQ(compared.original, ordinal); EXPECT_EQ(compared.indexed, ordinal);
    EXPECT_EQ(compared.original_counts.rows, ordinal + 1);
    EXPECT_LE(compared.indexed_counts.rows, 14u);
    EXPECT_FALSE(compared.original_counts.saturated || compared.indexed_counts.saturated);
    if (ordinal > 100) EXPECT_LT(compared.indexed_counts.rows, compared.original_counts.rows);
  }
  const auto absent = sorted_intersection_test::Key(sorted_intersection_test::Row(8192));
  const auto compared = sct::CompareIntersectionLookup(view, absent, index);
  EXPECT_EQ(compared.original, SIZE_MAX); EXPECT_EQ(compared.indexed, SIZE_MAX);
  EXPECT_EQ(compared.original_counts.rows, rows.size());
  EXPECT_LE(compared.indexed_counts.rows, 13u);
}

TEST(SelfContactSortedIntersections, AllSourceKeyFieldsRemainPartOfTheExactLookup) {
  auto row = sorted_intersection_test::Row(20);
  row.triangles[0].level = row.triangles[1].level = 2;
  row.triangles[0].local_facet = 3; row.triangles[1].local_facet = 4;
  const c::FixedTriangleIntersectionView view{&row, 1, true};
  const auto index = sct::SortedIntersectionsTestAccess::FromRaw(view);
  for (unsigned side = 0; side < 2; ++side) for (unsigned field = 0; field < 4; ++field) {
    auto key = sorted_intersection_test::Key(row);
    if (field == 0) ++key.paths[side].source_instance_id;
    if (field == 1) key.paths[side].parent_eid += 100;
    if (field == 2) ++key.paths[side].level;
    if (field == 3) ++key.paths[side].local_facet;
    const auto compared = sct::CompareIntersectionLookup(view, key, index);
    EXPECT_EQ(compared.original, SIZE_MAX); EXPECT_EQ(compared.indexed, SIZE_MAX);
  }
}

TEST(SelfContactSortedIntersections, UnsortedReversedAndDuplicateRowsRetainRawFirstMatch) {
  const auto target = sorted_intersection_test::Row(2);
  for (unsigned mutation = 0; mutation < 4; ++mutation) {
    std::array<c::FixedTriangleIntersection, 3> rows{
        sorted_intersection_test::Row(1), target, sorted_intersection_test::Row(3)};
    if (mutation == 0) std::swap(rows[0], rows[2]);
    if (mutation == 1) std::swap(rows[1].triangles[0], rows[1].triangles[1]);
    if (mutation == 2) { rows[2] = rows[1]; rows[1].local_exclusion = c::FixedTriangleLocalExclusion::None; }
    if (mutation == 3) rows[0].triangles[1] = rows[0].triangles[0];
    const c::FixedTriangleIntersectionView view{rows.data(), rows.size(), true};
    const auto index = sct::SortedIntersectionsTestAccess::FromRaw(view);
    EXPECT_FALSE(index.ordered());
    const auto compared = sct::CompareIntersectionLookup(view, sorted_intersection_test::Key(target), index);
    EXPECT_EQ(compared.original, 1u); EXPECT_EQ(compared.indexed, compared.original);
    EXPECT_EQ(compared.indexed_counts.rows, compared.original_counts.rows);
  }
}

TEST(SelfContactSortedIntersections, EmptyIncompleteAndForeignViewsCannotBorrowSortedAuthority) {
  const auto empty = sct::SortedIntersectionsTestAccess::FromRaw({nullptr, 0, true});
  const auto row = sorted_intersection_test::Row(7);
  const auto key = sorted_intersection_test::Key(row);
  for (const auto view : {c::FixedTriangleIntersectionView{nullptr, 0, true},
                          c::FixedTriangleIntersectionView{nullptr, 1, true},
                          c::FixedTriangleIntersectionView{&row, 1, false}}) {
    const auto index = sct::SortedIntersectionsTestAccess::FromRaw(view);
    const auto compared = sct::CompareIntersectionLookup(view, key, index);
    EXPECT_EQ(compared.original, compared.indexed);
    EXPECT_EQ(index.ordered(), view.complete && !view.count);
  }
  const c::FixedTriangleIntersectionView other{&row, 1, true};
  const auto foreign = sct::CompareIntersectionLookup(other, key, empty);
  EXPECT_FALSE(foreign.ordered); EXPECT_EQ(foreign.indexed, 0u);
  EXPECT_EQ(foreign.indexed_counts.rows, foreign.original_counts.rows);
}

TEST(SelfContactSortedIntersections, NativeCohortPreservesNormalizationAndCompletePolicyResults) {
  const auto first = Triangle(10, {1, 2, 3}, {{{0,0,0}, {2,0,0}, {0,2,0}}});
  for (const auto second : {
      Triangle(20, {1,2,4}, {{{0,0,0}, {2,0,0}, {0,-2,0}}}),
      Triangle(20, {1,4,5}, {{{0,0,0}, {-2,-1,1}, {-2,-1,-1}}}),
      Triangle(20, {4,5,6}, {{{.25,.25,0}, {.75,.25,0}, {.25,.75,0}}})}) {
    const c::CurrentFixedTriangle triangles[]{first, second};
    const c::FixedTrianglePair pair{0, 1};
    c::FixedTriangleFeatureDiscovery publisher;
    ASSERT_EQ(publisher.Initialize().status, c::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_EQ(publisher.Discover(triangles, 2, &pair, 1).status, c::FixedTriangleDiscoveryStatus::Ok);
    const auto view = publisher.intersections();
    ASSERT_EQ(view.count, 1u);
    const auto index = sct::SortedIntersectionsTestAccess::FromPublisher(publisher);
    ASSERT_TRUE(index.matches(view));
    const auto native = NativeLocalResults(first, first, second, second);
    ASSERT_EQ(native.size(), 1u);
    auto raw = native.front(), fast = raw;
    EXPECT_EQ(sct::NormalizeExactTranslatedLocal(view, &raw),
              sct::NormalizeExactTranslatedLocal(view, &fast, index));
    CheckTranslatedLocalUnchanged(raw, fast);
    c::SelfContactCandidatePolicyOutcome outcome;
    std::size_t count = 99;
    auto input = Input(&raw.key, 1, &raw, &outcome, &count);
    input.features = publisher.features(); input.intersections = view;
    sorted_intersection_test::ComparePolicy(input, index);
  }
}

TEST(SelfContactSortedIntersections, MalformedCrossingPriorityAndFailedOutputRetentionStayIdentical) {
  const auto first = Triangle(10, {1,2,3}, {{{0,0,0}, {2,0,0}, {0,2,0}}});
  const auto second = Triangle(20, {1,2,4}, {{{0,0,0}, {2,0,0}, {0,-2,0}}});
  const auto geometry = DiscoverPreparedPair(first, second);
  ASSERT_EQ(geometry.intersection_count, 1u);
  const c::FixedTriangleIntersectionView view{geometry.intersections.data(), 1, true};
  const auto index = sct::SortedIntersectionsTestAccess::FromRaw(view);
  const auto native = NativeLocalResults(first, first, second, second);
  ASSERT_EQ(native.size(), 1u);
  auto local = native.front();
  ASSERT_EQ(sct::NormalizeExactTranslatedLocal(view, &local), sct::TranslatedLocalStatus::Certified);
  for (unsigned mutation = 0; mutation < 10; ++mutation) {
    auto crossing = local;
    if (mutation == 0) crossing.classification = c::RepresentedIntervalClassification::Unresolved;
    if (mutation == 1) crossing.reason = c::RepresentedIntervalReason::WorkExhausted;
    if (mutation == 2) crossing.feature.kind = c::RepresentedFeatureKind::VertexFace;
    if (mutation == 3) crossing.accepted_event = 0;
    if (mutation == 4) crossing.witness_time_numerator = 1;
    if (mutation == 5) crossing.witness_time_depth = 1;
    if (mutation == 6) ++crossing.key.paths[1].parent_eid;
    c::SelfContactCandidatePolicyOutcome outcome;
    std::size_t count = 99;
    auto input = Input(&local.key, 1, &crossing, &outcome, &count);
    input.intersections = view;
    if (mutation == 7) input.intersections.complete = false;
    if (mutation == 8) input.crossings.complete = false;
    if (mutation == 9) input.outcome_capacity = 0;
    sorted_intersection_test::ComparePolicy(input, index);
  }
}

TEST(SelfContactSortedIntersections, BorrowCannotEscapeByCopyMoveOrPublicConstruction) {
  static_assert(!std::is_copy_constructible_v<sct::SortedIntersections>);
  static_assert(!std::is_move_constructible_v<sct::SortedIntersections>);
  static_assert(!std::is_constructible_v<sct::SortedIntersections, c::FixedTriangleIntersectionView>);
  static_assert(!std::is_constructible_v<sct::SortedIntersections, const c::FixedTriangleFeatureDiscovery&>);
  const auto row = sorted_intersection_test::Row(1);
  const auto index = sct::SortedIntersectionsTestAccess::FromRaw({&row, 1, true});
  auto other = row;
  EXPECT_FALSE(index.matches({&other, 1, true}));
  EXPECT_FALSE(index.matches({&row, 0, true}));
  EXPECT_FALSE(index.matches({&row, 1, false}));
}
