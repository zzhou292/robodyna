// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "Oracle.h"
#include "ResultAssertions.h"
#include "lib_src/collision/represented_interval_crossing/ExactPathReuseQualification.h"
#include "lib_src/collision/represented_interval_crossing/Batch.h"

namespace represented_interval_test {
namespace reuse = ct::represented_interval_crossing;
using ReusePaths = std::array<ct::RepresentedTrianglePath, 2>;

reuse::ExactPathReuseComparison CompareReuse(const ReusePaths& paths,
                                            ct::RepresentedIntervalLimits limits = {}) {
  const auto result = reuse::CompareExactPathReuse(paths[0], paths[1], limits);
  EXPECT_EQ(result.status, S::Ok);
  SameResult(result.current.result, result.original.result);
  EXPECT_FALSE(result.original.reused.saturated || result.current.reused.saturated);
  EXPECT_EQ(result.original.reused.single_sample_intervals, 0u);
  EXPECT_EQ(result.original.reused.dominated_edge_loops, 0u);
  EXPECT_EQ(result.current.exact.evaluated_cells, result.original.exact.evaluated_cells);
  EXPECT_EQ(result.current.reused.vertex_face_tests, result.original.reused.vertex_face_tests);
  EXPECT_LE(result.current.reused.edge_edge_tests, result.original.reused.edge_edge_tests);
  return result;
}

TEST(RepresentedExactPathReuse, ExactCommonTranslationKeepsContactSeparationAndDegeneracy) {
  const auto first = Static(10, BaseTriangle());
  const std::array<ct::Vec3, 3> transverse{{{0, 0, 0}, {-2, 0, 1}, {0, -2, -1}}};
  const std::array<ct::Vec3, 3> collapsed{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  for (const auto& second : {Static(20, BaseTriangle()), Static(20, BaseTriangle(1)),
                            Static(20, transverse), Static(20, collapsed)}) {
    const auto compared = CompareReuse({first, second});
    ASSERT_TRUE(compared.domain.eligible);
    EXPECT_EQ(compared.current.reused.single_sample_intervals, 1u);
    EXPECT_EQ(compared.current.result.work, 1u);
    EXPECT_LE(compared.current.exact.normal_evaluations, 2u);
    auto owner = Owner(); SameResult(One(owner, {first, second}), compared.original.result);
  }
  auto a = BaseTriangle(), b = BaseTriangle(1), a_next = a, b_next = b;
  for (auto* points : {&a_next, &b_next}) for (auto& point : *points) {
    point.x += 4; point.y -= 3; point.z += 2;
  }
  const auto translated = CompareReuse({Path(10, a, a_next), Path(20, b, b_next)});
  EXPECT_EQ(translated.current.reused.single_sample_intervals, 1u);
  EXPECT_EQ(translated.current.result.classification, C::CertifiedSeparated);
}

TEST(RepresentedExactPathReuse, AllSixVertexFaceTestsPrecedeDominatedEdgeSuppression) {
  const auto a = Static(10, BaseTriangle(), 100);
  const auto b = Permute(Static(20, BaseTriangle(), 1), {1, 2, 0});
  const auto compared = CompareReuse({a, b});
  ASSERT_EQ(compared.current.result.feature.kind, K::VertexFace);
  EXPECT_EQ(compared.current.result.feature.vertex.first, 1u);
  EXPECT_EQ(compared.current.result.feature.face.parent_eid, 10u);
  EXPECT_EQ(compared.current.reused.vertex_face_tests, 6u);
  EXPECT_EQ(compared.original.reused.edge_edge_tests, 9u);
  EXPECT_EQ(compared.current.reused.edge_edge_tests, 0u);
  EXPECT_EQ(compared.current.reused.dominated_edge_loops, 1u);
  const auto exact = ExactOracleAt(a, b, 0, 1);
  EXPECT_TRUE(exact.valid && exact.intersects && exact.vertex_face);
}

TEST(RepresentedExactPathReuse, EdgeOnlyWitnessStillRunsEveryEdgeFeaturePredicate) {
  const std::array<ct::Vec3, 3> a{{{0, 2, 0}, {-2, -1, 0}, {2, -1, 0}}};
  const std::array<ct::Vec3, 3> b{{{0, -2, 0}, {-2, 1, 0}, {2, 1, 0}}};
  const ReusePaths paths{Static(10, a), Static(20, b)};
  const auto exact = ExactOracleAt(paths[0], paths[1], 0, 1);
  ASSERT_TRUE(exact.valid && exact.intersects && exact.edge_edge);
  ASSERT_FALSE(exact.vertex_face);
  const auto compared = CompareReuse(paths);
  EXPECT_EQ(compared.current.result.feature.kind, K::EdgeEdge);
  EXPECT_EQ(compared.current.reused.vertex_face_tests, 6u);
  EXPECT_EQ(compared.current.reused.edge_edge_tests, 9u);
  EXPECT_EQ(compared.current.reused.dominated_edge_loops, 0u);
}

TEST(RepresentedExactPathReuse, NonuniformMotionAndLaterDegenerateSampleKeepOriginalWitnessOrder) {
  const std::array<ct::Vec3, 3> collapsed{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  for (const auto& paths : std::array<ReusePaths, 2>{{
      {Static(10, BaseTriangle()), Path(20, BaseTriangle(1), BaseTriangle(-3))},
      {Path(10, collapsed, BaseTriangle()), Static(20, BaseTriangle())}}}) {
    const auto compared = CompareReuse(paths);
    EXPECT_EQ(compared.current.reused.single_sample_intervals, 0u);
    EXPECT_EQ(compared.current.result.classification, C::CertifiedCrossingContact);
    EXPECT_GT(compared.current.result.witness_time_depth, 0u);
    const auto& result = compared.current.result;
    EXPECT_TRUE(ExactOracleAt(paths[0], paths[1], result.witness_time_numerator,
                            std::uint64_t{1} << result.witness_time_depth).intersects);
    auto owner = Owner(); SameResult(One(owner, {paths[0], paths[1]}), compared.original.result);
  }
}

TEST(RepresentedExactPathReuse, DeepCellsWorkBoundsAndNonDyadicContactRemainExact) {
  const auto first = Static(10, BaseTriangle());
  const auto second = Path(20, BaseTriangle(1), BaseTriangle(1 - std::ldexp(1., 40)));
  for (unsigned depth : {0u, 8u, 40u, 52u}) for (std::size_t work : {1u, 64u}) {
    ct::RepresentedIntervalLimits limits; limits.max_depth = depth; limits.max_work_per_pair = work;
    const auto compared = CompareReuse({first, second}, limits);
    EXPECT_EQ(compared.current.reused.single_sample_intervals, 0u);
  }
  ct::RepresentedIntervalLimits limits; limits.max_work_per_pair = 3;
  const auto missed = CompareReuse({first, Path(20, BaseTriangle(1), BaseTriangle(-2))}, limits);
  EXPECT_EQ(missed.current.result.classification, C::Unresolved);
  EXPECT_EQ(missed.current.result.reason, R::WorkExhausted);
}

TEST(RepresentedExactPathReuse, DomainThresholdWideInputsAndUnsupportedMotionKeepLegacyTraversal) {
  ct::RepresentedIntervalLimits limits; limits.max_depth = 20;
  const auto settings = CompareReuse({Static(10, BaseTriangle()), Static(20, BaseTriangle())}, limits).domain;
  ASSERT_TRUE(settings.supported);
  const auto bound = ((settings.karatsuba_cutoff - 1) * settings.limb_bits - 3) / 2;
  const int exponent = static_cast<int>(bound) - 1076 - static_cast<int>(limits.max_depth + 1);
  for (unsigned offset : {0u, 1u}) {
    auto vertices = BaseTriangle();
    const auto scale = std::ldexp(1., exponent + int(offset));
    for (auto& point : vertices) { point.x *= scale; point.y *= scale; }
    const auto compared = CompareReuse({Static(10, vertices), Static(20, vertices)}, limits);
    EXPECT_EQ(compared.domain.coordinate_bits, bound + offset);
    EXPECT_EQ(compared.domain.eligible, offset == 0);
    EXPECT_EQ(compared.current.reused.single_sample_intervals, offset ? 0u : 1u);
    EXPECT_EQ(compared.current.reused.dominated_edge_loops, offset ? 0u : 1u);
    if (offset) {
      EXPECT_EQ(compared.current.exact.normal_evaluations, compared.original.exact.normal_evaluations);
      EXPECT_EQ(compared.current.reused.edge_edge_tests, compared.original.reused.edge_edge_tests);
    }
  }
  const auto mixed = MixedExponentPaths();
  const auto wide = CompareReuse({mixed[0], mixed[1]}, limits);
  EXPECT_FALSE(wide.domain.eligible);
  EXPECT_EQ(wide.current.reused.dominated_edge_loops, 0u);
  auto unsupported = Static(20, BaseTriangle()); unsupported.motion = ct::RepresentedMotion::RigidArc;
  const auto arc = CompareReuse({Static(10, BaseTriangle()), unsupported}, limits);
  EXPECT_EQ(arc.current.result.reason, R::UnsupportedMotion);
  EXPECT_EQ(arc.current.reused.single_sample_intervals, 0u);
}

TEST(RepresentedExactPathReuse, VertexAndPairPermutationPreserveCanonicalFeatureAfterFullVfSweep) {
  const ReusePaths paths{Static(10, BaseTriangle(), 100), Static(20, BaseTriangle(), 1)};
  const auto expected = CompareReuse(paths).original.result;
  const std::array<std::array<unsigned, 3>, 6> orders{{
      {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
  for (const auto& a : orders) for (const auto& b : orders) {
    const auto compared = CompareReuse({Permute(paths[1], b), Permute(paths[0], a)});
    SameResult(compared.current.result, expected);
    EXPECT_EQ(compared.current.reused.vertex_face_tests, 6u);
  }
}

ct::RepresentedIntervalReport ReportFromOriginal(const std::vector<ct::RepresentedIntervalResult>& rows,
                                                std::size_t paths) {
  ct::RepresentedIntervalReport result;
  result.input_paths = paths; result.input_pairs = result.unique_pairs = rows.size();
  for (const auto& row : rows) {
    result.work += row.work;
    result.certified_separated += row.classification == C::CertifiedSeparated;
    result.certified_crossing_contact += row.classification == C::CertifiedCrossingContact;
    result.unresolved += row.classification == C::Unresolved;
  }
  return result;
}

TEST(RepresentedExactPathReuse, PersistentWorkersResetScratchAndPreserveCompleteReportsAcrossCalls) {
  const std::array<ct::Vec3, 3> collapsed{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  const std::vector<ct::RepresentedTrianglePath> paths{
      Static(10, BaseTriangle()), Static(20, BaseTriangle()), Static(30, BaseTriangle(1)),
      Path(40, BaseTriangle(1), BaseTriangle(-3)), Static(50, collapsed)};
  const std::vector<ct::RepresentedTrianglePair> pairs{{0, 1}, {0, 2}, {0, 3}, {0, 4}};
  std::vector<ct::RepresentedIntervalResult> expected;
  for (std::size_t i = 1; i < paths.size(); ++i)
    expected.push_back(CompareReuse({paths[0], paths[i]}).original.result);
  for (unsigned workers : {1u, 4u}) {
    ct::RepresentedIntervalLimits limits; limits.worker_count = workers;
    auto owner = Owner(limits);
    for (unsigned repeat = 0; repeat < 3; ++repeat) {
      auto order = pairs; if (repeat & 1) std::reverse(order.begin(), order.end());
      const auto report = owner.Certify(paths.data(), paths.size(), order.data(), order.size());
      SameNativeReport(report, ReportFromOriginal(expected, paths.size()));
      ASSERT_EQ(owner.results().count, expected.size());
      for (std::size_t i = 0; i < expected.size(); ++i) SameResult(owner.results().data[i], expected[i]);
    }
  }
}

TEST(RepresentedExactPathReuse, WorkAdmissionFailureRetainsPublicationAndOriginalReportThenRetries) {
  const std::vector<ct::RepresentedTrianglePath> paths{Static(10, BaseTriangle()),
      Static(20, BaseTriangle(1)), Path(30, BaseTriangle(1), BaseTriangle(-3))};
  const ct::RepresentedTrianglePair pairs[]{{0, 1}, {0, 2}};
  const auto first = CompareReuse({paths[0], paths[1]}).original.result;
  const auto second = CompareReuse({paths[0], paths[2]}).original.result;
  for (unsigned workers : {1u, 4u}) {
    ct::RepresentedIntervalLimits limits; limits.worker_count = workers; limits.max_total_work = 1;
    auto owner = Owner(limits); SameResult(One(owner, {paths[0], paths[1]}), first);
    const auto before = owner.results();
    const auto failed = owner.Certify(paths.data(), paths.size(), pairs, 2);
    auto expected = ReportFromOriginal({first}, paths.size());
    expected.status = S::ResourceLimit; expected.message = "resource limit";
    expected.input_pairs = expected.unique_pairs = 2; expected.input_pair = 1;
    expected.total_work_limit = 1; expected.rejected_pair_work = second.work;
    SameNativeReport(failed, expected);
    ASSERT_EQ(owner.results().data, before.data); ASSERT_EQ(owner.results().count, 1u);
    SameResult(owner.results().data[0], first);
    SameResult(One(owner, {paths[0], paths[1]}), first);
  }
}

TEST(RepresentedExactPathReuse, InvalidIdentityAndLimitsCannotAcquireFastPathAdmission) {
  const auto a = Static(10, BaseTriangle());
  auto b = Static(20, BaseTriangle(1)); b.vertices[0].endpoint[1].z = NAN;
  EXPECT_EQ(reuse::CompareExactPathReuse(a, b, {}).status, S::InvalidInput);
  ct::RepresentedIntervalLimits limits; limits.max_depth = 53;
  EXPECT_EQ(reuse::CompareExactPathReuse(a, Static(20, BaseTriangle()), limits).status, S::InvalidInput);
  limits.max_depth = 0; limits.max_work_per_pair = 0;
  EXPECT_EQ(reuse::CompareExactPathReuse(a, Static(20, BaseTriangle()), limits).status, S::InvalidInput);
}
}  // namespace represented_interval_test
