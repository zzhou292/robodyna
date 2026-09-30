// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "ResultAssertions.h"
#include "lib_src/collision/represented_interval_crossing/RelativeSeparationQualification.h"

namespace represented_interval_test {
namespace q = ct::represented_interval_crossing;
using Paths = std::array<ct::RepresentedTrianglePath, 2>;

Paths RelativeSweep(double scale = 1, std::uint64_t eid = 10) {
  const std::array<ct::Vec3, 3> a{{{0, 0, 0}, {0, scale, 0}, {0, 0, scale}}};
  auto next = a, b = a, b_next = a;
  for (unsigned i = 0; i < 3; ++i) {
    next[i].x = scale; b[i].x = scale / 8;
    b_next[i].x = scale + scale / 8 + scale / 1024;
  }
  return {Path(eid, a, next), Path(eid + 1, b, b_next)};
}
Paths ObliqueFaces(std::uint64_t eid = 30) {
  const std::array<ct::Vec3, 3> a{{{0, 0, 0}, {2, 0, 2}, {0, 2, 2}}};
  auto b = a, next = a;
  for (unsigned i = 0; i < 3; ++i) { b[i].z += .125; next[i].z += .25; }
  return {Static(eid, a), Path(eid + 1, b, next)};
}
q::RelativeSeparationComparison CompareSeparation(const Paths& paths,
                                                 ct::RepresentedIntervalLimits limits = {}) {
  const auto result = q::CompareRelativeSeparation(paths[0], paths[1], limits);
  EXPECT_EQ(result.status, S::Ok);
  EXPECT_FALSE(result.counters.saturated);
  return result;
}

TEST(RepresentedRelativeSeparation, RelativeAabbRemovesCommonDriftWithoutAssumingCommonTranslation) {
  const auto paths = RelativeSweep();
  const auto result = CompareSeparation(paths);
  ASSERT_TRUE(result.domain.eligible);
  ASSERT_EQ(result.legacy.classification, C::CertifiedSeparated);
  EXPECT_GT(result.legacy.work, 1u);
  EXPECT_EQ(result.current.classification, C::CertifiedSeparated);
  EXPECT_EQ(result.current.geometry, G::None);
  EXPECT_EQ(result.current.work, 1u);
  EXPECT_EQ(result.counters.relative_aabb_separated, 1u);
  EXPECT_EQ(result.counters.first_face_separated, 0u);
  auto owner = Owner(); SameResult(One(owner, {paths[0], paths[1]}), result.current);
}

TEST(RepresentedRelativeSeparation, LowerFaceAxisProvesAnObliqueGapAtOneVisit) {
  const auto paths = ObliqueFaces();
  ct::RepresentedIntervalLimits limits; limits.max_work_per_pair = 7; limits.max_depth = 8;
  const auto result = CompareSeparation(paths, limits);
  ASSERT_TRUE(result.domain.eligible);
  EXPECT_EQ(result.legacy.classification, C::Unresolved);
  EXPECT_EQ(result.legacy.reason, R::WorkExhausted);
  EXPECT_EQ(result.legacy.work, 7u);
  EXPECT_EQ(result.current.classification, C::CertifiedSeparated);
  EXPECT_EQ(result.current.work, 1u);
  EXPECT_EQ(result.counters.relative_aabb_separated, 0u);
  EXPECT_EQ(result.counters.first_face_separated, 1u);
  auto owner = Owner(limits); SameResult(One(owner, {paths[0], paths[1]}), result.current);
}

TEST(RepresentedRelativeSeparation, SecondFaceAxisCertifiesWhenFirstFaceAndCoordinatesCannot) {
  const std::array<ct::Vec3, 3> tilted{{{3, 0, -1}, {0, 3, 1}, {3, 0, 1}}};
  auto next = tilted; for (auto& point : next) point.x += .125;
  const Paths paths{Static(10, BaseTriangle()), Path(20, tilted, next)};
  ct::RepresentedIntervalLimits limits; limits.max_work_per_pair = 7; limits.max_depth = 8;
  const auto result = CompareSeparation(paths, limits);
  // A spans z=0 and x+y<=2. B straddles z=0, but its fixed plane is
  // x+y=3+.125*t: only B's face normal among this slice's axes separates.
  EXPECT_EQ(result.legacy.classification, C::Unresolved);
  EXPECT_EQ(result.legacy.work, 7u);
  EXPECT_EQ(result.current.classification, C::CertifiedSeparated);
  EXPECT_EQ(result.current.work, 1u);
  EXPECT_EQ(result.counters.relative_aabb_separated, 0u);
  EXPECT_EQ(result.counters.first_face_separated, 0u);
  EXPECT_EQ(result.counters.second_face_separated, 1u);
  auto owner = Owner(limits); SameResult(One(owner, {paths[0], paths[1]}), result.current);
}

TEST(RepresentedRelativeSeparation, FaceGapIsStrictAtTouchAndNextafterOnBothSides) {
  const std::array<ct::Vec3, 3> a{{{0, 0, 0}, {1, 0, 1}, {0, 1, 0}}};
  for (int sign : {-1, 0, 1}) {
    auto b = a, next = a;
    for (unsigned i = 0; i < 3; ++i) {
      if (sign) b[i].z = std::nextafter(a[i].z, sign > 0 ? INFINITY : -INFINITY);
      next[i].z = std::nextafter(b[i].z, sign >= 0 ? INFINITY : -INFINITY);
    }
    ct::RepresentedIntervalLimits limits; limits.max_work_per_pair = 3; limits.max_depth = 8;
    const auto result = CompareSeparation({Static(10, a), Path(20, b, next)}, limits);
    if (!sign) {
      EXPECT_EQ(result.current.classification, C::CertifiedCrossingContact);
      EXPECT_EQ(result.current.witness_time_numerator, 0u);
      SameResult(result.current, result.legacy);
      EXPECT_EQ(result.counters.eligible_cells, 0u);
    } else {
      EXPECT_EQ(result.current.classification, C::CertifiedSeparated);
      EXPECT_EQ(result.current.work, 1u);
      EXPECT_EQ(result.counters.first_face_separated, 1u);
    }
  }
}

TEST(RepresentedRelativeSeparation, TrueNonDyadicContactAndExcludedCoplanarAxesKeepLegacyBudgetBehavior) {
  ct::RepresentedIntervalLimits limits; limits.max_work_per_pair = 3; limits.max_depth = 20;
  const Paths collision{Static(10, BaseTriangle()), Path(20, BaseTriangle(1), BaseTriangle(-2))};
  const auto true_contact = CompareSeparation(collision, limits);
  // z=1-3*t: triangles coincide at exact rational t=1/3, so no complete
  // interval can have a strict gap, although dyadic samples miss the contact.
  EXPECT_EQ(true_contact.current.classification, C::Unresolved);
  EXPECT_EQ(true_contact.current.reason, R::WorkExhausted);
  EXPECT_EQ(true_contact.current.work, 3u);
  SameResult(true_contact.current, true_contact.legacy);
  const std::array<ct::Vec3, 3> diagonal{{{1.5, 1.5, 0}, {3.5, 1.5, 0}, {1.5, 3.5, 0}}};
  auto shifted = diagonal; for (auto& point : shifted) point.x += .125;
  const auto absent_axis = CompareSeparation({Static(10, BaseTriangle()), Path(20, diagonal, shifted)}, limits);
  EXPECT_EQ(absent_axis.current.classification, C::Unresolved);
  SameResult(absent_axis.current, absent_axis.legacy);
}

TEST(RepresentedRelativeSeparation, SampledAndInteriorDegeneracyCannotBeBypassedBySeparation) {
  const std::array<ct::Vec3, 3> initial{{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}};
  for (unsigned mutation = 0; mutation < 3; ++mutation) {
    auto first = initial, next = initial;
    if (mutation == 0) { first[2] = {2, 0, 0}; next = first; }
    if (mutation == 1) { next[1].x = -1; next[2].y = -1; }
    if (mutation == 2) next[1].x = -2;
    ct::RepresentedIntervalLimits limits; limits.max_work_per_pair = 31; limits.max_depth = 8;
    const auto result = CompareSeparation({Path(10, first, next), Static(20, BaseTriangle(2))}, limits);
    EXPECT_EQ(result.current.classification, C::Unresolved);
    SameResult(result.current, result.legacy);
    EXPECT_EQ(result.counters.eligible_cells, 0u);
  }
}

TEST(RepresentedRelativeSeparation, CanonicalAnchorAndFaceOrderIgnoreWindingAndCallerPairOrder) {
  const std::array<ct::Vec3, 3> a{{{0, 0, 0}, {1, 1, 0}, {0, 0, 1}}};
  auto next = a; for (auto& point : next) point.x = 10;
  const std::array<ct::Vec3, 3> b{{{2, 0, 0}, {2, 1, 0}, {2, 0, 1}}};
  auto b_next = b; for (auto& point : b_next) point.x = 10.5;
  const Paths deforming{Path(10, a, next), Path(20, b, b_next)};
  const std::array<std::array<unsigned, 3>, 6> orders{{
      {{0,1,2}}, {{0,2,1}}, {{1,0,2}}, {{1,2,0}}, {{2,0,1}}, {{2,1,0}}}};
  for (const auto& paths : {deforming, ObliqueFaces()}) {
    const auto reference = CompareSeparation(paths);
    EXPECT_EQ(reference.current.classification, C::CertifiedSeparated);
    if (paths[0].key.parent_eid == 10) EXPECT_GT(reference.current.work, 1u);
    for (const auto& first_order : orders)
      for (const auto& second_order : orders) {
        const auto compared = CompareSeparation({Permute(paths[1], second_order), Permute(paths[0], first_order)});
        SameResult(compared.current, reference.current);
      }
  }
}

TEST(RepresentedRelativeSeparation, ExactArithmeticDomainBoundaryKeepsWideFallbackUnchanged) {
  ct::RepresentedIntervalLimits limits; limits.max_depth = 20; limits.max_work_per_pair = 63;
  const auto settings = CompareSeparation(RelativeSweep(), limits).domain;
  ASSERT_TRUE(settings.supported);
  const auto largest_bits = ((settings.karatsuba_cutoff - 1) * settings.limb_bits - 3) / 2;
  const int power = static_cast<int>(largest_bits) - 1075 - static_cast<int>(limits.max_depth + 1);
  const auto narrow = CompareSeparation(RelativeSweep(std::ldexp(1.0, power)), limits);
  ASSERT_EQ(narrow.domain.coordinate_bits, largest_bits);
  ASSERT_TRUE(narrow.domain.eligible);
  EXPECT_EQ(narrow.current.work, 1u);
  const auto wide = CompareSeparation(RelativeSweep(std::ldexp(1.0, power + 1)), limits);
  EXPECT_EQ(wide.domain.coordinate_bits, largest_bits + 1);
  EXPECT_FALSE(wide.domain.eligible);
  EXPECT_GT(wide.counters.domain_fallback_cells, 0u);
  EXPECT_EQ(wide.counters.eligible_cells, 0u);
  SameResult(wide.current, wide.legacy);
  auto deeper = limits; ++deeper.max_depth;
  EXPECT_FALSE(CompareSeparation(RelativeSweep(std::ldexp(1.0, power)), deeper).domain.eligible);
}

TEST(RepresentedRelativeSeparation, CommonTranslationUnsupportedMotionAndMalformedInputsRemainUnchanged) {
  const Paths stationary{Static(10, BaseTriangle()), Static(20, BaseTriangle(1))};
  const auto result = CompareSeparation(stationary);
  SameResult(result.current, result.legacy); EXPECT_EQ(result.counters.eligible_cells, 0u);
  auto unsupported = stationary; unsupported[1].motion = ct::RepresentedMotion::RigidArc;
  const auto arc = CompareSeparation(unsupported); SameResult(arc.current, arc.legacy);
  EXPECT_EQ(arc.current.reason, R::UnsupportedMotion);
  auto bad = stationary; bad[1].vertices[0].endpoint[1].z = NAN;
  EXPECT_EQ(q::CompareRelativeSeparation(bad[0], bad[1], {}).status, S::InvalidInput);
  auto limits = ct::RepresentedIntervalLimits{}; limits.max_depth = 53;
  EXPECT_EQ(q::CompareRelativeSeparation(stationary[0], stationary[1], limits).status, S::InvalidInput);
}

TEST(RepresentedRelativeSeparation, WorkerCountsAndTotalWorkFailurePreservePublicationAndRetry) {
  const auto first = RelativeSweep(1, 10); const auto second = ObliqueFaces(30);
  const std::vector<ct::RepresentedTrianglePath> paths{first[0], first[1], second[0], second[1]};
  const ct::RepresentedTrianglePair pairs[]{{0,1},{2,3}};
  for (unsigned workers : {1u, 4u}) {
    ct::RepresentedIntervalLimits limits; limits.worker_count = workers;
    limits.max_work_per_pair = 3; limits.max_total_work = 1;
    auto owner = Owner(limits);
    const auto prior_result = One(owner, {Static(80, BaseTriangle()), Static(90, BaseTriangle(1))});
    const auto prior = owner.results();
    const auto denied = owner.Certify(paths.data(), paths.size(), pairs, 2);
    EXPECT_EQ(denied.status, S::ResourceLimit); EXPECT_EQ(denied.input_pair, 1u);
    EXPECT_EQ(denied.work, 1u); EXPECT_EQ(denied.total_work_limit, 1u);
    EXPECT_EQ(denied.rejected_pair_work, 1u);
    ASSERT_EQ(owner.results().data, prior.data); ASSERT_EQ(owner.results().count, 1u);
    SameResult(owner.results().data[0], prior_result);
    const auto retried = One(owner, {second[0], second[1]});
    EXPECT_EQ(retried.classification, C::CertifiedSeparated); EXPECT_EQ(retried.work, 1u);
    SameResult(retried, CompareSeparation(second, limits).current);
  }
}
}  // namespace represented_interval_test
