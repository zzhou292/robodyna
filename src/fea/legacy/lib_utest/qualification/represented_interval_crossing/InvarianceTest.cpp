// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace represented_interval_test {

TEST(RepresentedIntervalCrossing,
     SourcePairOrderAndWindingPermutationLeaveCertificatesInvariant) {
  const auto a = Static(10, BaseTriangle(), 100);
  const auto b =
      Path(20, BaseTriangle(1), BaseTriangle(-1), 200);
  const auto c = Static(30, BaseTriangle(3), 300);

  auto first_owner = Owner();
  std::vector<ct::RepresentedTrianglePath> first_paths{a, b, c};
  const ct::RepresentedTrianglePair first_pairs[]{{2, 0}, {1, 0},
                                                  {0, 1}};
  auto report =
      first_owner.Certify(first_paths.data(), first_paths.size(),
                          first_pairs, 3);
  ASSERT_EQ(report.status, S::Ok);
  EXPECT_EQ(report.unique_pairs, 2u);
  const auto first = first_owner.results();
  ASSERT_EQ(first.count, 2u);
  EXPECT_LT(first.data[0].key.paths[0].parent_eid,
            first.data[1].key.paths[1].parent_eid);

  auto second_owner = Owner();
  std::vector<ct::RepresentedTrianglePath> second_paths{
      Permute(c, {2, 1, 0}), Permute(a, {1, 0, 2}),
      Permute(b, {2, 0, 1})};
  const ct::RepresentedTrianglePair second_pairs[]{{1, 2}, {1, 0}};
  report = second_owner.Certify(second_paths.data(), second_paths.size(),
                                second_pairs, 2);
  ASSERT_EQ(report.status, S::Ok);
  const auto second = second_owner.results();
  ASSERT_EQ(second.count, first.count);
  for (std::size_t i = 0; i < first.count; ++i) {
    EXPECT_EQ(first.data[i].key.paths[0].parent_eid,
              second.data[i].key.paths[0].parent_eid);
    EXPECT_EQ(first.data[i].key.paths[1].parent_eid,
              second.data[i].key.paths[1].parent_eid);
    EXPECT_EQ(first.data[i].classification,
              second.data[i].classification);
    EXPECT_EQ(first.data[i].reason, second.data[i].reason);
    EXPECT_EQ(first.data[i].geometry, second.data[i].geometry);
    EXPECT_EQ(first.data[i].witness_time_numerator,
              second.data[i].witness_time_numerator);
    EXPECT_EQ(first.data[i].witness_time_depth,
              second.data[i].witness_time_depth);
    EXPECT_EQ(first.data[i].work, second.data[i].work);
    SameFeature(first.data[i].feature, second.data[i].feature);
  }
}

TEST(RepresentedIntervalCrossing,
     DegenerateRepresentedTriangleIsExplicitlyUnresolved) {
  auto owner = Owner();
  const std::array<ct::Vec3, 3> line{
      ct::Vec3{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
  const auto result =
      One(owner, {Static(10, line), Static(20, BaseTriangle(2))});
  EXPECT_EQ(result.classification, C::Unresolved);
  EXPECT_EQ(result.reason, R::DegenerateGeometry);
  EXPECT_EQ(result.geometry, G::None);
}

TEST(RepresentedIntervalCrossing,
     UnsupportedRigidArcIsUnresolvedBeforeEndpointBoxReasoning) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle());
  auto arc = Static(20, BaseTriangle(2));
  arc.motion = ct::RepresentedMotion::RigidArc;
  // The identical separated endpoints do not describe the intervening arc.
  // V1 therefore performs no chord/AABB substitution.
  const auto result = One(owner, {a, arc});
  EXPECT_EQ(result.classification, C::Unresolved);
  EXPECT_EQ(result.reason, R::UnsupportedMotion);
  EXPECT_EQ(result.work, 0u);
}

TEST(RepresentedIntervalCrossing,
     UnsupportedNonlinearPathIsNotSilentlyLinearized) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle());
  auto nonlinear = Static(20, BaseTriangle(2));
  nonlinear.motion = ct::RepresentedMotion::Nonlinear;
  const auto result = One(owner, {a, nonlinear});
  EXPECT_EQ(result.classification, C::Unresolved);
  EXPECT_EQ(result.reason, R::UnsupportedMotion);
}

}  // namespace represented_interval_test
