// SPDX-License-Identifier: MIT
#include "Oracle.h"

namespace represented_interval_test {

TEST(RepresentedIntervalCrossing,
     DeepDyadicAffineContactUsesOwnedIterativeStack) {
  ct::RepresentedIntervalLimits limits;
  limits.max_depth = 52;
  limits.max_work_per_pair = 64;
  auto owner = Owner(limits);
  const auto a = Static(10, BaseTriangle());
  const double slope = std::ldexp(1.0, 40);
  const auto b = Path(20, BaseTriangle(1),
                      BaseTriangle(1 - slope));
  const auto result = One(owner, {a, b});
  ASSERT_EQ(result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.witness_time_numerator, 1u);
  EXPECT_EQ(result.witness_time_depth, 40u);
  EXPECT_LE(result.work, 41u);
  const auto oracle = ExactOracleAt(
      a, b, result.witness_time_numerator,
      std::uint64_t{1} << result.witness_time_depth);
  EXPECT_TRUE(oracle.valid);
  EXPECT_TRUE(oracle.intersects);
}

TEST(RepresentedIntervalCrossing,
     StaticDisjointTouchingBoxesUseExactGeometryCertificate) {
  ct::RepresentedIntervalLimits limits;
  limits.max_depth = 52;
  limits.max_work_per_pair = 53;
  limits.max_total_work = 53;
  auto owner = Owner(limits);
  EXPECT_EQ(owner.forecast().dfs_frame_capacity, 53u);
  EXPECT_GT(owner.forecast().dfs_frame_bytes, 0u);
  EXPECT_GT(owner.forecast().exact_scratch_bytes, 0u);
  const std::array<ct::Vec3, 3> diagonal{
      ct::Vec3{1.5, 1.5, 0}, {3.5, 1.5, 0}, {1.5, 3.5, 0}};
  const auto a = Static(10, BaseTriangle());
  const auto b = Static(20, diagonal);
  const auto result = One(owner, {a, b});
  EXPECT_EQ(result.classification, C::CertifiedSeparated);
  EXPECT_EQ(result.reason, R::None);
  EXPECT_EQ(result.work, 1u);
  for (std::uint64_t numerator = 0; numerator <= 8; ++numerator)
    EXPECT_FALSE(ExactOracleAt(a, b, numerator, 8).intersects);
}

TEST(RepresentedIntervalCrossing,
     ExactCommonTranslationCertifiesSeparatedYarisGeometryInOneVisit) {
  ct::RepresentedIntervalLimits limits;
  limits.max_work_per_pair = 1;
  limits.max_total_work = 1;
  limits.max_depth = 20;
  auto owner = Owner(limits);
  const std::array<ct::Vec3, 3> first_base{{
      {-1.2143384000000002, -0.63109113000000006,
       0.84671514999999997},
      {-1.2142823, -0.63140399000000003,
       0.83110510000000004},
      {-1.2148208999999999, -0.65323883000000005,
       0.83184398999999998}}};
  const std::array<ct::Vec3, 3> first_current{{
      {-1.2143352707200001, -0.63109113000000006,
       0.84671514999999997},
      {-1.21427917072, -0.63140399000000003,
       0.83110510000000004},
      {-1.2148177707199999, -0.65323883000000005,
       0.83184398999999998}}};
  const std::array<ct::Vec3, 3> second_base{{
      {-1.2191921000000001, -0.62582568000000005,
       0.82527410999999995},
      {-1.2171582000000001, -0.63322559,
       0.83204822000000001},
      {-1.2125827999999998, -0.63369568000000009,
       0.82216882000000002}}};
  const std::array<ct::Vec3, 3> second_current{{
      {-1.2191889707200001, -0.62582568000000005,
       0.82527410999999995},
      {-1.2171550707200001, -0.63322559,
       0.83204822000000001},
      {-1.2125796707199998, -0.63369568000000009,
       0.82216882000000002}}};
  const auto a = Path(10, first_base, first_current);
  const auto b = Path(20, second_base, second_current);
  const auto result = One(owner, {a, b});
  EXPECT_EQ(result.classification, C::CertifiedSeparated);
  EXPECT_EQ(result.reason, R::None);
  EXPECT_EQ(result.work, 1u);
  for (const auto& time :
       {std::pair<std::uint64_t, std::uint64_t>{0, 1},
        {1, 2}, {1, 1}}) {
    const auto oracle =
        ExactOracleAt(a, b, time.first, time.second);
    ASSERT_TRUE(oracle.valid);
    EXPECT_FALSE(oracle.intersects);
  }
}

TEST(RepresentedIntervalCrossing,
     ExactCommonTranslationKeepsActualContactAndFeatureRepresented) {
  ct::RepresentedIntervalLimits limits;
  limits.max_work_per_pair = 1;
  limits.max_total_work = 1;
  auto owner = Owner(limits);
  auto first = BaseTriangle();
  auto second = BaseTriangle();
  for (auto& vertex : second)
    vertex.x += .25;
  auto first_current = first;
  auto second_current = second;
  for (auto* triangle : {&first_current, &second_current})
    for (auto& vertex : *triangle) {
      vertex.x += 4;
      vertex.y -= 3;
      vertex.z += 2;
    }
  const auto result = One(
      owner, {Path(10, first, first_current, 100),
              Path(20, second, second_current, 200)});
  EXPECT_EQ(result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.reason, R::None);
  EXPECT_NE(result.feature.kind, K::None);
  EXPECT_EQ(result.witness_time_numerator, 0u);
  EXPECT_EQ(result.witness_time_depth, 0u);
  EXPECT_EQ(result.work, 1u);
}

TEST(RepresentedIntervalCrossing,
     NondyadicIsolatedContactRemainsUnresolvedNeverSeparated) {
  ct::RepresentedIntervalLimits limits;
  limits.max_depth = 12;
  limits.max_work_per_pair = 256;
  auto owner = Owner(limits);
  const auto a = Static(10, BaseTriangle());
  const auto b = Path(20, BaseTriangle(1), BaseTriangle(-2));
  const auto result = One(owner, {a, b});
  EXPECT_EQ(result.classification, C::Unresolved);
  EXPECT_EQ(result.reason, R::WorkExhausted);
  const auto exact_third = ExactOracleAt(a, b, 1, 3);
  ASSERT_TRUE(exact_third.valid);
  EXPECT_TRUE(exact_third.intersects);
  for (const auto& time : {std::pair<std::uint64_t, std::uint64_t>{0, 1},
                           {1, 4}, {1, 2}, {1, 1}})
    EXPECT_FALSE(ExactOracleAt(a, b, time.first, time.second).intersects);
}

TEST(RepresentedIntervalCrossing,
     ExtremeBinary64ExponentsInterpolateWithoutFalseRangeResult) {
  ct::RepresentedIntervalLimits limits;
  limits.max_depth = 52;
  auto owner = Owner(limits);
  const auto paths = MixedExponentPaths();
  const auto& a = paths[0];
  const auto& b = paths[1];
  const auto result = One(owner, {a, b});
  EXPECT_EQ(result.classification, C::CertifiedSeparated);
  EXPECT_EQ(result.reason, R::None);
  for (std::uint64_t numerator = 0; numerator <= 4; ++numerator) {
    const auto oracle = ExactOracleAt(a, b, numerator, 4);
    EXPECT_TRUE(oracle.valid);
    EXPECT_FALSE(oracle.intersects);
  }
}

}  // namespace represented_interval_test
