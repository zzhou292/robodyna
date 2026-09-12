// SPDX-License-Identifier: MIT
#include "ExactOracle.h"
#include <gtest/gtest.h>
#include <cfloat>
#include <cstring>
#include <limits>
#include <map>
#include <random>

namespace majorant_test {
using S = ct::SurfaceMajorantStatus;
using Packet = ct::SurfaceJacobianMajorant;
TEST(SurfaceJacobianMajorant, SourceOrderMergeProjectionAndEightNodeSupport) {
  const ct::RepresentedJacobianTerm terms[]{{7, {0x1p53, 4, -0.0}, 2}, {3, {2, -3, 4}, 0},
      {7, {1, 2, -0.0}, 2}, {7, {-0x1p53, 0, -0.0}, 2}, {4, {1, 2, 3}, 7}};
  Packet value;
  ASSERT_EQ(ct::BuildRepresentedJacobianMajorant(terms, 5, 8, 2, &value), S::Ok);
  ASSERT_EQ(value.count, 3u);
  EXPECT_EQ(value.nodes[0].node, 3u);
  EXPECT_EQ(value.nodes[1].node, 4u);
  EXPECT_EQ(value.nodes[2].node, 7u);
  EXPECT_EQ(value.nodes[2].jacobian.x, 0); // RN source-order result, not exact-real 1.
  EXPECT_EQ(value.nodes[2].jacobian.y, 0); // Project after merging.
  EXPECT_TRUE(std::signbit(value.nodes[2].jacobian.z));
  EXPECT_FALSE(std::signbit(value.nodes[1].jacobian.z));
  EXPECT_EQ(value.nodes[1].norm_upper, 0);
  EXPECT_EQ(value.nodes[2].diagonal_n_m, 0);
  EXPECT_TRUE(ExactBounds(value));
  ct::RepresentedJacobianTerm eight[8];
  for (unsigned i = 0; i < 8; ++i) eight[i] = {7 - i, {i & 1 ? -.25 : .25, .5, -.125}, static_cast<std::uint8_t>(i)};
  ASSERT_EQ(ct::BuildRepresentedJacobianMajorant(eight, 8, 8, 1000, &value), S::Ok);
  EXPECT_EQ(value.count, 8u);
  EXPECT_TRUE(ExactBounds(value));
  for (unsigned i = 0; i < 8; ++i) EXPECT_EQ(value.nodes[i].node, i);
}
TEST(SurfaceJacobianMajorant, ExactDyadicBoundsAndQuadraticFormsAcrossSignedSupports) {
  std::mt19937_64 generator(732913);
  for (unsigned fixture = 0; fixture < 128; ++fixture) {
    SCOPED_TRACE(fixture);
    ct::RepresentedJacobianTerm terms[8];
    std::map<unsigned, ct::Vec3> expected;
    for (unsigned i = 0; i < 8; ++i) {
      const auto node = static_cast<unsigned>(generator() % 6);
      const auto sample = [&]() { return std::ldexp(static_cast<int>(generator() % 31) - 15,
          static_cast<int>(generator() % 35) - 17); };
      terms[i] = {node, {sample(), sample(), sample()}, static_cast<std::uint8_t>(node % 8)};
      auto found = expected.find(node);
      if (found == expected.end()) expected[node] = terms[i].value;
      else {
        found->second.x += terms[i].value.x;
        found->second.y += terms[i].value.y;
        found->second.z += terms[i].value.z;
      }
    }
    Packet value;
    ASSERT_EQ(ct::BuildRepresentedJacobianMajorant(terms, 8, 6,
        std::ldexp(3.0, static_cast<int>(fixture % 40) - 20), &value), S::Ok);
    ASSERT_TRUE(ExactBounds(value));
    unsigned index = 0;
    for (const auto& pair : expected) {
      const auto& actual = value.nodes[index++];
      const auto bits = pair.first % 8;
      EXPECT_EQ(actual.node, pair.first);
      EXPECT_EQ(actual.jacobian.x, bits & 1 ? 0 : pair.second.x);
      EXPECT_EQ(actual.jacobian.y, bits & 2 ? 0 : pair.second.y);
      EXPECT_EQ(actual.jacobian.z, bits & 4 ? 0 : pair.second.z);
    }
    for (unsigned probe = 0; probe < 6; ++probe) {
      ct::Vec3 velocity[8];
      for (unsigned i = 0; i < value.count; ++i) velocity[i] = {
          static_cast<double>(static_cast<int>(generator() % 41) - 20),
          static_cast<double>(static_cast<int>(generator() % 41) - 20),
          static_cast<double>(static_cast<int>(generator() % 41) - 20)};
      EXPECT_TRUE(ExactQuadratic(value, velocity));
    }
  }
}
TEST(SurfaceJacobianMajorant, NextafterNormalCounterexampleAndLegacyScalarOrder) {
  const auto normal = std::nextafter(1.0, 2.0);
  ASSERT_TRUE(OldWeightBoundFails(normal));
  const ct::SignedNodeWeight weights[]{{1, -.5}, {0, .25}, {0, .25}};
  const std::uint8_t fixed[]{0, 0};
  Packet value;
  ASSERT_EQ(ct::BuildSignedNormalMajorant(weights, 3, fixed, 2, {normal, 0, 0}, 1, &value), S::Ok);
  EXPECT_EQ(value.nodes[0].jacobian.x, .5 * normal);
  EXPECT_EQ(value.nodes[1].jacobian.x, -.5 * normal);
  ASSERT_TRUE(ExactBounds(value));
  const ct::Vec3 worst[]{{1, 0, 0}, {-1, 0, 0}};
  EXPECT_TRUE(ExactQuadratic(value, worst));
  auto broken = value;
  broken.nodes[0].diagonal_n_m = .5;
  broken.nodes[1].diagonal_n_m = .5;
  EXPECT_FALSE(ExactQuadratic(broken, worst));
  const double inverse[]{1, 1};
  ct::NormalJacobian legacy;
  ASSERT_EQ(ct::BuildNormalJacobian({inverse, fixed, 2, 0, ct::TranslationMassModel::kIsotropicLumped},
      weights, 3, {normal, 0, 0}, 1, &legacy), ct::Status::kOk);
  EXPECT_EQ(legacy.count, value.count);
  for (unsigned i = 0; i < legacy.count; ++i) {
    EXPECT_EQ(legacy.nodes[i], value.nodes[i].node);
    EXPECT_EQ(legacy.values[i].x, value.nodes[i].jacobian.x);
  }
  static_assert(ct::kMaxNormalNodes == 6, "Legacy support layout remains unchanged");
}
TEST(SurfaceJacobianMajorant, ZeroSubnormalOverflowAndPositiveUnderflowPolicies) {
  Packet value;
  ct::RepresentedJacobianTerm terms[]{{0, {-0.0, 0, -0.0}, 0}, {1, {1, 2, 3}, 7}};
  ASSERT_EQ(ct::BuildRepresentedJacobianMajorant(terms, 2, 2, -0.0, &value), S::Ok);
  EXPECT_TRUE(ExactBounds(value));
  EXPECT_EQ(value.norm_sum_upper, 0);
  EXPECT_FALSE(std::signbit(value.nodes[0].diagonal_n_m));
  terms[0].value = {1e-160, 0, 0};
  ASSERT_EQ(ct::BuildRepresentedJacobianMajorant(terms, 1, 2, 1, &value), S::Ok);
  EXPECT_GT(value.nodes[0].diagonal_n_m, 0);
  EXPECT_LT(value.nodes[0].diagonal_n_m, DBL_MIN);
  EXPECT_TRUE(ExactBounds(value));
  terms[0].value = {std::numeric_limits<double>::denorm_min(), 0, 0};
  ASSERT_EQ(ct::BuildRepresentedJacobianMajorant(terms, 1, 2, 0, &value), S::Ok);
  EXPECT_TRUE(ExactBounds(value));
  const Packet previous = value;
  const auto unchanged = [&]() { EXPECT_EQ(std::memcmp(&previous, &value, sizeof(value)), 0); };
  EXPECT_EQ(ct::BuildRepresentedJacobianMajorant(terms, 1, 2, 1, &value), S::Unrepresentable);
  unchanged();
  terms[0].value = {1, std::numeric_limits<double>::denorm_min(), 0};
  EXPECT_EQ(ct::BuildRepresentedJacobianMajorant(terms, 1, 2, 1, &value), S::Unrepresentable);
  unchanged();
  terms[0].value = {DBL_MAX, 0, 0};
  EXPECT_EQ(ct::BuildRepresentedJacobianMajorant(terms, 1, 2, 1, &value), S::Unrepresentable);
  unchanged();
  terms[0].value = {1, 0, 0};
  EXPECT_EQ(ct::BuildRepresentedJacobianMajorant(terms, 1, 2, DBL_MAX, &value), S::Unrepresentable);
  unchanged();
  const ct::SignedNodeWeight weight{0, .5};
  const std::uint8_t free = 0;
  EXPECT_EQ(ct::BuildSignedNormalMajorant(&weight, 1, &free, 1,
      {1, std::numeric_limits<double>::denorm_min(), 0}, 1, &value), S::Unrepresentable);
  unchanged();
}
TEST(SurfaceJacobianMajorant, LateInvalidInputsAndMaskDisagreementPreserveOutput) {
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const ct::RepresentedJacobianTerm good[]{{3, {1, 2, 3}, 1}, {0, {4, 5, 6}, 2}, {3, {7, 8, 9}, 1}};
  Packet value;
  ASSERT_EQ(ct::BuildRepresentedJacobianMajorant(good, 3, 4, 1, &value), S::Ok);
  const Packet previous = value;
  for (unsigned fault = 0; fault < 4; ++fault) {
    ct::RepresentedJacobianTerm terms[3];
    std::copy(good, good + 3, terms);
    if (fault == 0) terms[2].translation_fixed_bits = 2;
    if (fault == 1) terms[2].translation_fixed_bits = 8;
    if (fault == 2) terms[2].node = 4;
    if (fault == 3) terms[2].value.x = nan;
    EXPECT_NE(ct::BuildRepresentedJacobianMajorant(terms, 3, 4, 1, &value), S::Ok);
    EXPECT_EQ(std::memcmp(&previous, &value, sizeof(value)), 0);
  }
  EXPECT_EQ(ct::BuildRepresentedJacobianMajorant(good, 3, 4, -1, &value), S::InvalidInput);
  EXPECT_EQ(ct::BuildRepresentedJacobianMajorant(good, 3, 4, nan, &value), S::InvalidInput);
  EXPECT_EQ(ct::BuildRepresentedJacobianMajorant(good, 0, 4, 1, &value), S::InvalidInput);
  EXPECT_EQ(ct::BuildRepresentedJacobianMajorant(reinterpret_cast<const ct::RepresentedJacobianTerm*>(1),
      9, 4, 1, &value), S::OutOfRange);
  EXPECT_EQ(std::memcmp(&previous, &value, sizeof(value)), 0);
  ASSERT_EQ(ct::BuildRepresentedJacobianMajorant(good, 3, 4, 1, &value), S::Ok);
}
} // namespace majorant_test
