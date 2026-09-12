#include "TestSupport.h"
#include "../surface_jacobian_majorant/ExactOracle.h"
#include "lib_src/collision/weighted_surface/Adapters.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace pair_test {
namespace {
using S = ct::SurfacePenaltyStatus;
void Near(ct::Vec3 a, ct::Vec3 b, double tolerance = 1e-12) {
  EXPECT_NEAR(a.x, b.x, tolerance);
  EXPECT_NEAR(a.y, b.y, tolerance);
  EXPECT_NEAR(a.z, b.z, tolerance);
}
TEST(SurfacePenaltyPair, AnalyticEqualOppositeMomentWorkAndExactMajorant) {
  Case value;
  ct::SurfacePenaltyPacket result;
  ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(value.input(), &result), S::Ok);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.active);
  EXPECT_DOUBLE_EQ(result.normal_force_n, 8);
  EXPECT_DOUBLE_EQ(result.elastic_energy_j, 1);
  Near(result.force_a_n, {0, 0, 8}, 0);
  Near(result.force_b_n, {0, 0, -8}, 0);
  ct::Vec3 total, moment;
  double work = 0;
  ct::Vec3 velocity[8];
  for (unsigned i = 0; i < result.count; ++i) {
    const auto& node = result.nodes[i];
    total = ct::Add(total, node.force_n);
    moment = ct::Add(moment, ct::geometry_detail::Cross(value.input().positions.at(node.node), node.force_n));
    velocity[i] = value.input().velocities.at(node.node);
    work += ct::Dot(node.force_n, velocity[i]);
  }
  Near(total, {}, 0);
  Near(moment, {}, 0);
  EXPECT_DOUBLE_EQ(work, result.normal_force_n * result.normal_velocity_m_s);
  EXPECT_TRUE(majorant_test::ExactBounds(result.normal_majorant));
  EXPECT_TRUE(majorant_test::ExactQuadratic(result.normal_majorant, velocity));
}
TEST(SurfacePenaltyPair, FixedFeatureEnergyGradientForObliqueT3AndQ4) {
  for (bool triangle : {false, true}) {
    Case value;
    for (unsigned i = 0; i < 4; ++i) {
      value.x[3 * i] += .3;
      value.x[3 * i + 1] += .4;
    }
    if (triangle) {
      const std::uint32_t a[3]{0, 1, 2}, b[3]{4, 5, 6};
      const double weights[3]{.25, .25, .5};
      ASSERT_EQ(ct::MakeWeightedT3Point(a, 8, weights, &value.a.point), ct::Status::kOk);
      ASSERT_EQ(ct::MakeWeightedT3Point(b, 8, weights, &value.b.point), ct::Status::kOk);
    }
    value.a.reference_half_thickness_m = .5;
    value.b.reference_half_thickness_m = .5;
    ASSERT_TRUE(RefreshGap(value));
    ct::SurfacePenaltyPacket result;
    ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(value.input(), &result), S::Ok);
    for (unsigned i = 0; i < result.count; ++i) {
      const auto node = result.nodes[i].node;
      const double force[3]{result.nodes[i].force_n.x, result.nodes[i].force_n.y, result.nodes[i].force_n.z};
      for (unsigned axis = 0; axis < 3; ++axis) {
        Case plus = value, minus = value;
        constexpr double step = 0x1p-16;
        plus.x[3 * node + axis] += step;
        minus.x[3 * node + axis] -= step;
        ASSERT_TRUE(RefreshGap(plus));
        ASSERT_TRUE(RefreshGap(minus));
        ct::SurfacePenaltyPacket p, m;
        ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(plus.input(), &p), S::Ok);
        ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(minus.input(), &m), S::Ok);
        EXPECT_NEAR((p.elastic_energy_j - m.elastic_energy_j) / (2 * step), -force[axis], 3e-9);
      }
    }
  }
}
TEST(SurfacePenaltyPair, SharedNodeCancellationMasksAndFrozenBodyCongruence) {
  Case value;
  // Equal .25 coefficients on shared node zero cancel before bounds. Remaining
  // support still separates the endpoints, so no invented zero-distance normal.
  value.b.point.nodes[0] = 0;
  value.a.translation_fixed_bits[0] = 5;
  value.b.translation_fixed_bits[0] = 5;
  value.a.translation_fixed_bits[1] = 4;
  ASSERT_TRUE(RefreshGap(value));
  ct::SurfacePenaltyPacket result;
  ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(value.input(), &result), S::Ok);
  ASSERT_EQ(result.count, 7u);
  ASSERT_EQ(result.nodes[0].node, 0u);
  Near(result.nodes[0].force_n, {}, 0);
  Near(result.normal_majorant.nodes[0].jacobian, {}, 0);
  EXPECT_DOUBLE_EQ(result.normal_majorant.nodes[0].diagonal_n_m, 0);
  EXPECT_NE(result.nodes[1].force_n.z, 0);
  EXPECT_DOUBLE_EQ(result.nodes[1].free_force_n.z, 0);
  EXPECT_DOUBLE_EQ(result.normal_majorant.nodes[1].jacobian.z, 0);
  EXPECT_TRUE(majorant_test::ExactBounds(result.normal_majorant));
  // Arbitrary represented frozen nodal-to-body congruence, not an actual owner.
  double transform[24 * 6]{};
  for (unsigned i = 0; i < 3 * result.count; ++i)
    for (unsigned j = 0; j < 6; ++j)
      transform[i * 6 + j] = (int((i + 3 * j) % 7) - 3) * .125;
  const double generalized[6]{.5, -1, .25, .125, -.75, 2};
  EXPECT_TRUE(majorant_test::ExactCongruence(result.normal_majorant, transform, 6, generalized));
  const auto before = result;
  value.b.translation_fixed_bits[0] = 1;
  EXPECT_EQ(ct::EvaluateSurfacePenaltyPair(value.input(), &result), S::InconsistentMask);
  EXPECT_EQ(std::memcmp(&result, &before, sizeof(result)), 0);
}
TEST(SurfacePenaltyPair, InactiveTouchingAndSignedZeroGapKeepDeclaredProfile) {
  Case value;
  ct::SurfacePenaltyPacket result;
  value.a.reference_half_thickness_m = .125;
  value.b.reference_half_thickness_m = .125;
  ASSERT_TRUE(RefreshGap(value));
  ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(value.input(), &result), S::Ok);
  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.normal_force_n, 0);
  EXPECT_DOUBLE_EQ(result.elastic_energy_j, 0);
  EXPECT_DOUBLE_EQ(result.normal_majorant.stiffness_n_m, 0);
  value.a.reference_half_thickness_m = .25;
  value.b.reference_half_thickness_m = .25;
  value.gap = -0.;
  ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(value.input(), &result), S::Ok);
  EXPECT_TRUE(result.active);
  EXPECT_DOUBLE_EQ(result.normal_force_n, 0);
  EXPECT_DOUBLE_EQ(result.normal_majorant.stiffness_n_m, 32);
  const auto before = result;
  value.gap = std::nextafter(0., 1.);
  EXPECT_EQ(ct::EvaluateSurfacePenaltyPair(value.input(), &result), S::GapMismatch);
  EXPECT_EQ(std::memcmp(&result, &before, sizeof(result)), 0);
}
TEST(SurfacePenaltyPair, LateFailuresPreserveWholePacketAndRetry) {
  Case base;
  ct::SurfacePenaltyPacket output;
  ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(base.input(), &output), S::Ok);
  const auto before = output;
  for (unsigned failure = 0; failure < 8; ++failure) {
    SCOPED_TRACE(failure);
    Case value = base;
    S expected = S::InvalidInput;
    if (failure == 0) value.v[23] = std::numeric_limits<double>::quiet_NaN();
    if (failure == 1) value.b.point.nodes[3] = 6;
    if (failure == 2) { value.b.point.nodes[3] = 8; expected = S::OutOfRange; }
    if (failure == 3) value.b.translation_fixed_bits[3] = 8;
    if (failure == 4) value.b.reference_half_thickness_m = -1;
    if (failure == 5) value.k = 0;
    if (failure == 6) { value.b.point = value.a.point; expected = S::ZeroDistance; }
    if (failure == 7) {
      value.k = std::numeric_limits<double>::max();
      for (unsigned i = 0; i < 4; ++i) {
        value.a.point.weights[i] = i ? 0 : 1;
        value.b.point.weights[i] = i ? 0 : 1;
      }
      expected = S::Unrepresentable;
    }
    EXPECT_EQ(ct::EvaluateSurfacePenaltyPair(value.input(), &output), expected);
    EXPECT_EQ(std::memcmp(&output, &before, sizeof(output)), 0);
    ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(base.input(), &output), S::Ok);
  }
}
TEST(SurfacePenaltyPair, TangentialEnergyCurvatureIsOutsideNormalMajorant) {
  Case value;
  ct::SurfacePenaltyPacket center;
  ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(value.input(), &center), S::Ok);
  constexpr double step = 0x1p-12;
  Case plus = value, minus = value;
  for (unsigned i = 0; i < 4; ++i) {
    plus.x[3 * i] += step;
    minus.x[3 * i] -= step;
  }
  ASSERT_TRUE(RefreshGap(plus));
  ASSERT_TRUE(RefreshGap(minus));
  ct::SurfacePenaltyPacket p, m;
  ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(plus.input(), &p), S::Ok);
  ASSERT_EQ(ct::EvaluateSurfacePenaltyPair(minus.input(), &m), S::Ok);
  const double curvature = (p.elastic_energy_j - 2 * center.elastic_energy_j + m.elastic_energy_j) / (step * step);
  EXPECT_NEAR(curvature, value.k * center.gap_m / center.distance_m, 3e-6);
  EXPECT_LT(curvature, 0);
  for (unsigned i = 0; i < center.normal_majorant.count; ++i)
    EXPECT_DOUBLE_EQ(center.normal_majorant.nodes[i].jacobian.x, 0);
}
} // namespace
} // namespace pair_test
