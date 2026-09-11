// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <gtest/gtest.h>
#include <limits>
using namespace solid18_test;

TEST(Solid18Reference, AffinePartitionAndMass) {
  auto in = Cube();
  s::Reference r;
  ASSERT_EQ(s::InitializeReference(in, r), s::Status::Success);
  EXPECT_TRUE(r.prepared());
  EXPECT_NEAR(r.geometry().volume_m3, 1, 1e-14);
  EXPECT_NEAR(r.mass().element_mass_kg, 1070, 1e-11);
  for (unsigned n = 0; n < 8; ++n) {
    EXPECT_EQ(r.source_slot(n), n);
    EXPECT_EQ(r.input().source_node_id[n], in.source_node_id[n]);
    EXPECT_NEAR(r.mass().source_nodal_mass_kg[n], 1070./8, 1e-12);
  }
  for (const auto& p : r.geometry().point) {
    double sum = 0;
    s::Vec3 gradient;
    for (unsigned n = 0; n < 8; ++n) {
      sum += p.shape[n];
      gradient = s::detail::Add(gradient, p.derivative_per_m[n]);
    }
    EXPECT_NEAR(sum, 1, 1e-14);
    EXPECT_NEAR(s::detail::Dot(gradient, gradient), 0, 1e-28);
  }
}

TEST(Solid18Reference, DistortedWeightsAndNativeOrientationMap) {
  auto in = Distorted();
  s::Reference r, reversed;
  ASSERT_EQ(s::InitializeReference(in, r), s::Status::Success);
  double sum = 0;
  for (double value : r.mass().source_nodal_mass_kg) sum += value;
  EXPECT_NEAR(sum, r.mass().element_mass_kg, 1e-10);
  EXPECT_GT(std::abs(r.mass().source_nodal_mass_kg[0]-r.mass().source_nodal_mass_kg[6]), 1.0);
  for (unsigned n = 0; n < 4; ++n) {
    std::swap(in.position_m[n], in.position_m[n+4]);
    std::swap(in.source_node_id[n], in.source_node_id[n+4]);
  }
  ASSERT_LT(s::detail::SignedCenterVolume(in.position_m), 0);
  ASSERT_EQ(s::InitializeReference(in, reversed), s::Status::Success);
  for (unsigned n = 0; n < 8; ++n) {
    const unsigned original = (n+4)%8;
    EXPECT_EQ(reversed.source_slot(n), original);
    EXPECT_EQ(reversed.input().source_node_id[original], r.input().source_node_id[n]);
    EXPECT_DOUBLE_EQ(reversed.mass().source_nodal_mass_kg[original],r.mass().source_nodal_mass_kg[n]);
  }
}

TEST(Solid18Reference, RejectedShapeAndLateCoefficientPreserveThenRetry) {
  const auto in = Distorted();
  s::Reference out;
  ASSERT_EQ(s::InitializeReference(in, out), s::Status::Success);
  const auto saved = Bytes(out);
  auto bad = in;
  bad.source_node_id[7] = bad.source_node_id[0];
  EXPECT_EQ(s::InitializeReference(bad, out), s::Status::InvalidInput);
  EXPECT_EQ(Bytes(out), saved);
  bad = in;
  bad.position_m[6] = {.1,.1,-1};
  EXPECT_EQ(s::InitializeReference(bad, out), s::Status::InvalidGeometry);
  EXPECT_EQ(Bytes(out), saved);
  bad = in;
  bad.density_kg_m3 = std::numeric_limits<double>::max();
  EXPECT_EQ(s::InitializeReference(bad, out), s::Status::NonfiniteResult);
  EXPECT_EQ(Bytes(out), saved);
  const auto expected = Values(out);
  ASSERT_EQ(s::InitializeReference(out.input(), out), s::Status::Success);
  EXPECT_EQ(Values(out), expected);
}

TEST(Solid18Reference, ExplicitRoleAndCollapsedCellStayClosed) {
  auto in = Cube();
  s::Reference out;
  const auto saved = Bytes(out);
  in.profile.material_law = 44;
  EXPECT_EQ(s::InitializeReference(in, out), s::Status::UnsupportedProfile);
  EXPECT_EQ(Bytes(out), saved);
  in = Cube();
  for (auto& x : in.position_m) x.z = 0;
  EXPECT_EQ(s::InitializeReference(in, out), s::Status::InvalidGeometry);
  EXPECT_EQ(Bytes(out), saved);
  ASSERT_EQ(s::InitializeReference(Cube(), out), s::Status::Success);
}
