// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <gtest/gtest.h>

namespace solid18_test {
namespace {
void Compare(const s::ReferenceInput& input) {
  s::Reference actual;
  const auto expected = Native(input);
  ASSERT_EQ(expected.status,0);
  ASSERT_EQ(s::InitializeReference(input,actual),s::Status::Success);
  ASSERT_TRUE(Agree(Values(actual),expected.values));
  for (unsigned n = 0; n < 8; ++n) {
    EXPECT_EQ(actual.source_slot(n),expected.source_slot[n]);
    EXPECT_EQ(actual.input().source_node_id[n],input.source_node_id[n]);
  }
}
}
TEST(Solid18ReferenceNative, CompleteFrameGaussWeightsAndMass) {
  Compare(Cube());
  if (HasFatalFailure()) return;
  Compare(Distorted());
  if (HasFatalFailure()) return;
  auto rotated = Distorted();
  const double c = std::cos(.71), z = std::sin(.71);
  for (auto& x : rotated.position_m) {
    x = {c*x.x-z*x.y+2.5,z*x.x+c*x.y-.87,x.z+4.1};
  }
  Compare(rotated);
}
TEST(Solid18ReferenceNative, NativeReversalPreservesSourceSlotMass) {
  auto input = Distorted();
  for (unsigned n = 0; n < 4; ++n) {
    std::swap(input.position_m[n],input.position_m[n+4]);
    std::swap(input.source_node_id[n],input.source_node_id[n+4]);
  }
  const auto native = Native(input);
  ASSERT_EQ(native.status,0);
  for (unsigned n = 0; n < 8; ++n) EXPECT_EQ(native.source_slot[n],(n+4)%8);
  Compare(input);
}
TEST(Solid18ReferenceNative, NativeNegativePointJacobianRejectionAndRetry) {
  auto bad = Cube();
  bad.position_m[6] = {.1,.1,-1};
  const auto rejected = Native(bad);
  ASSERT_GT(rejected.status,0);
  s::Reference actual;
  ASSERT_EQ(s::InitializeReference(Distorted(),actual),s::Status::Success);
  const auto saved = Bytes(actual);
  EXPECT_EQ(s::InitializeReference(bad,actual),s::Status::InvalidGeometry);
  EXPECT_EQ(Bytes(actual),saved);
  Compare(Distorted());
}
}  // namespace solid18_test
