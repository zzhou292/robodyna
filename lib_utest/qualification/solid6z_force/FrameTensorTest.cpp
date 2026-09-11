// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/elements/solid_common/FrameTensor.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace {
namespace c = tl::fea::solid_common;
TEST(Solid6zFrameTensor, NonsymmetricGradientUsesFrameColumnsAndSupportsAlias) {
  c::Matrix3 frame;
  const double rotation[9]{0,-1,0,1,0,0,0,0,1};
  for (unsigned k = 0; k < 9; ++k) frame.v[k] = rotation[k];
  double gradient[9]{1,2,3,4,5,6,7,8,9};
  const double expected[9]{5,-4,6,-2,1,-3,8,-7,9};
  ASSERT_TRUE(c::MaterialGradient(frame,gradient,gradient));
  for (unsigned k = 0; k < 9; ++k) EXPECT_EQ(gradient[k],expected[k]);
}
TEST(Solid6zFrameTensor, NonfiniteInputAndLateOverflowPreserveOutputThenRetry) {
  c::Matrix3 frame;
  const double identity[9]{1,0,0,0,1,0,0,0,1};
  for (unsigned k = 0; k < 9; ++k) frame.v[k] = identity[k];
  double gradient[9]{1,2,3,4,5,6,7,8,9};
  double output[9]{-1,-2,-3,-4,-5,-6,-7,-8,-9};
  unsigned char saved[sizeof(output)];
  std::memcpy(saved,output,sizeof(output));
  gradient[8] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(c::MaterialGradient(frame,gradient,output));
  EXPECT_EQ(std::memcmp(saved,output,sizeof(output)),0);
  gradient[8] = std::numeric_limits<double>::max();
  frame.v[8] = 2;
  EXPECT_FALSE(c::MaterialGradient(frame,gradient,output));
  EXPECT_EQ(std::memcmp(saved,output,sizeof(output)),0);
  gradient[8] = 9;
  frame.v[8] = 1;
  ASSERT_TRUE(c::MaterialGradient(frame,gradient,output));
  for (unsigned k = 0; k < 9; ++k) EXPECT_EQ(output[k],gradient[k]);
}
}  // namespace
