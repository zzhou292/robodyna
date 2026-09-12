// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/solvers/NodalCinLayout.h"
#include <gtest/gtest.h>

namespace tl::fea::cin_group_test {
TEST(CinParallelGroupsLayout, ActualGroupTailAndExactCapsLeaveFailedOutputUntouched) {
  nodal_detail::CinLayout old, next;
  NodalCinLimits limits;
  ASSERT_TRUE(old.Initialize(376930, 11165, 13173, limits, 512));
  ASSERT_TRUE(next.Initialize(376930, 11165, 13173, limits, 512, 779));
  EXPECT_EQ(next.group_reports.count, 779u);
  EXPECT_EQ(next.device_bytes-old.device_bytes, 24u*779u);
  EXPECT_EQ(next.optional_device_bytes-old.optional_device_bytes, 24u*779u);
  EXPECT_EQ(next.state_values, old.state_values);
  EXPECT_EQ(next.scratch_values, old.scratch_values);
  EXPECT_EQ(next.host_bytes, old.host_bytes);
  const auto expected = next;
  limits.max_device_bytes = next.optional_device_bytes;
  limits.max_host_bytes = next.host_bytes;
  ASSERT_TRUE(next.Initialize(376930, 11165, 13173, limits, 512, 779));
  --limits.max_device_bytes;
  EXPECT_FALSE(next.Initialize(376930, 11165, 13173, limits, 512, 779));
  EXPECT_EQ(next.device_bytes, expected.device_bytes);
  EXPECT_EQ(next.group_reports.offset, expected.group_reports.offset);
  ++limits.max_device_bytes;
  --limits.max_host_bytes;
  EXPECT_FALSE(next.Initialize(376930, 11165, 13173, limits, 512, 779));
  EXPECT_EQ(next.optional_device_bytes, expected.optional_device_bytes);
  EXPECT_FALSE(next.Initialize(8, 1, 1, NodalCinLimits{}, 512, 5));
  EXPECT_EQ(next.nodes, expected.nodes);
  nodal_detail::CinLayout none;
  ASSERT_TRUE(none.Initialize(376930, 11165, 13173, NodalCinLimits{}, 512, 0));
  EXPECT_EQ(none.device_bytes, old.device_bytes);
  EXPECT_EQ(none.group_reports.count, 0u);
}
} // namespace tl::fea::cin_group_test
