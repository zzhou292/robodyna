// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/radioss_type25/runtime/RowLaunch.h"
#include <gtest/gtest.h>
#include <limits>
#include <vector>
namespace rd = tlfea::contact::radioss_type25::runtime_detail;
TEST(NativeContactRowLaunch, AdmitsSmallAndFormerlyCappedRosters) {
  EXPECT_EQ(rd::IndependentRowBlocks(0), 1u);
  EXPECT_EQ(rd::IndependentRowBlocks(1), 1u);
  EXPECT_EQ(rd::IndependentRowBlocks(128), 1u);
  EXPECT_EQ(rd::IndependentRowBlocks(129), 2u);
  EXPECT_EQ(rd::IndependentRowBlocks(32768), 256u);
  EXPECT_EQ(rd::IndependentRowBlocks(32769), 257u);
  EXPECT_EQ(rd::IndependentRowBlocks(359583), 2810u);
  EXPECT_EQ(rd::IndependentRowBlocks(376934), 2945u);
}
TEST(NativeContactRowLaunch, BoundedGridAndStrideDoNotOverflow) {
  constexpr std::size_t full = std::size_t(rd::MaximumRowBlocks)*rd::RowThreads;
  EXPECT_EQ(rd::IndependentRowBlocks(full), rd::MaximumRowBlocks);
  EXPECT_EQ(rd::IndependentRowBlocks(full+1), rd::MaximumRowBlocks);
  EXPECT_EQ(rd::IndependentRowBlocks(std::numeric_limits<std::size_t>::max()), rd::MaximumRowBlocks);
  EXPECT_LT(full, std::numeric_limits<unsigned>::max());
}
TEST(NativeContactRowLaunch, EveryRowHasOneWriterAcrossBlockAndGridBoundaries) {
  const std::size_t sizes[]{0, 1, 127, 128, 129, 32767, 32768, 32769,
      std::size_t(rd::MaximumRowBlocks)*rd::RowThreads+129};
  for (const auto rows : sizes) {
    SCOPED_TRACE(rows);
    std::vector<unsigned char> writers(rows);
    const auto stride = rd::IndependentRowBlocks(rows)*rd::RowThreads;
    for (unsigned thread = 0; thread < stride; ++thread)
      for (std::size_t row = thread; row < rows; row += stride) ++writers[row];
    for (const auto count : writers) ASSERT_EQ(count, 1);
  }
}
