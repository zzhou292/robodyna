// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25Assembly.h"
#include <gtest/gtest.h>
#include <algorithm>
namespace ass = tlfea::contact::radioss_type25::assembly;
TEST(Type25AssemblyConsumer, CompleteOccurrenceOrderKeepsNativeCohorts) {
  const ass::Connectivity rows[] = {{{0,0,0,0},0},{{0,0,0,0},0},{{0,0,0,0},0}};
  const std::uint32_t ends[] = {2,3};
  const ass::Schedule schedule{ends,2,3};
  std::uint32_t offsets[2],ranks[15];
  ASSERT_TRUE(ass::BuildIncidence(rows,schedule,1,offsets,2,ranks,15));
  EXPECT_EQ(offsets[0],0u); EXPECT_EQ(offsets[1],15u);
  for (unsigned rank = 0; rank < 15; ++rank) EXPECT_EQ(ranks[rank],rank);
  const unsigned expected_row[] = {0,0,0,0,1,1,1,1,0,1,2,2,2,2,2};
  const unsigned expected_slot[] = {0,1,2,3,0,1,2,3,4,4,0,1,2,3,4};
  for (unsigned rank = 0; rank < 15; ++rank) {
    ass::Occurrence value; ASSERT_TRUE(ass::DecodeOccurrence(schedule,rank,&value));
    EXPECT_EQ(value.row,expected_row[rank]); EXPECT_EQ(value.slot,expected_slot[rank]);
  }
}
TEST(Type25AssemblyConsumer, MalformedConnectivityCohortsAndAliasingAreRejectedBeforeWrites) {
  ass::Connectivity row{{0,0,0,0},1}; std::uint32_t end = 1;
  ass::Schedule schedule{&end,1,1};
  std::uint32_t offsets[2] = {77,88},ranks[5] = {91,92,93,94,95};
  EXPECT_FALSE(ass::BuildIncidence(&row,schedule,1,offsets,2,ranks,5));
  EXPECT_EQ(offsets[0],77u); EXPECT_EQ(ranks[4],95u);
  row.secondary = 0; end = 0;
  EXPECT_FALSE(ass::BuildIncidence(&row,schedule,1,offsets,2,ranks,5));
  EXPECT_EQ(offsets[0],77u); EXPECT_EQ(ranks[4],95u);
  end = 1;
  EXPECT_FALSE(ass::BuildIncidence(&row,schedule,1,ranks,2,ranks,5));
  EXPECT_FALSE(ass::BuildIncidence(&row,schedule,1,&row.main[0],2,ranks,5));
  EXPECT_EQ(row.main[0],0u); EXPECT_EQ(ranks[0],91u);
  EXPECT_FALSE(ass::BuildIncidence(&row,schedule,1,offsets,2,ranks,4));
}
TEST(Type25AssemblyConsumer, EmptyRowsHaveAnExplicitCompleteEmptyIncidence) {
  std::uint32_t offsets[4] = {1,2,3,4};
  ASSERT_TRUE(ass::BuildIncidence(nullptr,{},3,offsets,4,nullptr,0));
  for (auto value : offsets) EXPECT_EQ(value,0u);
  const ass::NativeNodalValue input{{-0.,2.,3.},4.}; ass::NativeNodalValue out;
  ASSERT_EQ(ass::GatherNode(1,nullptr,static_cast<const ass::NativeEndpoints*>(nullptr),{},
      {offsets,nullptr,3,0},input,&out),ass::Status::Ok);
  EXPECT_EQ(out.force.y,2.); EXPECT_EQ(out.stiffness,4.);
}
