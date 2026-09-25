#include "lib_src/elements/ShellMixedSectionArenaLayout.h"
#include <gtest/gtest.h>
#include <cstring>
namespace global_law1_execution_budget_test {
namespace detail=tl::fea::shell_batch_plasticity_detail;
TEST(GlobalLaw1ExecutionBudget,OptionalProfilesHaveExactArenaCostAndNoLegacyPayload) {
  constexpr std::size_t count=5;detail::MixedLayout legacy,global;
  ASSERT_TRUE(legacy.Initialize(count,3,1u<<20));ASSERT_TRUE(global.Initialize(count,3,1u<<20,true));
  EXPECT_EQ(legacy.global_profiles.count,0u);EXPECT_EQ(global.global_profiles.count,count);
  EXPECT_EQ(global.bytes-legacy.bytes,count*sizeof(tl::fea::ShellGlobalLaw1Profile));
  tl::util::HostArena old_arena,new_arena;ASSERT_TRUE(old_arena.Initialize(legacy.bytes));ASSERT_TRUE(new_arena.Initialize(global.bytes));
  auto* old=legacy.Construct(old_arena);auto* current=global.Construct(new_arena);
  ASSERT_NE(old,nullptr);ASSERT_NE(current,nullptr);EXPECT_EQ(old->global_law1,nullptr);ASSERT_NE(current->global_law1,nullptr);
  for(std::size_t i=0;i<count;++i)EXPECT_TRUE(tl::fea::shell_global_law1::Valid(current->global_law1[i]));
  detail::MixedLayout target=legacy;const auto before=target;
  EXPECT_FALSE(target.Initialize(count,3,global.bytes-1,true));EXPECT_EQ(std::memcmp(&target,&before,sizeof target),0);
  ASSERT_TRUE(target.Initialize(count,3,global.bytes,true));EXPECT_EQ(target.bytes,global.bytes);
}
}
