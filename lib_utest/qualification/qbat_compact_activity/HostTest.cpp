// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/elements/qbat/activity/Values.h"
#include "lib_src/elements/qeph/mapped/ActivityLayout.h"
#include <gtest/gtest.h>
#include <type_traits>
#include <vector>
namespace qbat_activity_test {
namespace fe=tl::fea; namespace q=fe::qbat; namespace a=q::activity;
TEST(QbatActivityPacket, SharedLayoutPreservesQephAliasAlignmentExactCapsAndFailureAtomicity) {
  static_assert(std::is_same_v<fe::mapped_shell::ActivityLayout,fe::qeph::mapped::ActivityLayout>);
  for(std::size_t parents:{1u,17u,127u,128u,129u,4250u}) {
    fe::mapped_shell::ActivityLayout layout;
    ASSERT_TRUE(layout.Initialize(13,parents,1u<<20));
    EXPECT_EQ(layout.active.offset,layout.first_invalid.offset+sizeof(std::uint32_t));
    EXPECT_EQ(layout.active.bytes,parents);
    EXPECT_EQ(layout.bytes-layout.first_invalid.offset,fe::mapped_shell::ActivityBytes(parents));
    EXPECT_EQ(layout.first_invalid.offset%alignof(std::uint32_t),0u);
    fe::mapped_shell::ActivityLayout exact;
    ASSERT_TRUE(exact.Initialize(13,parents,layout.bytes));
    exact.bytes=79;
    EXPECT_FALSE(exact.Initialize(13,parents,layout.bytes-1)); EXPECT_EQ(exact.bytes,79u);
    tl::util::HostArena arena; ASSERT_TRUE(arena.Initialize(layout.bytes));
    fe::mapped_shell::ActivityMemory packet; ASSERT_TRUE(layout.Construct(arena,packet));
    const auto rebased=layout.Rebase(arena.data());
    EXPECT_EQ(packet.first_invalid,rebased.first_invalid); EXPECT_EQ(packet.active,rebased.active);
  }
  fe::mapped_shell::ActivityLayout invalid; invalid.bytes=83;
  EXPECT_FALSE(invalid.Initialize(0,0,1024)); EXPECT_EQ(invalid.bytes,83u);
  EXPECT_FALSE(invalid.Initialize(0,UINT32_MAX,1024)); EXPECT_EQ(invalid.bytes,83u);
}
TEST(QbatActivityCompletion, CatalogLookupAndDeviceFailureRetainEarliestParentAndLookupOrder) {
  for(std::uint32_t device:{0u,2u,4u,UINT32_MAX}) for(std::uint32_t source:{0u,2u,4u,UINT32_MAX}) {
    const std::uint8_t flags[]{1,0,1,0,1}; std::vector<std::size_t> seen;
    const auto report=a::Complete(device,flags,5,[&](std::size_t p){seen.push_back(p);return p!=source;});
    const auto earliest=std::min(device,source);
    if(earliest==UINT32_MAX) {EXPECT_EQ(report.status,q::BatchStatus::Success);EXPECT_EQ(seen.size(),5u);}
    else {EXPECT_EQ(report.status,q::BatchStatus::NonfiniteResult);EXPECT_EQ(report.element,earliest);EXPECT_EQ(seen.size(),earliest+1u);}
    for(std::size_t p=0;p<seen.size();++p) EXPECT_EQ(seen[p],p);
  }
}
TEST(QbatActivityCompletion, InvalidEncodingJoinsTheSameOrderAndTransportIndexCannotInventAParent) {
  std::uint8_t flags[]{1,1,2,1,1};
  const auto report=a::Complete(4,flags,5,[](std::size_t){return true;});
  EXPECT_EQ(report.status,q::BatchStatus::NonfiniteResult);EXPECT_EQ(report.element,2u);
  unsigned calls=0;
  const auto invalid=a::Complete(5,flags,5,[&](std::size_t){++calls;return true;});
  EXPECT_EQ(invalid.status,q::BatchStatus::NonfiniteResult);EXPECT_EQ(invalid.element,UINT32_MAX);EXPECT_EQ(calls,0u);
}
TEST(QbatActivityBudget, BothResidentProfilesConstructRebaseAndChargeTheNewDevicePacket) {
  namespace b=q::batch_detail;
  for(bool mapped:{false,true}) {
    b::Layout layout;
    ASSERT_TRUE(mapped?layout.InitializeMapped(17,20,0,1u<<20):layout.Initialize(17,20,0,1u<<20));
    EXPECT_EQ(layout.activity.active.count,17u);
    EXPECT_EQ(layout.activity.active.bytes,17u);
    EXPECT_EQ(layout.activity.active.offset,layout.activity.first_invalid.offset+4);
    tl::util::HostArena arena; ASSERT_TRUE(arena.Initialize(layout.bytes));
    const auto* storage=layout.Construct(arena);ASSERT_NE(storage,nullptr);
    const auto rebased=layout.Rebase(*storage,arena.data());
    EXPECT_EQ(storage->activity.active,rebased.activity.active);
    EXPECT_EQ(storage->activity.first_invalid,rebased.activity.first_invalid);
    b::Layout exact;
    ASSERT_TRUE(mapped?exact.InitializeMapped(17,20,0,layout.bytes):exact.Initialize(17,20,0,layout.bytes));
    exact.bytes=97;
    EXPECT_FALSE(mapped?exact.InitializeMapped(17,20,0,layout.bytes-1):exact.Initialize(17,20,0,layout.bytes-1));
    EXPECT_EQ(exact.bytes,97u);
  }
}
} // namespace qbat_activity_test
