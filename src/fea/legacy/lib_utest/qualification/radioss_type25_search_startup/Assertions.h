// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include <gtest/gtest.h>
#include <cstring>
namespace type25_search_startup_test {
inline std::uint64_t Bits(double x){std::uint64_t b;std::memcpy(&b,&x,sizeof(b));return b;}
inline void Same(const s::Snapshot& a,const NativeResult& b) {
  EXPECT_EQ(Bits(a.multiplier),Bits(b.scalar[0]));EXPECT_EQ(Bits(a.mean_length),Bits(b.scalar[1]));
  EXPECT_EQ(Bits(a.margin),Bits(b.scalar[2]));EXPECT_EQ(Bits(a.maximum_extent),Bits(b.scalar[3]));
  ASSERT_EQ(a.primary_count,b.extent.size());ASSERT_EQ(a.main_count+1,b.main_offsets.size());
  ASSERT_EQ(a.secondary_count+1,b.secondary_offsets.size());ASSERT_EQ(a.removal_count,b.removed_nodes.size());
  ASSERT_EQ(a.removal_count,b.removed_mains.size());ASSERT_EQ(a.secondary_count,b.contact.size());
  for(std::size_t i=0;i<a.primary_count;++i)EXPECT_EQ(Bits(a.primary_extent[i]),Bits(b.extent[i]));
  for(std::size_t i=0;i<=a.main_count;++i)EXPECT_EQ(a.main_offsets[i],b.main_offsets[i]);
  for(std::size_t i=0;i<=a.secondary_count;++i)EXPECT_EQ(a.secondary_offsets[i],b.secondary_offsets[i]);
  for(std::size_t i=0;i<a.secondary_count;++i)EXPECT_EQ(a.initial_contact[i],b.contact[i]);
  for(std::size_t i=0;i<a.removal_count;++i){EXPECT_EQ(a.removed_nodes[i],b.removed_nodes[i]);EXPECT_EQ(a.removed_mains[i],b.removed_mains[i]);}
}
inline std::vector<unsigned char> Bytes(const Built& b) {
  const auto* first=static_cast<const unsigned char*>(b.output.data());return {first,first+b.output.bytes()};
}
} // namespace type25_search_startup_test
