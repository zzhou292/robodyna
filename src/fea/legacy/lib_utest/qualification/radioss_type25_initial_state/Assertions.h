#pragma once
#include "Fixture.h"
#include <gtest/gtest.h>
#include <cstring>
namespace initial_state_test {
inline std::uint64_t Bits(double value){std::uint64_t out;std::memcpy(&out,&value,8);return out;}
inline void Same(const init::Winner& actual,const n::tied_removal::History& expected) {
  for(unsigned i=0;i<4;++i)EXPECT_EQ(actual.row.irtlm[i],expected.irtlm[i]);
  EXPECT_EQ(Bits(actual.row.penetration_offset),Bits(expected.penetration[4]));
  EXPECT_EQ(Bits(actual.row.history.normal.previous_penetration),Bits(expected.penetration[1]));
  EXPECT_EQ(Bits(actual.row.history.normal.staged_penetration),Bits(expected.penetration[0]));
  EXPECT_EQ(Bits(actual.row.history.normal.damping_half_force),Bits(expected.penetration[2]));
  EXPECT_EQ(Bits(actual.row.penetration_auxiliary),Bits(expected.penetration[3]));
}
}
