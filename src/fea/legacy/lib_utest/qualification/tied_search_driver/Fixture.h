#pragma once
#include "lib_src/constraints/tied_shell/search/Internal.h"
#include <gtest/gtest.h>
namespace tied_driver_test {
namespace ts = tl::constraints::tied_shell;
namespace detail = ts::driver_detail;
struct Fixture {
  std::vector<ts::Vec3> positions{{-10,-10,0},{10,-10,0},{10,10,0},{-10,10,0},
                                {0,0,.1},{100,100,100},{0,0,0}};
  // Distinct declared patches may intentionally share all physical nodes.
  std::vector<ts::SearchMasterInput> masters{{{0,1,2,3},ts::MasterTopology::Quad,1,1},
                                           {{0,1,2,3},ts::MasterTopology::Quad,1,1}};
  std::vector<std::uint32_t> secondaries{4,5,0,6};
  ts::SearchDriverInput Input() const {
    return {positions.data(), masters.data(), secondaries.data(), positions.size(),
            masters.size(), secondaries.size(), .001, 0};
  }
};
inline void SameChoice(const ts::SearchDriverRow& a, const ts::SearchDriverRow& b) {
  EXPECT_EQ(a.choice.matched, b.choice.matched);
  EXPECT_EQ(a.choice.ordered_master, b.choice.ordered_master);
  EXPECT_EQ(a.choice.projection.selection_distance, b.choice.projection.selection_distance);
  EXPECT_EQ(a.choice.projection.s, b.choice.projection.s);
  EXPECT_EQ(a.choice.projection.t, b.choice.projection.t);
  EXPECT_EQ(a.force_patch_status, b.force_patch_status);
  EXPECT_EQ(a.within_native_bounds, b.within_native_bounds);
}
}
