#pragma once
#include "lib_src/constraints/tied_shell/search/TiedSearchFinalization.h"
#include <gtest/gtest.h>
namespace tied_finalization_test {
namespace ts = tl::constraints::tied_shell;
struct Fixture {
  std::vector<std::array<std::uint32_t,4>> masters{{0,1,2,3},{2,3,4,4}};
  std::vector<std::uint32_t> mains{4,6,1,0,5,3,2};
  std::vector<std::uint32_t> slaves{9,8,10,7};
  std::vector<ts::SearchChoice> choices;
  Fixture() : choices(4) {
    for (std::size_t s=0;s<4;++s) {
      auto& c=choices[s];
      c.matched=true;
      c.ordered_master=s%2+1;
      c.projection.s=.1*s;
      c.projection.t=-.2*s;
      c.projection.selection_distance=.3+s;
    }
  }
  ts::FinalizationInput Input() const {
    return {masters.data(),slaves.data(),mains.data(),choices.data(),11,masters.size(),slaves.size(),mains.size()};
  }
};
}
