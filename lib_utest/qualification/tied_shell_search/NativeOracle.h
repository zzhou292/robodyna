// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
namespace tied_search_test {
struct NativeChoice {
  int selected=0;
  double st[2]{};
  double distance=std::numeric_limits<double>::max();
};
ts::CandidateProjection Native(const ts::SearchInput&,int,NativeChoice&);
ts::CandidateProjection Native(const ts::WorkingSearchInput&,int,NativeChoice&);
} // namespace tied_search_test
