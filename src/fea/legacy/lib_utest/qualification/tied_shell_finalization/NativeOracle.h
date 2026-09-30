#pragma once
#include "lib_src/constraints/tied_shell/search/TiedSearchFinalization.h"
#include <array>
#include <vector>
namespace tied_finalization_test {
namespace ts = tl::constraints::tied_shell;
struct NativeResult {
  std::array<int,4> counts{};
  std::vector<int> nsv, msr, selected, irupt;
  std::vector<double> st, stb, dpara, nmas;
  std::vector<std::array<int,9>> events;
  std::vector<std::array<double,3>> event_values;
  int status = -1;
};
NativeResult Native(const ts::FinalizationInput&, int mode = 0, int event_capacity = -1);
// Complete native prefix and warning payload comparison; no production
// compaction or warning helper is called to obtain the expected values.
void Compare(const ts::FinalizationInput&, const ts::FinalizationMaps&, const NativeResult&);
}
