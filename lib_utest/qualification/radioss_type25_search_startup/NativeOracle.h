// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25SearchStartup.h"
#include <array>
#include <vector>
namespace type25_search_startup_test {
namespace s=tlfea::contact::radioss_type25::search_startup;
struct NativeResult {
  std::array<double,5> scalar{}; // multiplier, mean, margin, maxextent, source MINSEG.
  std::vector<double> extent;
  std::vector<std::uint32_t> main_offsets,removed_nodes,secondary_offsets,removed_mains;
  std::vector<int> contact;
};
// Original serial preprocessing only; no OpenMP partition-invariance claim.
// Qualification bounds are4096 source nodes/secondaries and512 expanded mains.
NativeResult Oracle(const s::Input&);
double OracleMultiplier(int physical_nodes);
} // namespace type25_search_startup_test
