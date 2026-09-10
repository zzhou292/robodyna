#pragma once
#include "WallBoostComparison.h"

namespace tl::qualification::qeph::wall_recurrence {
struct WallScreenStepSelection {
  double h=0;
  std::array<bool,6> jobs{}; // (1,0),(1,-8),(1,+8),(2,0),(2,-8),(2,+8).
  std::array<bool,4> boosts{}; // (1,-8),(1,+8),(2,-8),(2,+8).
  bool passed=false;
};
struct WallScreenSelection {
  std::array<WallScreenStepSelection,6> steps;
  double selected_h=0;
  bool input_valid=false,passed=false;
  std::string diagnostic;
};
// Exactly six validated job summaries and four freshly computed boost
// comparisons, in any input order. Retains 4H0 diagnostics but selects from
// individual points using the existing factor-two-margin selector. Pinned
// source/raw/derived receipts and aggregate byte checks belong to the caller.
WallScreenSelection SelectWallScreen(const std::array<WallJobSummary,6>&,
                                     const std::array<WallBoostComparison,4>&);
} // namespace tl::qualification::qeph::wall_recurrence
