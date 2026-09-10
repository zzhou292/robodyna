#pragma once
#include "WallResponseData.h"

namespace tl::qualification::qeph::wall_response {
constexpr double CoarseResponseLimit=.02,FineResponseLimit=.015;
// The existing .75 contraction uses the coarse LOWER and fine UPPER bounds.
// These are named accuracy requirements, not a claimed temporal order.
constexpr double ResponseContractionFloor=1e-8,EnergyContractionFloor=1e-10;
struct ResponseDifference {
  Interval maximum{};
  double time=0;
  unsigned field=0;
};
struct Comparison {
  bool input_valid=false,passed=false;
  unsigned cells=0;
  double selected_h=0,energy_normalization=0;
  std::string screen_index_sha;
  ResponseDifference coarse_medium,medium_fine;
  std::array<Interval,3> residual_ratios{},analytic_differences{};
  bool response_passed=false,energy_passed=false,analytic_passed=false;
  std::string diagnostic;
};

// Checks retained identities, phases and summaries; it does not evaluate a
// force, restore native History, or authenticate a self-provided source hash.
bool ValidateRun(const Run&,bool require_complete,std::string& error);
// One immutable physical experiment, ordered coarse/medium/fine (1,2,4).
// Raw positions retain their common coordinate frame. Only the deformation
// observer removes translation; this comparison performs no alignment.
Comparison Compare(const std::array<Run,3>&);
} // namespace tl::qualification::qeph::wall_response
