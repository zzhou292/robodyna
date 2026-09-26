// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25GapSource.h"
#include <vector>
namespace gap_source_test {
namespace g=tlfea::contact::radioss_type25::source_gaps;
namespace n=tlfea::contact::radioss_type25;
struct Result {
  std::vector<double> secondary,main_nodes;
  std::vector<g::MainGapFields> mains;
  double minimum_secondary=0,maximum_secondary=0;
};
// Bounded qualification-only raw native value oracle, no source authority.
Result Oracle(const g::Input&);
} // namespace gap_source_test
