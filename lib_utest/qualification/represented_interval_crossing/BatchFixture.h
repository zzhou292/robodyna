// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
namespace represented_interval_test {
// Extracted unchanged from the existing native compound-roster qualification.
struct Roster {
  std::vector<ct::RepresentedTrianglePath> paths;
  std::vector<ct::RepresentedTrianglePair> pairs;

  explicit Roster(std::size_t count) {
    paths.push_back(Static(10, BaseTriangle()));
    for (std::size_t index = 0; index < count; ++index) {
      const auto eid = 20 + index;
      switch (index % 4) {
        case 0:
          paths.push_back(Static(eid, BaseTriangle(1)));
          break;
        case 1:
          paths.push_back(Static(eid, BaseTriangle()));
          break;
        case 2:
          paths.push_back(Path(eid, BaseTriangle(1), BaseTriangle(-3)));
          break;
        default:
          paths.push_back(Path(eid, BaseTriangle(1), BaseTriangle(-1),
                               0, ct::RepresentedMotion::RigidArc));
          break;
      }
      pairs.push_back({0, static_cast<std::uint32_t>(index + 1)});
    }
  }
};
inline ct::RepresentedIntervalLimits Limits(const Roster& roster, std::size_t capacity) {
  ct::RepresentedIntervalLimits result;
  result.max_paths = roster.paths.size();
  result.max_input_pairs = capacity;
  result.max_results = capacity;
  result.max_work_per_pair = 31;
  result.max_total_work = capacity * result.max_work_per_pair;
  return result;
}
}  // namespace represented_interval_test
