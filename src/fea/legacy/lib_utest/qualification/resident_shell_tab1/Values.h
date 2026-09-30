#pragma once
#include "Source.h"
#include "lib_src/elements/failure/ShellFailureValues.h"

namespace resident_tab1_test {
inline std::vector<double> Values(const fe::ShellBatchFailureState& value) {
  using failure_force_test::Append;
  std::vector<double> out;
  Append(out, static_cast<unsigned>(value.policy()));
  Append(out, value.active);
  Append(out, value.current_force_point[0].stress);
  Append(out, value.current_force_point[1].stress);
  Append(out, value.current_force_point[2].stress);
  if (const auto* points = value.constant_points()) {
    for (unsigned p = 0; p < 3; ++p) {
      Append(out, points[p].damage);
      Append(out, points[p].failure_time_s);
      Append(out, points[p].point_active);
    }
  } else if (const auto* points = value.tab1_points()) {
    for (unsigned p = 0; p < 3; ++p) {
      Append(out, points[p].damage);
      Append(out, points[p].maximum_damage);
      Append(out, points[p].failure_time_s);
      Append(out, points[p].table_segment);
      Append(out, points[p].point_active);
    }
  }
  return out;
}
inline std::vector<double> Values(const fe::ShellBatchSectionState& value) {
  std::vector<double> out;
  failure_force_test::Append(out, value.history);
  failure_force_test::Append(out, value.diagnostics);
  failure_force_test::Append(out, value.cumulative_plastic_work_J);
  return out;
}
} // namespace resident_tab1_test
