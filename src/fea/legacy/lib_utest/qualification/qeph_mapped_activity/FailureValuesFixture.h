// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/qeph/mapped/FailureActivityValues.h"
#include "SerialFailureValues.h"
#include <array>
#include <cstring>
#include <limits>

namespace qeph_activity_test {
namespace fe = tl::fea;
struct FailureCase {
  fe::ShellBatchSectionState section;
  fe::ShellBatchFailureState value;
  fe::ShellFailurePolicy policy = fe::ShellFailurePolicy::None;
  double time = 1;
  bool plastic = false;
  bool expected = false;
};
inline constexpr std::size_t FailureCases = 30;
inline void FillFailureCases(std::array<FailureCase,FailureCases>& cases) {
  using Policy = fe::ShellFailurePolicy;
  for (std::size_t i = 0; i < cases.size(); ++i) {
    auto& c = cases[i];
    if (i % 3 == 1) {
      c.policy = Policy::ConstantAllPoints;
      c.value = fe::ShellBatchFailureState::Constant();
      c.plastic = true;
    } else if (i % 3 == 2) {
      c.policy = Policy::Tab1AnyPoint;
      c.value = fe::ShellBatchFailureState::Tab1();
      c.plastic = true;
    }
  }
  for (unsigned i = 0; i < 9; ++i) cases[i].expected = true;
  // None (LAW1/skin) accepts signed-zero reserved forces, but no other value.
  cases[3].value.current_force_point[2].stress[4] = -0.;
  cases[6].time = -0.;
  // Constant: partial point removal leaves the parent active; all three remove it.
  auto& partial = cases[4];
  partial.value.constant_points()[0] = {1,.25,false};
  partial.value.current_force_point[0].stress[1] = 4;
  auto& removed = cases[7];
  removed.value.active = false;
  for (unsigned p = 0; p < 3; ++p) {
    removed.value.constant_points()[p] = {1,.25,false};
    removed.value.current_force_point[p].stress[2] = p + 1;
  }
  // TAB1 removes the parent as soon as one actual point has failed.
  for (unsigned i : {5u,8u}) {
    auto& c = cases[i];
    auto& point = c.value.tab1_points()[i == 5 ? 0 : 2];
    point.damage = 1;
    point.maximum_damage = 1;
    point.failure_time_s = .25;
    point.point_active = false;
    c.value.active = false;
  }
  const double nan = std::numeric_limits<double>::quiet_NaN();
  cases[9].value.current_force_point[1].stress[2] = 1;
  cases[10].value.constant_points()[0].damage = -1;
  cases[11].value.tab1_points()[1].damage = nan;
  cases[12].value.active = false;
  cases[13].value.constant_points()[2].failure_time_s = 2;
  cases[14].value.tab1_points()[0].maximum_damage = .5;
  cases[15].value.current_force_point[0].stress[0] = nan;
  cases[16].section.history.point[0].stress[1] = 1;
  cases[17].value.current_force_point[2].stress[3] = 1;
  *reinterpret_cast<unsigned char*>(&cases[18].value.active) = 2;
  *reinterpret_cast<unsigned char*>(&cases[19].value.constant_points()[1].point_active) = 255;
  *reinterpret_cast<unsigned char*>(&cases[20].value.tab1_points()[2].point_active) = 2;
  cases[21].policy = Policy::ConstantAllPoints;
  cases[22].plastic = false;
  cases[23].time = nan;
  cases[24].time = -1;
  cases[25].value.constant_points()[0].damage = 1;
  cases[26].value.tab1_points()[2].failure_time_s = .5;
  cases[27].policy = static_cast<Policy>(255);
  cases[28].value.active = false;
  cases[29].value.active = false;
}
inline const fe::ShellBatchSectionState* Section(const FailureCase& c) {
  return c.plastic ? &c.section : nullptr;
}
inline bool SerialFailure(const FailureCase& c) {
  return c.value.policy() == c.policy && frozen_failure::ValidFailureEncoding(c.value) &&
      frozen_failure::ValidFailureState(c.value,c.policy,Section(c),c.time);
}
} // namespace qeph_activity_test
