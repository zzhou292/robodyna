#pragma once
#include "sections/ShellLaw44MembranePoint.h"
#include <type_traits>

namespace tl::fea {
// One genuine point, including its current/saved stress and D1 history.
// Native WPLA is a diagnostic separate from the shell's EINT work.
struct ShellBatchOnePointSectionState {
  sections::MembraneLaw44PointResult point;
  double cumulative_plastic_work_J = 0;
};
static_assert(std::is_trivially_copyable_v<ShellBatchOnePointSectionState>);
static_assert(sizeof(ShellBatchOnePointSectionState) == 224,
              "One-point resident payload is explicitly budgeted");
} // namespace tl::fea
