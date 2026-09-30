// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellGlobalLaw1Profile.h"
#include "lib_src/math/ScalarBits.h"
namespace tl::fea {
enum class ShellParentExecutionPolicy : unsigned char { FromSection, GlobalLaw1Npt0 };
// A resolved per-parent/group execution choice. Raw MID/SID/NIP stay in their
// original declarations; shared properties are never rewritten or duplicated.
struct ShellParentExecution {
  ShellParentExecutionPolicy policy=ShellParentExecutionPolicy::FromSection;
  ShellGlobalLaw1Profile global_law1{};
};
inline bool SameShellParentExecution(const ShellParentExecution& a,const ShellParentExecution& b) noexcept {
  if(a.policy!=b.policy||a.global_law1.thickness!=b.global_law1.thickness)return false;
  return tl::math::SameScalarBits(a.global_law1.coefficient_working_length_m,
      b.global_law1.coefficient_working_length_m);
}
inline bool ValidShellParentExecution(const ShellParentExecution& p) noexcept {
  if(p.policy==ShellParentExecutionPolicy::FromSection)
    return SameShellParentExecution(p,ShellParentExecution{});
  return p.policy==ShellParentExecutionPolicy::GlobalLaw1Npt0&&shell_global_law1::Valid(p.global_law1);
}
} // namespace tl::fea
