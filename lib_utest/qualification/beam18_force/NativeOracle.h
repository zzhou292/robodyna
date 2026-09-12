// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
namespace beam18_force_test {
struct NativeState {
  std::array<double,27> reference{};
  std::array<double,41> history{};
  std::array<int,4> cursor{};
};
struct NativeResult {
  NativeState next;
  std::array<double,85> si{};
  std::array<double,3> work_increment_j{}, work_difference_scale_j{};
  int status = -1;
};
NativeResult Native(const b::Reference&,const b::Material&,const NativeState&,
    const b::PrescribedInterval&,bool initial=false);
std::array<double,85> Values(const b::ForceTrial&);
void Compare(const b::ForceTrial&,const NativeResult&);
} // namespace beam18_force_test
