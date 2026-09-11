// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
namespace beam18_test {
struct NativeResult {
  std::array<double,32> values{};
  std::array<double,3> node_stiffness{}, node_rotation{};
  double part_mass = 0;
  int status = 1;
};
NativeResult Native(const beam::Input&);
void Compare(const beam::Reference&,const NativeResult&);
} // namespace beam18_test
