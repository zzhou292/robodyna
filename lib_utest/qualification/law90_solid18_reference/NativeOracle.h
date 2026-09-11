// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
namespace law90_reference_test {
struct NativeReference {
  std::array<double,ReferenceCount> values{};
  int source_slot[8]{};
  int allocation[4]{};
  int status=-1;
};
NativeReference ReferenceOracle(const s::ReferenceInput&);
struct NativeCurrent {
  std::array<double,CurrentCount> values{};
  int status=-1;
};
NativeCurrent CurrentOracle(const s::ReferenceInput&,const t::KinematicsInput&);
}
