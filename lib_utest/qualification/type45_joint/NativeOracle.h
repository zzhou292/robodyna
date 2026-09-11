// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "native/Native.h"

namespace type45_test {
struct NativeOracle {
  int kind=0;
  double length=1,mass=1,inertia=1,force=1;
  std::array<double,14> property{};
  type45_native::State state;
  type45_native::Startup startup;
  explicit NativeOracle(const Fixture& fixture, int& status);
  bool Step(const Interval&,type45_native::Step&);
};
void CompareReference(const Fixture&,const Reference&,const NativeOracle&);
void CompareStep(const Evaluation&,const NativeOracle&,const type45_native::Step&);
} // namespace type45_test
