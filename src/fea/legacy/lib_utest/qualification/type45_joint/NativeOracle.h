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
  // Comparison-only packets; never used as inputs to the native calculation.
  GeometryInput geometry;
  Interval last_interval;
  type45_native::State initial_state, previous_state;
  Reference compared_reference;
  HistoryValues compared_history;
  bool comparison_ready=false;
  explicit NativeOracle(const Fixture& fixture, int& status);
  bool Step(const Interval&,type45_native::Step&);
};
void CompareReference(const Fixture&,const Reference&,NativeOracle&);
void CompareStep(const Evaluation&,NativeOracle&,const type45_native::Step&);
} // namespace type45_test
