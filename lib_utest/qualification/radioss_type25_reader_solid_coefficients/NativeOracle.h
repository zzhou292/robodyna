// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25Coefficients.h"
#include <array>
namespace reader_solid_test {
namespace n=tlfea::contact::radioss_type25;
struct Case {
  n::NativeInternalSolidMainCoefficientInput input;
  bool internal=true;
  std::array<double,24> second_coordinates{};
  double unused_second_controlled_bulk=0;
};
struct NativeResult {n::NativeSolidMainCoefficientResult value;double second_volume;};
NativeResult Oracle(const Case&);
}
