// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"

namespace type45_test {
// Test-only sensitivity of the length-derived force and its offset couple.
// All other native comparisons retain their original tolerance.
struct LengthRoundoff {
  Vec3 local_force, world_force, endpoint_couple[2];
  Vec3 projection_force_error[2]; // Independent precise-packet witness, SI/native.
};
LengthRoundoff CheckLengthRoundoff(const Evaluation&,const NativeOracle&,const type45_native::Step&);
bool WithinLengthBound(double actual,double expected,double length_bound);
} // namespace type45_test
