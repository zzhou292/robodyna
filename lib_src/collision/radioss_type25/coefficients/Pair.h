// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from OpenRadioss, Copyright (C) 2026 Siemens; see ../LICENSE.md.
// Observed local I25COR3_3 IGSTI4 and final clamp, source a62b27e6.
#pragma once
#include "Common.h"
namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativePairCoefficient(
    PairCoefficientProfile profile, const NativePairCoefficientInput& in,
    NativeScalarCoefficient* output) {
  using namespace coefficient_detail;
  if (!output || !Finite(in.main) || !Finite(in.secondary) || !Nonnegative(in.minimum) ||
      !Nonnegative(in.maximum) || in.minimum > in.maximum) return CoefficientStatus::InvalidInput;
  if (profile.stiffness_formulation != 4 || profile.mass_timestep_augmentation != 0)
    return CoefficientStatus::UnsupportedProfile;
  const double side = Min(in.main, ::fabs(in.secondary));
  const double result = Max(in.minimum, Min(side, in.maximum));
  if (!Finite(result)) return CoefficientStatus::NonfiniteResult;
  *output = {result}; return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
