// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from OpenRadioss, Copyright (C) 2026 Siemens; see ../LICENSE.md.
// Exterior solid contribution/corrections from I25GAPM, source a62b27e6.
#pragma once
#include "Common.h"
namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeSolidMainCoefficient(
    const NativeSolidMainCoefficientInput& in, NativeSolidMainCoefficientResult* output) {
  using namespace coefficient_detail;
  if (!output || !Finite(in.scale) || !Nonnegative(in.fill) || !Positive(in.area) ||
      !Positive(in.volume) || !Nonnegative(in.bulk) || !Nonnegative(in.controlled_bulk))
    return CoefficientStatus::InvalidInput;
  if (in.face != MainFaceKind::OrdinaryExterior || in.scale < 0 ||
      (in.layout != SolidLayout::EightSlot && in.layout != SolidLayout::TenNode &&
       in.layout != SolidLayout::TwentyNode && in.layout != SolidLayout::SixteenNode))
    return CoefficientStatus::UnsupportedProfile;
  const double bulk = in.incompressibility_control == 1 ? in.controlled_bulk : in.bulk;
  NativeSolidMainCoefficientResult result;
  result.stiffness = in.scale * in.fill * in.area * in.area * bulk / in.volume;
  result.characteristic_length = in.volume / in.area;
  if (in.layout == SolidLayout::TenNode) {
    result.characteristic_length = 3. * (1. / 8.) * result.characteristic_length;
    result.stiffness = 16. * result.stiffness;
  } else if (in.layout == SolidLayout::SixteenNode) {
    result.characteristic_length = result.characteristic_length / 4;
  }
  if (!Finite(result.stiffness) || !Finite(result.characteristic_length))
    return CoefficientStatus::NonfiniteResult;
  *output = result; return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
