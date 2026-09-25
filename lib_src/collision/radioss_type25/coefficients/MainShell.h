// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from OpenRadioss, Copyright (C) 2026 Siemens; see ../LICENSE.md.
// Ordinary material branch of I25GAPM, source a62b27e6. No coating/stack binding.
#pragma once
#include "Common.h"
namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeShellMainCoefficient(
    const NativeShellMainCoefficientInput& in, NativeScalarCoefficient* output) {
  using namespace coefficient_detail;
  if (!output || !Finite(in.scale) || !Nonnegative(in.element_thickness) ||
      !Nonnegative(in.property_thickness) || !Nonnegative(in.young) || in.stack_material < 0)
    return CoefficientStatus::InvalidInput;
  if (in.face != MainFaceKind::OrdinaryExterior ||
      (in.layout != ShellLayout::Quad4 && in.layout != ShellLayout::Triangle3) || in.scale < 0 ||
      in.property_type == 52 || (in.stack_material > 0 &&
          (in.property_type == 11 || in.property_type == 17 || in.property_type == 51)))
    return CoefficientStatus::UnsupportedProfile;
  double contribution;
  if (in.element_thickness != 0 && in.input_thickness_mode == 0)
    contribution = in.scale * in.element_thickness * in.young;
  else if (in.property_type == 17 || in.property_type == 51)
    contribution = in.scale * in.element_thickness * in.young;
  else
    contribution = in.scale * in.property_thickness * in.young;
  if (!Finite(contribution)) return CoefficientStatus::NonfiniteResult;
  const double stiffness = Max(0., contribution); // Ordinary exterior starts STF=ZERO.
  if (!Finite(stiffness)) return CoefficientStatus::NonfiniteResult;
  *output = {stiffness}; return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
