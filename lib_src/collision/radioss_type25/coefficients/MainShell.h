// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from OpenRadioss, Copyright (C) 2026 Siemens; see ../LICENSE.md.
// Ordinary material branch of I25GAPM, source a62b27e6. Source binding is external.
#pragma once
#include "Common.h"
namespace tlfea::contact::radioss_type25 {
namespace coefficient_detail {
// Raw STC is also needed for a coating partner; do not clamp it to the initial
// ordinary-face STF before publishing that partner. Keep the exact priority.
TL_MATH_HOST_DEVICE inline CoefficientStatus ShellContribution(
    const NativeShellMainCoefficientInput& in, MainFaceKind face, double* output) {
  using namespace coefficient_detail;
  if (!output || !Finite(in.scale) || !Nonnegative(in.element_thickness) ||
      !Nonnegative(in.property_thickness) || !Nonnegative(in.young) || in.stack_material < 0)
    return CoefficientStatus::InvalidInput;
  if (in.face != face ||
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
  *output = contribution; return CoefficientStatus::Ok;
}
} // namespace coefficient_detail
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeShellMainCoefficient(
    const NativeShellMainCoefficientInput& in, NativeScalarCoefficient* output) {
  using namespace coefficient_detail;
  if (!output) return CoefficientStatus::InvalidInput;
  double contribution;
  const auto status = ShellContribution(in, MainFaceKind::OrdinaryExterior, &contribution);
  if (status != CoefficientStatus::Ok) return status;
  const double stiffness = Max(0., contribution); // Preserve legacy zero/MAX behavior.
  if (!Finite(stiffness)) return CoefficientStatus::NonfiniteResult;
  *output = {stiffness}; return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
