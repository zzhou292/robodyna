// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from OpenRadioss, Copyright (C) 2026 Siemens; see ../LICENSE.md.
// ASSTIFI and local I25STSECND arithmetic, source a62b27e6.
#pragma once
#include "Common.h"
namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline CoefficientStatus FinalizeNativeNodalCoefficient(
    const NativeAccumulatedNodalCoefficients& in, NativeNodalCoefficientResult* output) {
  using namespace coefficient_detail;
  if (!output || !Nonnegative(in.volume) || !Nonnegative(in.bulk_volume) ||
      !Nonnegative(in.young_thickness_sum) || !Nonnegative(in.existing_stiffness) ||
      in.shell_incidence_count < 0) return CoefficientStatus::InvalidInput;
  NativeNodalCoefficientResult result;
  // EM30 here is a native VOLUME floor. SI inputs convert before this operation.
  result.normalized_bulk = in.bulk_volume / Max(native_constant::em30, in.volume);
  result.stiffness = in.existing_stiffness +
      result.normalized_bulk * ::pow(2. * in.volume, 1. / 3.);
  // Preserve the source fractional power; substituting cbrt changes its arithmetic.
  if (in.shell_incidence_count > 0)
    result.stiffness = result.stiffness + in.young_thickness_sum / in.shell_incidence_count;
  if (!Finite(result.normalized_bulk) || !Finite(result.stiffness))
    return CoefficientStatus::NonfiniteResult;
  *output = result; return CoefficientStatus::Ok;
}
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeSecondaryCoefficient(
    const NativeSecondaryCoefficientInput& in, NativeScalarCoefficient* output) {
  using namespace coefficient_detail;
  if (!output || !Finite(in.existing) || !Finite(in.scale)) return CoefficientStatus::InvalidInput;
  // The native masked branch does not read STIFINT(J). Preserve even -0 exactly.
  if (in.existing == 0) { *output = {in.existing}; return CoefficientStatus::Ok; }
  if (!Nonnegative(in.global_stiffness)) return CoefficientStatus::InvalidInput;
  const double scale = in.scale > 0 ? in.scale : 1.;
  const double result = scale * in.global_stiffness;
  if (!Finite(result)) return CoefficientStatus::NonfiniteResult;
  *output = {result}; return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
