// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss a62b27e6 RINIT3 selected translational STR; see ../LICENSE.md.
#pragma once
#include "Common.h"
#include "../NodalContributionTypes.h"

namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeSpringNodalCoefficient(
    const NativeSpringNodalInput& in, NativeScalarCoefficient* output) {
  using namespace coefficient_detail;
  if (!output) return CoefficientStatus::InvalidInput;
  if (!in.interface_initialization ||
      (in.kind != SpringNodalKind::Type13 && in.kind != SpringNodalKind::Type25))
    return CoefficientStatus::UnsupportedProfile;

  const unsigned count = in.kind == SpringNodalKind::Type13 ? 3 : 2;
  double terms[3]{};
  for (unsigned i = 0; i < count; ++i) {
    if (!Finite(in.translation[i].slope) || !Finite(in.translation[i].scale))
      return CoefficientStatus::InvalidInput;
    terms[i] = in.translation[i].slope * in.translation[i].scale;
    if (!Finite(terms[i])) return CoefficientStatus::NonfiniteResult;
  }
  double length = 1.;
  if (in.length_mode > 0) {
    if (!Nonnegative(in.geometric_length)) return CoefficientStatus::InvalidInput;
    length = in.geometric_length;
  }
  double maximum = Max(terms[0], terms[1]);
  if (count == 3) maximum = Max(maximum, terms[2]);
  const double result = maximum / Max(native_constant::em30, length);
  if (!Finite(result)) return CoefficientStatus::NonfiniteResult;
  *output = {result};
  return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
