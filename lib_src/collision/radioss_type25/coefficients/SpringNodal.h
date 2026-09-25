// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss a62b27e6 RINIT3 selected translational STR; see ../LICENSE.md.
#pragma once
#include "Common.h"
#include "../NodalContributionTypes.h"

namespace tlfea::contact::radioss_type25 {
namespace coefficient_detail::spring_nodal {
// Actual qualified native MAX2/MAX3 retains the later equal operand, including
// signed zero. All operands are already finite before this leaf-local helper.
TL_MATH_HOST_DEVICE inline double Maximum(double first, double later) {
  return first > later ? first : later;
}
} // namespace coefficient_detail::spring_nodal

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
  double maximum = spring_nodal::Maximum(terms[0], terms[1]);
  if (count == 3) maximum = spring_nodal::Maximum(maximum, terms[2]);
  const double result = maximum / Max(native_constant::em30, length);
  if (!Finite(result)) return CoefficientStatus::NonfiniteResult;
  *output = {result};
  return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
