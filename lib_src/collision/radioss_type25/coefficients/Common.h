// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../CoefficientTypes.h"
#include "../NormalResponse.h"
namespace tlfea::contact::radioss_type25::coefficient_detail {
TL_MATH_HOST_DEVICE inline double Max(double a, double b) { return a < b ? b : a; }
TL_MATH_HOST_DEVICE inline double Min(double a, double b) { return b < a ? b : a; }
TL_MATH_HOST_DEVICE inline bool Finite(double x) { return tl::math::Finite(x); }
TL_MATH_HOST_DEVICE inline bool Nonnegative(double x) { return Finite(x) && x >= 0; }
TL_MATH_HOST_DEVICE inline bool Positive(double x) { return Finite(x) && x > 0; }
} // namespace tlfea::contact::radioss_type25::coefficient_detail
