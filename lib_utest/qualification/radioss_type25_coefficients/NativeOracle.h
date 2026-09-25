// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/CoefficientTypes.h"
namespace type25_coefficient_test {
namespace n = tlfea::contact::radioss_type25;
n::NativeScalarCoefficient Oracle(const n::NativeShellMainCoefficientInput&);
n::NativeSolidMainCoefficientResult Oracle(const n::NativeSolidMainCoefficientInput&);
n::NativeNodalCoefficientResult Oracle(const n::NativeAccumulatedNodalCoefficients&);
n::NativeScalarCoefficient Oracle(const n::NativeSecondaryCoefficientInput&);
n::NativeScalarCoefficient Oracle(const n::NativePairCoefficientInput&);
} // namespace type25_coefficient_test
