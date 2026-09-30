// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25Coefficients.h"
namespace coated_coefficient_test {
namespace n = tlfea::contact::radioss_type25;
n::NativeCoatedMainCoefficientResult Oracle(const n::NativeCoatedMainCoefficientInput&);
}
