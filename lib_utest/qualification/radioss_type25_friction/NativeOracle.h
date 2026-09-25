// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
namespace type25_friction_test {
n::NativeFrictionResult Oracle(const Case&, bool foreign_slot = false);
n::HistoryPhaseResult BeginOracle(const n::NativeContactRow&, n::HistoryPhaseInput);
n::NativeContactRow EndOracle(const n::NativeContactRow&);
} // namespace type25_friction_test
