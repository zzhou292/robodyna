// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "lib_utest/qualification/solid_law44_point/NativeOracle.h"
namespace law44_analytic_test {
law44_solid_test::NativeResult Native(const law::Parameters&, const law::History&,
    const law::Input&, bool initialization = false);
}
