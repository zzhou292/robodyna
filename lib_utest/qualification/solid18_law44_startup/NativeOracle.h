// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
namespace rear_startup_test {
rear_force_test::NativeResult NativeInitialize(const law::Material&,
    const s::ReferenceInput&, s::Vec3 velocity);
}  // namespace rear_startup_test
