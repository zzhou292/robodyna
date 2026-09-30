// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ResultValues.h"
#include "../solid18_force/NativeOracle.h"
#include "../solid24_force/NativeOracle.h"
#include "../solid6z_force/NativeOracle.h"
#include "HephRoundoff.h"
namespace solid_resident_test {
void CheckNative(const s::Result18&,const solid18_force_test::NativeResult&);
void CheckNative(const s::Result24&,const heph_test::NativeTrial&,const HephRoundoff& = {});
void CheckNative(const s::Result6z&,const solid6z_force_test::NativeResult&,double storage_volume);
}
