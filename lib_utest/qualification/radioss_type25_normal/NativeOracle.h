// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25Normal.h"
namespace type25_normal_test {
namespace normal = tlfea::contact::radioss_type25;
normal::NativeNormalResult Oracle(const normal::ResolvedNormalConfig&,
    const normal::NativeNormalInput&, const normal::NativeNormalHistory&, bool foreign_slot = false);
} // namespace type25_normal_test
