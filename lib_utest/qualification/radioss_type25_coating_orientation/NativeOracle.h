// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
namespace coating_orientation_test {
namespace s = tlfea::contact::radioss_type25::startup;
s::NativeCoatingOrientationResult Oracle(const s::NativeCoatingOrientationInput&);
}
