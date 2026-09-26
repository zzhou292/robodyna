// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25MainGeometry.h"
namespace main_geometry_test {
namespace n = tlfea::contact::radioss_type25;
n::NativeExteriorMainGeometryResult Oracle(const n::NativeExteriorMainGeometryInput&);
double NativeEm20();
} // namespace main_geometry_test
