// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../radioss_type25_main_geometry/NativeOracle.h"
namespace reader_main_geometry_test {
namespace n=tlfea::contact::radioss_type25;
struct Observation {
  n::NativeInternalMainGeometryResult value;
  unsigned source_corner[4]{};
  bool reversed=false;
};
Observation InternalOracle(const n::NativeExteriorMainGeometryInput&);
double VolumeOracle(const n::Vector (&)[8]);
}
