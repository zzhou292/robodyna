// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid24/controlled_hourglass/UnitResponse.h"
#include "lib_src/materials/law42/MechanicalSlots.h"
#include "lib_src/elements/solid_common/distortion/UnitTypes.h"
namespace tl::fea::solid24::controlled_distortion {
namespace hour=solid24::controlled_hourglass;
namespace distortion=solid_common::distortion;
using Reference=hour::WorkingReference;
TL_BRICK_HD inline ForceStatus PrepareReference(const solid24::Reference& source,const Material& material,
    distortion::UnitScale units,Reference& output)noexcept {
  return hour::PrepareWorkingReference(source,material,units,output);
}
} // namespace tl::fea::solid24::controlled_distortion
