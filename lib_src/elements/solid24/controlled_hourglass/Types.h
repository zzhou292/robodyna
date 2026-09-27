// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid24/Solid24ForceTypes.h"
#include "lib_src/elements/solid_common/controlled_hourglass/Types.h"

namespace tl::fea::solid24::controlled_hourglass {
namespace hg=tl::fea::solid_common::controlled_hourglass;
// Selected SI LAW42 controlled recurrence. No conversion from legacy Pa FHOUR.
struct HistoryValues {
  tl::material::law42::CallerHistory material;
  hg::State controlled_hourglass;
};
// Partial native stage; distortion, nodal assembly and owner publication follow.
struct BeforeDistortionResult {
  HistoryValues proposed_values;
  hg::Result hourglass;
  Vec3 local_force_after_material_n[8]{};
  Vec3 world_native_force_before_distortion_n[8]{};
  Vec3 world_force_before_distortion_n[8]{}; // Original source slots, observation.
  double raw_stiffness_before_distortion_n_m=0;
};
} // namespace tl::fea::solid24::controlled_hourglass
