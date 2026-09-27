// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid6z/Solid6zForceTypes.h"
#include "lib_src/elements/solid_common/controlled_hourglass/Types.h"
namespace tl::fea::solid6z::controlled_hourglass {
namespace hg=tl::fea::solid_common::controlled_hourglass;
inline constexpr bool NativeGeometricDistortionEnabled=false; // a62 S6ZFORC3 (1==2).
struct HistoryValues {tl::material::law42::CallerHistory material;hg::State controlled_hourglass;};
struct Result {
  HistoryValues proposed_values;hg::Result expanded_hourglass;
  double native_projection[4][3]{};
  Vec3 material_local_force_n[6]{},local_force_n[6]{},world_native_force_n[6]{},world_force_n[6]{};
  double raw_stin_n_m=0;
};
} // namespace tl::fea::solid6z::controlled_hourglass
