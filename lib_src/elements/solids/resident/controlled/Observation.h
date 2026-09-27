// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/UnitConversions.h"
namespace tl::fea::solids {
enum class ResultProfile:unsigned { Legacy,NativeControlled };
struct ControlledObservation {
  tlfea::contact::radioss_type25::UnitScale units{};
  double material_raw_stiffness_n_m=0,hourglass_raw_stiffness_n_m=0,after_distortion_raw_stiffness_n_m=0;
  double material_dt_s=0,material_work_j=0,hourglass_work_j=0;
  double distortion_energy_j=0,distortion_work_j=0;
};
} // namespace tl::fea::solids
