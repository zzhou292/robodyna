// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../State.h"
#include "Storage.h"
namespace tl::fea::solids::batch_detail::controlled {
TL_BRICK_HD inline int Capture(const h24::Result& result,State<Traits24>& output)noexcept {
  Cache24 cache;cache.profile=ResultProfile::NativeControlled;
  if(!stiffness_detail::Prepare(result.nodal_raw_stiffness_n_m,result.minimum_unscaled_dt_s,.25,cache.stiffness))return -1;
  for(unsigned n=0;n<8;++n)cache.rhs_force_n[n]=result.rhs_force_n[n];
  cache.controlled={result.proposed_history.units(),result.material_raw_stiffness_n_m,
    result.hourglass_raw_stiffness_n_m,result.nodal_raw_stiffness_n_m,result.minimum_unscaled_dt_s,
    result.material_work_increment_j,result.hourglass_work_increment_j,result.distortion_energy_j,result.distortion_work_increment_j};
  output.history.SetNative(result.proposed_history);output.cache=cache;return 0;
}
TL_BRICK_HD inline int Capture(const foam::Result& result,State<Traits18Law90>& output)noexcept {
  Cache18Law90 cache;cache.profile=ResultProfile::NativeControlled;
  if(!stiffness_detail::Prepare(result.nodal_raw_stiffness_n_m,result.minimum_unscaled_dt_s,.25,cache.stiffness))return -1;
  for(unsigned n=0;n<8;++n)cache.rhs_force_n[n]=result.rhs_force_n[n];
  cache.diagnostics={result.minimum_unscaled_dt_s,result.nodal_raw_stiffness_n_m,result.material_work_increment_j};
  cache.controlled={result.proposed_history.units(),result.nodal_raw_stiffness_n_m,0,
    result.last_point_raw_stiffness_after_distortion_n_m,result.minimum_unscaled_dt_s,
    result.material_work_increment_j,0,result.distortion_energy_j,result.distortion_work_increment_j};
  output.history.SetNative(result.proposed_history);output.cache=cache;return 0;
}
TL_BRICK_HD inline int FoamStatus(solid_common::distortion::Status status)noexcept {
  using S=solid_common::distortion::Status;
  if(status==S::Success)return 0;
  if(status==S::UnsupportedProfile)return int(solid18::Status::UnsupportedProfile);
  if(status==S::NonfiniteResult)return int(solid18::Status::NonfiniteResult);
  return int(solid18::Status::InvalidInput);
}
} // namespace tl::fea::solids::batch_detail::controlled
