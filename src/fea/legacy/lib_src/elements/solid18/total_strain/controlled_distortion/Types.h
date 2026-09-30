// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Reference.h"
namespace tl::fea::solid18::total_strain::controlled_distortion {
// Private history retains working-unit values across accepted steps. Only the
// observer/output boundary converts to SI; no repeated force/history roundtrip.
class History {
 public:
  TL_SOLID18_HD bool prepared()const noexcept{return native_.prepared();}
  TL_SOLID18_HD const total_strain::History& native_history()const noexcept{return native_;}
  TL_SOLID18_HD double native_distortion_energy()const noexcept{return energy_;}
  TL_SOLID18_HD distortion::UnitScale units()const noexcept{return units_;}
 private:
  total_strain::History native_;
  double energy_=0;
  distortion::UnitScale units_{};
  friend struct HistoryWriter;
};
struct Result {
  History proposed_history;
  Vec3 rhs_force_n[8]{};
  double nodal_raw_stiffness_n_m=0; // Accumulated STIN -> SCUMU3, unchanged by distortion.
  double last_point_raw_stiffness_after_distortion_n_m=0; // STI, observation only.
  double minimum_unscaled_dt_s=0,material_work_increment_j=0;
  double distortion_energy_j=0,distortion_work_increment_j=0;
};
// Caller-owned unpublished storage; never alias an accepted History/Result.
struct Scratch {
  ForceScratch force;
  distortion::PreparedForceValues distortion;
  distortion::DampingActivity activity;
  bool valid=false;
};
struct HistoryWriter {
  TL_SOLID18_HD static void Set(const total_strain::History& history,double energy,
      distortion::UnitScale units,History& output) noexcept {
    output.native_=history;output.energy_=energy;output.units_=units;
  }
};
} // namespace tl::fea::solid18::total_strain::controlled_distortion
