// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Reference.h"
namespace tl::fea::solid24::controlled_distortion {
class History {
 public:
  TL_BRICK_HD bool prepared()const noexcept{return prepared_;}
  TL_BRICK_HD const hour::HistoryValues& native_values()const noexcept{return values_;}
  TL_BRICK_HD double native_distortion_energy()const noexcept{return distortion_energy_;}
  TL_BRICK_HD const solid24::Reference& reference()const noexcept{return reference_;}
  TL_BRICK_HD const Material& material()const noexcept{return material_;}
  TL_BRICK_HD HistoryStamp stamp()const noexcept{return stamp_;}
  TL_BRICK_HD distortion::UnitScale units()const noexcept{return units_;}
 private:
  hour::HistoryValues values_;double distortion_energy_=0;
  solid24::Reference reference_;Material material_;HistoryStamp stamp_;
  distortion::UnitScale units_{};bool prepared_=false;
  friend struct HistoryWriter;
};
struct Result {
  History proposed_history;Vec3 rhs_force_n[8]{};
  double material_raw_stiffness_n_m=0,hourglass_raw_stiffness_n_m=0,nodal_raw_stiffness_n_m=0;
  double minimum_unscaled_dt_s=0,material_work_increment_j=0,hourglass_work_increment_j=0;
  double distortion_energy_j=0,distortion_work_increment_j=0,internal_energy_density_j_m3=0;
  int damping_applied=0,center_contacts=0,corner_contacts=0;
};
// Caller-owned unpublished scratch. Never alias accepted history or result.
struct Scratch {
  Reference reference;hour::WorkingResult prefix;distortion::PreparedForceValues distortion;
  distortion::DampingActivity activity;HistoryStamp proposed_stamp;bool valid=false;
};
struct HistoryWriter {
  TL_BRICK_HD static void Set(const Scratch& scratch,double distortion_energy,History& output)noexcept {
    output.values_=scratch.prefix.stage.proposed_values;output.distortion_energy_=distortion_energy;
    output.reference_=scratch.reference.reference();output.material_=scratch.reference.material();
    output.stamp_=scratch.proposed_stamp;output.units_=scratch.reference.units();output.prepared_=true;
  }
};
} // namespace tl::fea::solid24::controlled_distortion
