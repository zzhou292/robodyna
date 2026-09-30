// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "KinematicsTypes.h"
#include "lib_src/materials/law90/CallerTypes.h"

namespace tl::fea::solid18::total_strain {
using Material=tl::material::law90::PreparedMaterial;
using PointHistory=tl::material::law90::CallerHistory;
struct GlobalHistory {
  double stress_pa[6]{};
  double density_kg_m3=0,internal_energy_density_j_m3=0,bulk_pressure_pa=0;
  double scalar_rate_per_s=0; // G_EPSD; no absent PLA/WPLA placeholder.
};
struct HistoryValues {
  PointHistory point[8]{}; // Native IP=r+2*s+4*t; point storage volumes live in Reference.
  GlobalHistory global{};
};
namespace force_detail { struct HistoryWriter; }
class History {
 public:
  TL_SOLID18_HD bool prepared()const noexcept{return prepared_;}
  TL_SOLID18_HD const HistoryValues& data()const noexcept{return values_;}
  TL_SOLID18_HD HistoryStamp stamp()const noexcept{return stamp_;}
  TL_SOLID18_HD const Reference& reference()const noexcept{return reference_;}
  TL_SOLID18_HD const Material& material()const noexcept{return material_;}
 private:
  HistoryValues values_{};
  HistoryStamp stamp_{};
  Reference reference_{};
  Material material_{};
  bool prepared_=false;
  friend struct force_detail::HistoryWriter;
};
struct PointObservation {
  tl::material::law90::CallerResult material{};
  double current_volume_m3=0,storage_volume_m3=0,characteristic_length_m=0;
};
struct ForceDiagnostics {
  double minimum_unscaled_dt_s=0,raw_stiffness_n_m=0,internal_work_increment_j=0;
};
struct ForceTrial {
  History proposed_history;
  PointObservation point[8]{};
  Vec3 rhs_force_n[8]{}; // Native negative internal force, original source slots.
  ForceDiagnostics diagnostics{};
};
struct ForceScratch {
  ForceTrial staged;
  HistoryValues next;
  KinematicsScratch kinematics;
  Vec3 local_force_n[8]{};
};
} // namespace tl::fea::solid18::total_strain
