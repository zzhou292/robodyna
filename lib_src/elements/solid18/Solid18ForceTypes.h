// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected S8EFORC3 packets: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18Types.h"
#include "lib_src/materials/SolidLaw36Point.h"

namespace tl::fea::solid18 {
using Material = tl::material::law36::Parameters;

struct HistoryStamp {
  double time_s = 0;
  std::uint64_t sample_index = 0;  // Caller label, not an owner clock.
};

struct PrescribedInterval {
  Vec3 position_endpoint_m[8]{};  // Original source slots, world coordinates.
  Vec3 velocity_midpoint_m_s[8]{};
  double base_time_s = 0;
  double dt_s = 0;
  std::uint64_t sample_index = 0;
};

struct PointHistory {
  tl::material::law36::CallerHistory material{};
  double density_kg_m3 = 0;
  double storage_volume_m3 = 0;  // LBUF%VOL, corrected by S8EDEFO3.
  double initial_volume_m3 = 0;  // VOL0DP: unchanged in selected IRESP0.
  double bulk_pressure_pa = 0;   // Accepted QVIS; distinct from material pressure.
};

struct GlobalHistory {
  double stress_pa[6]{};
  double plastic_strain = 0;
  double density_kg_m3 = 0;
  double internal_energy_density_j_m3 = 0;
  double plastic_work_j = 0;
  double bulk_pressure_pa = 0;
};

struct HistoryValues {
  PointHistory point[8]{};  // Engine IP=r+2*s+4*t; eight independent points.
  GlobalHistory global{};
  Vec3 saved_local_position_m[7]{}; // S8SAV3: latest x1..7 minus x8.
};

class History {
 public:
  TL_SOLID18_HD bool prepared() const noexcept { return prepared_; }
  TL_SOLID18_HD const HistoryValues& data() const noexcept { return data_; }
  TL_SOLID18_HD HistoryStamp stamp() const noexcept { return stamp_; }
  TL_SOLID18_HD const Reference& reference() const noexcept { return reference_; }
  TL_SOLID18_HD const Material& material() const noexcept { return material_; }
 private:
  HistoryValues data_{};
  HistoryStamp stamp_{};
  Reference reference_{};
  Material material_{};
  bool prepared_ = false;
  friend TL_SOLID18_HD Status PreparePrescribedHistory(const Reference&,
      const Material&, const HistoryValues&, HistoryStamp, History&) noexcept;
};

struct PointDerivatives {
  double regular_per_m[3][8]{}; // PX/PY/PZ, axes then native node slot.
  double shear_per_m[6][8]{};   // PXY/PYX/PXZ/PZX/PYZ/PZY.
  double cross_per_m[6][8]{};   // BXY/BYX/BXZ/BZX/BYZ/BZY.
  Matrix3 inverse_scaled_jacobian_per_m;
  double current_volume_m3 = 0;
};

struct CurrentGeometry {
  Matrix3 frame;
  Vec3 local_position_m[8]{};
  Vec3 local_velocity_m_s[8]{};
  double center_gradient_per_m[3][4]{}; // Other four follow native opposite slots.
  double center_volume_m3 = 0;
  double inverse_center_face_scale_per_m2 = 0;
  PointDerivatives point[8]{};
};

struct PointObservation {
  tl::material::law36::CallerResult material{};
  double engineering_rate_per_s[6]{};
  double selective_volume_increment = 0; // Native SDV, dimensionless.
  double storage_volume_factor = 0;
  double volume_increment_m3 = 0;
  double bulk_pressure_pa = 0;
  double unscaled_element_dt_s = 0;
  double raw_stiffness_n_m = 0; // MQVISCB STI, before SCUMU3's 2/8 scatter.
};

struct ForceDiagnostics {
  double selection_factor = 0;
  double selective_poisson_ratio = 0;
  unsigned selected_point = 0; // Native EPSIP defaults to the first engine point.
  double minimum_unscaled_dt_s = 0;
  double raw_stiffness_n_m = 0;
  double internal_work_increment_j = 0;
  double plastic_work_increment_j = 0;
};

struct ForceTrial {
  History proposed_history;
  CurrentGeometry geometry;
  PointObservation point[8]{};
  Vec3 rhs_force_n[8]{}; // Native negative internal force in original source slots.
  ForceDiagnostics diagnostics;
};
}  // namespace tl::fea::solid18
