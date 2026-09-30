// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Reference.h"
#include "lib_src/elements/solid18/Solid18ForceTypes.h"
#include "lib_src/materials/law44/solid/Update.h"

namespace tl::fea::solid18::law44 {
namespace point = tl::material::law44::solid;
using Material = point::Parameters;
using PrescribedInterval = solid18::PrescribedInterval;
using HistoryStamp = solid18::HistoryStamp;
using CurrentGeometry = solid18::CurrentGeometry;
using PointDerivatives = solid18::PointDerivatives;
struct PointHistory {
  point::History material{};
  double density_kg_m3 = 0;
  double storage_volume_m3 = 0;
  double initial_volume_m3 = 0;
  double energy_density_j_m3 = 0;
  double plastic_work_j = 0;
  double bulk_pressure_pa = 0;
};
struct GlobalHistory {
  double stress_pa[6]{};
  double plastic_strain = 0;
  double filtered_rate_per_s = 0;
  double density_kg_m3 = 0;
  double energy_density_j_m3 = 0;
  double plastic_work_j = 0;
  double bulk_pressure_pa = 0;
};
struct HistoryValues {
  PointHistory point[8]{};
  GlobalHistory global{};
  Vec3 saved_local_position_m[7]{};
};
namespace detail { struct HistoryWriter; }
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
  Material material_{};  // Curve backing is borrowed and immutable.
  bool prepared_ = false;
  friend struct detail::HistoryWriter;
};
struct PointObservation {
  point::Result material{};
  double engineering_rate_per_s[6]{};
  double selective_volume_increment = 0;
  double storage_volume_factor = 0;
  double volume_increment_m3 = 0;
  double relative_density = 0;
  double average_volume_m3 = 0;
  double internal_work_j = 0;
  double plastic_work_increment_j = 0;
  double bulk_pressure_pa = 0;
  double unscaled_element_dt_s = 0;
  double raw_stiffness_n_m = 0;
};
struct ForceDiagnostics {
  unsigned native_degeneracy = 0;  // Complete DEGENES8 result before +10.
  unsigned caller_degeneracy = 0;
  double center_divergence_per_s = 0;
  double mean_pressure_pa = 0;  // S8EFMOY3 PP, including its Q subtraction.
  double minimum_unscaled_dt_s = 0;
  double raw_stiffness_n_m = 0;
  double internal_work_increment_j = 0;
  double plastic_work_increment_j = 0;
};
struct ForceTrial {
  History proposed_history;
  CurrentGeometry geometry;
  PointObservation point[8]{};
  Vec3 rhs_force_n[8]{};  // Original eight source slots; no premature coalescing.
  ForceDiagnostics diagnostics;
};
}  // namespace tl::fea::solid18::law44
