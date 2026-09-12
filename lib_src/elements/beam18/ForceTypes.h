// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Reference.h"
#include "lib_src/materials/law44/beam/Update.h"

namespace tl::fea::beam18 {
using Material = tl::material::law44::beam::Parameters;
namespace point = tl::material::law44::beam;
struct ForceStamp { double time_s = 0; std::uint64_t sample_index = 0; };
struct HistoryValues {
  point::History point[4]{};
  double total_strain[4][3]{};
  Vec3 section_seed{};  // Carried PEVEC3 SKEW; N3 is never a runtime force node.
  Vec3 section_force_n{}, section_moment_nm{};  // Undamped native FOR/MOM.
  double filtered_neutral_rate_per_s = 0;
  double internal_energy_j[2]{};  // Membrane/shear and flexural/torsional EINT.
  double plastic_work_j = 0;
};
struct ForceGeometry { Vec3 axis[3]{}; double length_m = 0; };
struct GeneralizedRate {
  double axial = 0, shear_y = 0, shear_z = 0;
  double curvature_x = 0, curvature_y = 0, curvature_z = 0;
};
struct PrescribedInterval {
  double base_time_s = 0, dt_s = 0;
  std::uint64_t sample_index = 0;
  Vec3 position_endpoint_m[2]{};
  Vec3 velocity_midpoint_m_s[2]{};
  Vec3 angular_velocity_midpoint_rad_s[2]{};
};
struct ForceDiagnostics {
  double translation_stiffness_n_m = 0, rotation_stiffness_nm = 0;
  double minimum_unscaled_dt_s = 0;
  double internal_work_increment_j[2]{};
  double plastic_work_increment_j = 0;
  Vec3 damped_section_force_n{}, damped_section_moment_nm{};
};
class ForceHistory {
 public:
  TL_BEAM18_HD bool prepared() const noexcept { return prepared_; }
  TL_BEAM18_HD const Reference& reference() const noexcept { return reference_; }
  TL_BEAM18_HD const Material& material() const noexcept { return material_; }
  TL_BEAM18_HD const HistoryValues& values() const noexcept { return values_; }
  TL_BEAM18_HD ForceStamp stamp() const noexcept { return stamp_; }
 private:
  Reference reference_{};
  Material material_{};  // Caller retains immutable curve backing.
  HistoryValues values_{};
  ForceStamp stamp_{};
  bool prepared_ = false;
  friend struct ForceHistoryWriter;
};
struct ForceTrial {
  ForceHistory proposed_history{};
  ForceGeometry geometry{};
  GeneralizedRate rate{};
  point::Result point[4]{};
  Vec3 rhs_force_n[2]{}, rhs_couple_nm[2]{};
  ForceDiagnostics diagnostics{};
};
} // namespace tl::fea::beam18
