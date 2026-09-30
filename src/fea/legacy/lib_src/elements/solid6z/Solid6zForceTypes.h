// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid6zTypes.h"
#include "lib_src/materials/law42/CallerTypes.h"

namespace tl::fea::solid6z {
using Material = tl::material::law42::Parameters;

enum class StabilizationProfile : std::uint8_t {
  Law42MaterialSoundSpeedV1
};
struct ForceProfile {
  // Explicit corrected S6ZHOUR3 LAW42 profile; see the native repair receipt.
  StabilizationProfile stabilization = StabilizationProfile::Law42MaterialSoundSpeedV1;
  double damping_coefficient = .1; // Native GEO13 (DN), dimensionless.
  int engine_frame = 1;
  int integration_control = 2;
  int compressibility_control = 1;
  int degenerate_step_control = 0; // Native default /DTSDE/OFF.
};
struct HistoryStamp {
  double time_s = 0;
  std::uint64_t sample_index = 0; // Caller label, not an owner clock.
};
struct PrescribedInterval {
  Vec3 position_endpoint_m[6]{}; // Original six source slots, world coordinates.
  Vec3 velocity_midpoint_m_s[6]{};
  double base_time_s = 0;
  double dt_s = 0;
  std::uint64_t sample_index = 0;
};
struct HistoryValues {
  // NPTTOT1 aliases GBUF and LBUF. Material and stabilization advance this
  // same accepted EINT; there is no independent global-energy history.
  tl::material::law42::CallerHistory material;
  double hourglass_stress_pa[3][4]{}; // Native FHOUR(component,mode).
};
class History {
 public:
  TL_BRICK_HD bool prepared() const noexcept { return prepared_; }
  TL_BRICK_HD const HistoryValues& data() const noexcept { return data_; }
  TL_BRICK_HD const Reference& reference() const noexcept { return reference_; }
  TL_BRICK_HD const Material& material() const noexcept { return material_; }
  TL_BRICK_HD const ForceProfile& profile() const noexcept { return profile_; }
  TL_BRICK_HD HistoryStamp stamp() const noexcept { return stamp_; }
 private:
  HistoryValues data_{};
  Reference reference_{};
  Material material_{};
  ForceProfile profile_{};
  HistoryStamp stamp_{};
  bool prepared_ = false;
  friend TL_BRICK_HD Status PreparePrescribedHistory(const Reference&, const Material&,
      const ForceProfile&, const HistoryValues&, HistoryStamp, History&) noexcept;
};
struct CurrentGeometry {
  Matrix3 frame;
  Vec3 local_position_m[6]{};
  Vec3 local_velocity_m_s[6]{};
  double point_gradient_per_m[3][6]{};
  double world_displacement_gradient[9]{};
  double material_displacement_gradient[9]{};
  double velocity_gradient_per_s[9]{};
  double engineering_rate_per_s[6]{};
  double current_volume_m3 = 0;
  double characteristic_length_m = 0;
};
struct HourglassObservation {
  double modal_velocity_m_s[3][4]{};
  double modal_force_n[3][4]{}; // After metric mapping, before node projection.
  double effective_shear_modulus_pa = 0;
  double damping_kg_m_s = 0; // kg/(m*s); multiplied by modal velocity then H(m).
  double first_work_j = 0;
  double second_work_j = 0;
};
struct ForceTrial {
  History proposed_history;
  CurrentGeometry geometry;
  tl::material::law42::CallerResult material;
  HourglassObservation stabilization;
  Vec3 material_local_force_n[6]{};
  Vec3 stabilized_local_force_n[6]{};
  Vec3 rhs_force_n[6]{}; // Negative internal force in original source slots.
  double total_internal_work_increment_j = 0;
};
} // namespace tl::fea::solid6z
