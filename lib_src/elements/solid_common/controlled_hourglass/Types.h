// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SHOUR_CTL, OpenRadioss (C) 2026 Siemens, revision a62b27e6.
#pragma once
#include "lib_src/elements/solid_common/BrickFrame.h"

namespace tl::fea::solid_common::controlled_hourglass {
// SI, one-term alpha=2/no-Prony LAW42, OFF=1, DN=0.1. The family caller
// supplies native eight-slot local velocities and SZDERI/S6CHOUR projection.
// This leaf does not select a family profile or implement distortion control.
inline constexpr double MaximumPoissonRatio = static_cast<double>(0.48999f);
enum class Status { Success, InvalidInput, UnsupportedProfile, NonfiniteResult };
struct State {
  double force_n[3][4]{}; // Native controlled FHOUR, NOT ordinary Pa history.
};
struct Input {
  double mu_pa=0, poisson_ratio=0;
  double density_kg_m3=0, material_sound_speed_m_s=0;
  double dt_s=0, current_volume_m3=0, reference_volume_m3=0;
  Vec3 local_velocity_m_s[8]{};
  double projection[4][3]{}; // [native PX1..4][H1..3], dimensionless.
  Vec3 incoming_local_force_n[8]{};
  double internal_energy_density_j_m3=0, raw_stiffness_n_m=0;
};
struct Result {
  State proposed_state;
  Vec3 local_force_n[8]{};
  double internal_energy_density_j_m3=0, raw_stiffness_n_m=0;
  double work_j=0; // Actual dt * native modal power, before /reference volume.
  double modal_velocity_m_s[3][4]{}, modal_force_n[3][4]{};
};
} // namespace tl::fea::solid_common::controlled_hourglass
