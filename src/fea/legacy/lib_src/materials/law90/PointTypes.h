// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"

namespace tl::material::law90 {
enum class PointStatus : std::uint8_t {
  Ok, InvalidMaterial, InvalidInput, InvalidHistory, InvalidStretch,
  InvalidCurve, InvalidCursor, NonfiniteResult
};
// All ten native UVAR entries remain distinct, in native order. These Pa path
// quantities are material algebraic histories, not the caller's physical EINT.
struct PointHistory {
  double stress_norm_pa = 0;                       // 1: precap YLD
  double maximum_path_energy_pa = 0;               // 2
  double scalar_rate_s_inverse = 0;                // 3
  double path_energy_pa = 0;                       // 4
  double reserved5 = 0;                            // 5: carried, not interpreted
  double strain_norm = 0;                         // 6
  double unloading_factor = 0;                     // 7
  double effective_modulus_pa = 0;                 // 8
  double instantaneous_quasistatic_energy_pa = 0;  // 9
  double residual_strain = 0;                     // 10
  std::uint32_t cursor[3]{}; // Native principal order; not fixed spatial axes.
};
struct PointKinematics {
  // XX/YY/ZZ/XY/YZ/ZX in the actual material frame. Off-diagonals are doubled.
  double total_b_minus_i_engineering[6]{};
  double engineering_rate_s_inverse[6]{};
};
struct PointResult {
  PointHistory history{};
  double cauchy_stress_pa[6]{};
  double sound_speed_m_s = 0;
  double scalar_rate_s_inverse = 0;
  double tangent_factor = 0;          // Actual ET=min(queried scaled slopes)/E0.
  double maximum_viscosity_pa_s = 0; // Selected VISCMAX0.
  double active = 1;                 // Selected FAIL0, always active.
};
} // namespace tl::material::law90
