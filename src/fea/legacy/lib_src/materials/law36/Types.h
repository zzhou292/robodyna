// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected LAW36 solid branch adapted from OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "lib_src/math/Quaternion.h"
#include <cstdint>
#if defined(__CUDACC__)
#define TL_LAW36_HD __host__ __device__
#else
#define TL_LAW36_HD
#endif
namespace tl::material::law36 {
// Borrowed immutable host/device arrays. Preparation validates 2..1024 points.
struct Curve {
  const double* plastic_strain = nullptr;
  const double* yield_stress_pa = nullptr;
  std::uint32_t count = 0;
};
struct Parameters {
  Curve curve{};
  double young_pa = 0;
  double poisson_ratio = 0;
  double density_kg_m3 = 0;
  double shear_pa = 0;
  double twice_shear_pa = 0;
  double three_shear_pa = 0;
  double bulk_pa = 0;
  double sound_speed_m_s = 0;
};
struct History {
  // Current native material frame: XX, YY, ZZ, XY, YZ, ZX.
  double stress_pa[6]{};
  double engineering_strain[6]{};
  double plastic_strain = 0;
  double deviatoric_rate_per_s = 0;  // Native output, unfiltered for NRATE1.
};
struct Kinematics {
  double engineering_rate_per_s[6]{};
  double dt_s = 0;  // Zero is the native startup force packet.
};
struct Input {
  Kinematics kinematics{};
  double relative_density = 0;  // Actual MMAIN AMU; not trace(strain).
};
struct Result {
  History history{};
  double plastic_increment = 0;  // Actual SIGEPS36 DPLA1.
  double yield_stress_pa = 0;     // Actual updated native YLD.
  double equivalent_stress_pa = 0;
  double sound_speed_m_s = 0;
  double material_viscosity_pa_s = 0;
  // MULAW uses the rounded subtraction PLAnew-PLAold, separately from DPLA1.
  double plastic_work_density_j_m3 = 0;
};
struct CallerHistory {
  History point{};
  double internal_energy_density_j_m3 = 0;  // Native stored EINT.
  double plastic_work_j = 0;                // Native accumulated point WPLA.
};
struct Measures {
  // Already computed by the native solid geometry/density/viscosity caller.
  // They are distinct quantities for selective pressure; no geometry is inferred.
  double density_kg_m3 = 0;
  double storage_volume_m3 = 0;
  double current_volume_m3 = 0;
  double volume_increment_m3 = 0;
  double old_bulk_pressure_pa = 0;
  double new_bulk_pressure_pa = 0;
};
struct CallerResult {
  CallerHistory history{};
  Result point{};
  double relative_density = 0;
  double average_volume_m3 = 0;
  double internal_work_j = 0;
  double plastic_work_increment_j = 0;
};
enum class Status : std::uint8_t {
  Ok, InvalidParameters, InvalidCurve, InvalidHistory, InvalidInput,
  SentinelDomainExceeded, NonfiniteResult
};
// No kinematic/rate-dependent hardening, damage, EOS, thermal, nonlocal, or
// inactive element branch is admitted. Owner/source admission is separate.
}  // namespace tl::material::law36
