// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Source and supported branch: TabulatedShellPlasticity.md.
#pragma once
#include "lib_src/math/Quaternion.h"
#include <cstdint>

#if defined(__CUDACC__)
#define TL_TABULATED_SHELL_HD __host__ __device__
#else
#define TL_TABULATED_SHELL_HD
#endif

namespace tl::material {
// Immutable storage belongs to the caller and must be accessible on the
// executing host/device. Prepare validates it once; it must then stay unchanged.
struct TabulatedShellPlasticityCurve {
  const double* plastic_strain = nullptr;
  const double* yield_stress_pa = nullptr;
  std::uint32_t count = 0;
};
struct TabulatedShellPlasticityParameters {
  TabulatedShellPlasticityCurve curve{};
  double young_pa = 0, poisson_ratio = 0, density_kg_m3 = 0;
  double shear_modulus = 0, a11 = 0, a12 = 0, three_g = 0;
  double sound_speed = 0;
};
struct TabulatedShellPlasticityHistory {
  double stress[5]{}; // XX, YY, XY, YZ, ZX, Pa; transverse shear stays elastic.
  double plastic_strain = 0; // Accumulated equivalent plastic strain.
};
struct TabulatedShellPlasticityInput {
  double strain_increment[5]{}; // XX, YY, engineering XY, YZ, ZX.
  double transverse_shear_modulus = 0; // Actual element/section GS, Pa.
};
struct TabulatedShellPlasticityResult {
  TabulatedShellPlasticityHistory history{};
  double plastic_increment = 0;
  double tangent_ratio = 1; // Native ETSE; not a consistent tangent matrix.
  double elastic_thickness_strain = 0;
  double plastic_thickness_strain = 0;
  double yield_before_pa = 0;
  double equivalent_stress_pa = 0;
  // Native MULAWC diagnostic: .5*(old/new equivalent stress)*delta PLA.
  // J/m3. This is not total stress work or a complete energy decomposition.
  double plastic_work_density = 0;
};
enum class TabulatedShellPlasticityStatus : std::uint8_t {
  Ok, InvalidParameters, InvalidCurve, InvalidHistory, InvalidIncrement,
  CurveDomainExceeded, NonfiniteResult
};

TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
PrepareTabulatedShellPlasticity(double young, double nu, double rho,
    TabulatedShellPlasticityCurve curve, TabulatedShellPlasticityParameters& output) noexcept;

// Pure trial update. Rate dependence, failure, kinematic hardening and nonlocal
// corrections are explicitly absent. Accepted history/output remain untouched
// on failure. No owner, allocation, time integration or stress-work ledger.
TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
UpdateTabulatedShellPlasticity(const TabulatedShellPlasticityParameters& parameters,
    const TabulatedShellPlasticityHistory& accepted,
    const TabulatedShellPlasticityInput& input,
    TabulatedShellPlasticityResult& output) noexcept;
} // namespace tl::material

#include "lib_src/materials/detail/TabulatedShellPlasticityCurve.h"
#include "lib_src/materials/detail/TabulatedShellPlasticityUpdate.h"
#undef TL_TABULATED_SHELL_HD
