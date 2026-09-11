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
// Explicit LAW44 VP=2, ISRATE=1 branch: filtered shell total strain rate.
// Disabled declarations have zero scalar values; no hidden cutoff default.
struct TabulatedShellPlasticityRate {
  bool enabled = false;
  double cowper_symonds_c_per_s = 0, cowper_symonds_p = 0, cutoff_hz = 0;
};
enum class ShellPlasticityHardeningKind : std::uint8_t { Tabulated, LinearLaw44 };
// StrictDomain preserves historical admission. NativeLastSegment uses the
// pinned VINTER final-segment extrapolation below the separate native cap.
enum class ShellPlasticityCurveContinuation : std::uint8_t {
  StrictDomain, NativeLastSegment
};
// Original MAT024 LCSS=0, blank inline points. ETAN is the uniaxial tangent,
// not the native plastic hardening modulus. Only positive-C/P VP2 is admitted.
struct Law44LinearHardening {
  double initial_yield_pa = 0, tangent_modulus_pa = 0;
};
struct TabulatedShellPlasticityParameters {
  TabulatedShellPlasticityCurve curve{};
  double young_pa = 0, poisson_ratio = 0, density_kg_m3 = 0;
  double shear_modulus = 0, a11 = 0, a12 = 0, three_g = 0;
  double sound_speed = 0;
  TabulatedShellPlasticityRate rate{};
  double inverse_rate_c = 0, inverse_rate_p = 0, angular_cutoff_per_s = 0;
  // Trailing fields retain prior positional aggregate initialization. Existing
  // tabulated preparation leaves these zero/default; no history layout changes.
  ShellPlasticityHardeningKind hardening = ShellPlasticityHardeningKind::Tabulated;
  Law44LinearHardening linear{};
  double plastic_hardening_pa = 0;
  ShellPlasticityCurveContinuation continuation = ShellPlasticityCurveContinuation::StrictDomain;
};
struct TabulatedShellPlasticityHistory {
  double stress[5]{}; // XX, YY, XY, YZ, ZX, Pa; transverse shear stays elastic.
  double plastic_strain = 0; // Accumulated equivalent plastic strain.
  double filtered_rate_per_s = 0; // Native UVAR1, independently owned by each point.
};
struct TabulatedShellPlasticityInput {
  double strain_increment[5]{}; // XX, YY, engineering XY, YZ, ZX.
  double transverse_shear_modulus = 0; // Actual element/section GS, Pa.
  double dt = 0, total_strain_rate_per_s = 0; // Required only when rate.enabled.
  // Native parent OFF==1. Inactive parents still evaluate predictor/rate fields;
  // only plastic gather and local thickness additions are disabled.
  bool element_active = true;
};
struct TabulatedShellPlasticityResult {
  TabulatedShellPlasticityHistory history{};
  double plastic_increment = 0;
  double tangent_ratio = 1; // Native ETSE; not a consistent tangent matrix.
  double elastic_thickness_strain = 0;
  double plastic_thickness_strain = 0;
  double yield_before_pa = 0;
  double equivalent_stress_pa = 0;
  // Native MULAWC diagnostic: .5*(old/new equivalent stress)*rounded delta PLA.
  // The accumulated-PLA subtraction can differ from plastic_increment.
  // J/m3. This is not total stress work or a complete energy decomposition.
  double plastic_work_density = 0;
};
enum class TabulatedShellPlasticityStatus : std::uint8_t {
  Ok, InvalidParameters, InvalidCurve, InvalidHistory, InvalidIncrement,
  CurveDomainExceeded, NonfiniteResult, HardeningDomainExceeded
};

TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
PrepareTabulatedShellPlasticity(double young, double nu, double rho,
    TabulatedShellPlasticityCurve curve, TabulatedShellPlasticityParameters& output) noexcept;
TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
PrepareTabulatedShellPlasticity(double young, double nu, double rho,
    TabulatedShellPlasticityCurve curve, TabulatedShellPlasticityRate rate,
    TabulatedShellPlasticityParameters& output) noexcept;
TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
PrepareTabulatedShellPlasticity(double young, double nu, double rho,
    TabulatedShellPlasticityCurve curve, TabulatedShellPlasticityRate rate,
    ShellPlasticityCurveContinuation continuation,
    TabulatedShellPlasticityParameters& output) noexcept;
TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
PrepareLinearLaw44ShellPlasticity(double young, double nu, double rho,
    Law44LinearHardening linear, TabulatedShellPlasticityRate rate,
    TabulatedShellPlasticityParameters& output) noexcept;

// Common LAW44 recurrence, with independently selected immutable hardening.
TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
UpdateLaw44ShellPlasticity(const TabulatedShellPlasticityParameters& parameters,
    const TabulatedShellPlasticityHistory& accepted,
    const TabulatedShellPlasticityInput& input,
    TabulatedShellPlasticityResult& output) noexcept;

// Pure trial update. Failure, kinematic hardening and nonlocal corrections are
// explicitly absent. Accepted history/output remain untouched
// on failure. No owner, allocation, time integration or stress-work ledger.
TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus
UpdateTabulatedShellPlasticity(const TabulatedShellPlasticityParameters& parameters,
    const TabulatedShellPlasticityHistory& accepted,
    const TabulatedShellPlasticityInput& input,
    TabulatedShellPlasticityResult& output) noexcept;
} // namespace tl::material

#include "lib_src/materials/detail/TabulatedShellPlasticityRate.h"
#include "lib_src/materials/detail/Law44ElasticParameters.h"
#include "lib_src/materials/detail/TabulatedShellPlasticityCurve.h"
#include "lib_src/materials/detail/Law44Hardening.h"
#include "lib_src/materials/detail/TabulatedShellPlasticityUpdate.h"
#undef TL_TABULATED_SHELL_HD
