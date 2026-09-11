// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SIGEPS42 branch, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "lib_src/math/Quaternion.h"
#include <cstdint>
#if defined(__CUDACC__)
#define TL_LAW42_HD __host__ __device__
#else
#define TL_LAW42_HD
#endif
namespace tl::material::law42 {
// One-term Ogden alpha=2, no Prony/bulk curve/thermal strain; IFORM1, ISMSTR10.
// This is the original MAT007 converter material profile, not a generic parser.
struct Parameters {
  double mu_pa=0;
  double poisson_ratio=0;
  double bulk_pa=0;
  double density_kg_m3=0;
  double tension_cutoff_pa=0;
};
struct Input {
  // Exact SIGEPS42 EPSXX/YY/ZZ/XY/YZ/ZX caller measures. Off-diagonals
  // are halved before diagonalization. Eigenvalues+1 are stretch squared.
  // The element caller must supply these measures in its actual material frame.
  double total_strain[6]{};
  double density_kg_m3=0;
  double active=1; // Material OFF; parent OFFG=1, no small-strain fallback.
};
struct Result {
  double stress_pa[6]{};
  double maximum_principal_stress_pa=0;
  double minimum_principal_stress_pa=0;
  double active=1;
  double relative_volume=0;
  double sound_speed_m_s=0;
  double hourglass_tangent_factor=0; // Native ET output, required by HEPH.
  double material_viscosity_pa_s=0;
};
enum class Status : std::uint8_t {
  Ok, InvalidParameters, InvalidInput, InvalidStretch, NonfiniteResult
};
} // namespace tl::material::law42
