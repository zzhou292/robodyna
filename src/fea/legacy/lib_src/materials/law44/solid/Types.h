// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SIGEPS44/MULAW values, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "lib_src/math/Quaternion.h"
#include <cstdint>

#if defined(__CUDACC__)
#define TL_LAW44_SOLID_HD __host__ __device__
#else
#define TL_LAW44_SOLID_HD
#endif

namespace tl::material::law44::solid {
// Controls conversion of the native finite stress cap and EM20 stress floors.
// All public dimensional values are SI, including the borrowed curve.
enum class WorkingUnits : std::uint8_t { SI, TonneMillimetreSecond };
enum class HardeningKind : std::uint8_t { Tabulated, Analytic };
struct AnalyticHardening {
  double a_pa = 0;
  double b_pa = 0;
  double exponent = 0;
  // Zero selects the actual finite native reader default, not IEEE infinity.
  double maximum_stress_pa = 0;
  double maximum_plastic_strain = 0;
};
struct Curve {
  const double* plastic_strain = nullptr;
  const double* yield_stress_pa = nullptr;
  std::uint32_t count = 0;
};
struct Material {
  double young_pa = 0;
  double poisson_ratio = 0;
  double density_kg_m3 = 0;
  double rate_c_per_s = 0;
  double rate_p = 0;
  double cutoff_hz = 0;
  WorkingUnits native_units = WorkingUnits::SI;
  HardeningKind hardening = HardeningKind::Tabulated;
  AnalyticHardening analytic{};
};
struct Parameters;
namespace detail {
TL_LAW44_SOLID_HD bool HardeningCoefficients(Material, double, Parameters&, const Parameters*) noexcept;
}
// The prepared EPSGM is an immutable value, including its host libm rounding.
// Device admission checks this exact receipt instead of recalculating pow with
// a different libm. Ordinary public field edits cannot update the receipt.
class AnalyticPreparation {
 public:
  TL_LAW44_SOLID_HD bool initialized() const noexcept { return initialized_; }
  TL_LAW44_SOLID_HD bool Matches(const Material& material, double cap) const noexcept {
    if (!initialized_ || material.hardening != HardeningKind::Analytic ||
        material.native_units != units_) return false;
    const auto& a = material.analytic;
    return Same(a.a_pa, hardening_.a_pa) && Same(a.b_pa, hardening_.b_pa) &&
        Same(a.exponent, hardening_.exponent) && Same(a.maximum_stress_pa, hardening_.maximum_stress_pa) &&
        Same(a.maximum_plastic_strain, hardening_.maximum_plastic_strain) && Same(cap, plastic_cap_);
  }
 private:
  TL_LAW44_SOLID_HD static bool Same(double a, double b) noexcept {
    const auto* x = reinterpret_cast<const unsigned char*>(&a);
    const auto* y = reinterpret_cast<const unsigned char*>(&b);
    for (unsigned i = 0; i < sizeof(double); ++i) if (x[i] != y[i]) return false;
    return true;
  }
  AnalyticHardening hardening_{};
  double plastic_cap_ = 0;
  WorkingUnits units_ = WorkingUnits::SI;
  bool initialized_ = false;
  friend TL_LAW44_SOLID_HD bool detail::HardeningCoefficients(
      Material, double, Parameters&, const Parameters*) noexcept;
};
struct Parameters {
  Material material{};
  Curve curve{};  // Immutable backing belongs to the caller.
  double shear_pa = 0;
  double twice_shear_pa = 0;
  double three_shear_pa = 0;
  double bulk_pa = 0;
  double sound_speed_m_s = 0;
  double inverse_rate_c = 0;
  double inverse_rate_p = 0;
  double angular_cutoff_per_s = 0;
  double stress_limit_pa = 0;
  double stress_floor_pa = 0;
  double plastic_cap_strain = 0;       // Native EPSGM; distinct from EPMAX.
  double failure_plastic_strain = 0;   // Native EPMAX.
  AnalyticPreparation analytic_preparation{};
};
struct History {
  // Current native material frame: XX, YY, ZZ, XY, YZ, ZX.
  double stress_pa[6]{};
  double engineering_strain[6]{};
  double plastic_strain = 0;
  double filtered_rate_per_s = 0;
  std::uint32_t curve_cursor = 0;  // Zero-based retained segment; TF offsets are external.
};
struct Input {
  double engineering_rate_per_s[6]{};
  double dt_s = 0;
  double relative_density = 0;  // Actual MMAIN AMU, supplied independently.
};
struct Result {
  History history{};
  double plastic_increment = 0;
  double yield_stress_pa = 0;
  double sound_speed_m_s = 0;
  double material_viscosity_pa_s = 0;
  double tangent_factor = 0;  // Actual SIGEPS44 ET, not a recomputed secant.
};
enum class Status : std::uint8_t {
  Ok, InvalidParameters, InvalidCurve, InvalidHistory, InvalidInput,
  NonfiniteResult
};
}  // namespace tl::material::law44::solid
