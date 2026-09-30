#pragma once
#include "lib_src/materials/law90/Prepare.h"
#include "lib_src/materials/law90/Curve.h"
#include "source_fixture/OriginalMaterial.h"
#include "source_fixture/NativeSdiOriginal.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace law90_test {
namespace law = tl::material::law90;
inline law::PreparationInput OriginalInput() {
  law::PreparationInput input;
  input.density_kg_m3 = original_radiator::density_kg_m3;
  input.card_young_pa = 21e6;
  input.contact_modulus_pa = 20000e6;
  input.tension_cutoff_pa = 15e6;
  return input;
}
inline law::CurveView OriginalCurve() {
  return {original_radiator::strain, original_radiator::stress_pa,
          original_radiator::point_count};
}
// Actual direct/export/re-read SDI receipt: blank HU enters HM_READ_MAT90 as0.
// The legacy OriginalInput helper retains its already-qualified explicit-HU1 case.
inline law::PreparationInput OriginalBlankHuInput() {
  auto input = OriginalInput();
  input.hysteresis = 0;
  input.density_kg_m3 = original_radiator_sdi::density_kg_m3;
  input.curve_scale_dimension = 1e6;
  return input;
}
inline law::CurveView OriginalBlankHuCurve() {
  return {original_radiator::strain, original_radiator::stress_mpa,
          original_radiator::point_count};
}
inline std::array<double, 13> InputValues(const law::PreparationInput& p) {
  return {p.density_kg_m3, p.reference_density_kg_m3, p.card_young_pa,
          p.poisson_ratio, p.contact_modulus_pa, p.tension_cutoff_pa,
          p.hysteresis, p.shape, p.alpha, p.curve_scale,
          p.curve_scale_dimension, p.curve_rate_s_inverse, p.filter_cutoff_hz};
}
inline std::array<int, 3> InputFlags(const law::PreparationInput& p) {
  return {p.smooth, p.tension_flag, p.failure_mode};
}
TL_LAW90_HD inline void Pack(const law::PreparedMaterial& p, double* values) {
  const auto& r = p.reader();
  const auto& u = p.updated();
  const double next[] = {
      r.density_kg_m3, r.reference_density_kg_m3, r.card_young_pa,
      r.initial_shear_pa, r.poisson_ratio, r.contact_modulus_pa,
      r.contact_bulk_pa, r.tension_cutoff_pa, r.hysteresis, r.shape,
      r.alpha, r.curve_scale, r.curve_rate_s_inverse, r.filter_cutoff_hz,
      double(r.smooth), double(r.rate_flag), double(r.loading_flag),
      double(r.damage_flag), double(r.tension_flag), double(r.failure_mode),
      double(r.material_viscosity_flag), double(r.history_count), double(r.cursor_count),
      u.minimum_curve_slope_pa, u.maximum_curve_slope_pa, u.initial_curve_slope_pa,
      u.average_curve_slope_pa, u.young_pa, u.shear_pa, u.bulk_pa,
      u.maximum_modulus_pa, u.hourglass_modulus_pa, u.maximum_strain};
  for (unsigned i = 0; i < 33; ++i) {
    values[i] = next[i];
  }
}
inline bool SameBits(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}
template<class T>
std::array<unsigned char, sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  return bytes;
}
inline bool Close(double a, double b, double tolerance = 2e-14) {
  return std::isfinite(a) && std::isfinite(b) &&
         std::abs(a - b) <= tolerance * std::fmax(1.0, std::fmax(std::abs(a), std::abs(b)));
}
}  // namespace law90_test
