// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/math/Quaternion.h"
#include <cstdint>

#if defined(__CUDACC__)
#define TL_LAW90_HD __host__ __device__
#else
#define TL_LAW90_HD
#endif

namespace tl::material::law90 {
enum class Status : std::uint8_t {
  Ok, InvalidInput, UnsupportedProfile, InvalidCurve, InvalidCursor,
  NonfiniteResult
};

// Borrowed immutable arrays, including the original zero ordinate. No ownership
// or source identity is implied. Storage must outlive every prepared value use.
struct CurveView {
  const double* compression_strain = nullptr;
  // Native base ordinates. Historical member name is retained for source
  // compatibility: effective stress in Pa is reader.curve_scale * stress_pa[i].
  // Either Pa ordinates/scale1 or raw working ordinates/dimensioned scale are
  // explicit valid representations; never pre-scale and scale again.
  const double* stress_pa = nullptr;
  std::uint32_t count = 0;
};

// SI quantities at HM_READ_MAT90's post-HM_GET boundary. This is not an LS
// parser. Explicit zeros retain the selected native reader default semantics.
struct PreparationInput {
  double density_kg_m3 = 0;
  double reference_density_kg_m3 = 0;
  double card_young_pa = 0;
  double poisson_ratio = 0;
  double contact_modulus_pa = 0;
  double tension_cutoff_pa = 0;
  double hysteresis = 1;
  double shape = 0;
  double alpha = 0;
  double curve_scale = 0;
  double curve_scale_dimension = 1; // Pa per base ordinate unit when scale is blank/zero.
  double curve_rate_s_inverse = 0;
  double filter_cutoff_hz = 0;
  int smooth = 1;
  int tension_flag = 2;
  int failure_mode = 0;
};

struct ReaderValues {
  double density_kg_m3 = 0;           // PM89
  double reference_density_kg_m3 = 0; // PM1
  double card_young_pa = 0;           // UPARAM1 before LAW90_UPD
  double initial_shear_pa = 0;        // UPARAM4 remains the pre-update value
  double poisson_ratio = 0;
  double contact_modulus_pa = 0;      // PARMAT2 and UPARAM19, not PM20
  double contact_bulk_pa = 0;         // PARMAT1
  double tension_cutoff_pa = 0;       // UPARAM18
  double hysteresis = 0;
  double shape = 0;
  double alpha = 0;
  double curve_scale = 0;
  double curve_rate_s_inverse = 0;
  double filter_cutoff_hz = 0;
  int smooth = 0;
  int rate_flag = 0;
  int loading_flag = 0;
  int damage_flag = 0;
  int tension_flag = 0;
  int failure_mode = 0;
  int material_viscosity_flag = 0;
  int history_count = 0;
  int cursor_count = 0;
};

struct UpdatedValues {
  double minimum_curve_slope_pa = 0;
  double maximum_curve_slope_pa = 0;
  double initial_curve_slope_pa = 0;  // also LAW90_UPD's YOUNG0
  double average_curve_slope_pa = 0;
  double young_pa = 0;               // updated UPARAM1 and PM20
  double shear_pa = 0;               // PM22, distinct from UPARAM4
  double bulk_pa = 0;                // PM32
  double maximum_modulus_pa = 0;     // UPARAM13
  double hourglass_modulus_pa = 0;   // PM24
  double maximum_strain = 0;         // UPARAM14
};

class PreparedMaterial {
 public:
  TL_LAW90_HD bool initialized() const noexcept { return initialized_; }
  TL_LAW90_HD CurveView curve() const noexcept { return curve_; }
  TL_LAW90_HD const ReaderValues& reader() const noexcept { return reader_; }
  TL_LAW90_HD const UpdatedValues& updated() const noexcept { return updated_; }
 private:
  CurveView curve_{};
  ReaderValues reader_{};
  UpdatedValues updated_{};
  bool initialized_ = false;
  friend TL_LAW90_HD Status RelocatePreparedCurve(
      const PreparedMaterial&, CurveView, PreparedMaterial&) noexcept;
  friend TL_LAW90_HD Status PrepareSI(
      const PreparationInput&, CurveView, PreparedMaterial&) noexcept;
};

struct CurveResult {
  // Unscaled native base value/slope; SIGEPS90 applies reader.curve_scale later.
  double stress_pa = 0;
  double slope_pa = 0;
  std::uint32_t cursor = 0;
};
}  // namespace tl::material::law90
