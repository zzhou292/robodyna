// SPDX-License-Identifier: AGPL-3.0-or-later
// HM_READ_MAT90, FUNC_SLOPE and LAW90_UPD, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "lib_src/materials/law90/Types.h"

namespace tl::material::law90 {
namespace detail {
TL_LAW90_HD inline bool Positive(double value) noexcept {
  return tl::math::Finite(value) && value > 0;
}
TL_LAW90_HD inline double Maximum(double a, double b) noexcept {
  return a < b ? b : a;
}
TL_LAW90_HD inline double Minimum(double a, double b) noexcept {
  return b < a ? b : a;
}

TL_LAW90_HD inline Status ReadSelectedSI(
    const PreparationInput& input, ReaderValues& output) noexcept {
  const double fields[] = {
      input.density_kg_m3, input.reference_density_kg_m3, input.card_young_pa,
      input.poisson_ratio, input.contact_modulus_pa, input.tension_cutoff_pa,
      input.hysteresis, input.shape, input.alpha, input.curve_scale,
      input.curve_scale_dimension, input.curve_rate_s_inverse,
      input.filter_cutoff_hz};
  for (double value : fields) {
    if (!tl::math::Finite(value)) {
      return Status::InvalidInput;
    }
  }
  if (!Positive(input.density_kg_m3) || !Positive(input.card_young_pa) ||
      !Positive(input.curve_scale_dimension) ||
      input.reference_density_kg_m3 < 0 || input.contact_modulus_pa < 0 ||
      input.curve_scale < 0) {
    return Status::InvalidInput;
  }

  ReaderValues next;
  next.density_kg_m3 = input.density_kg_m3;
  next.reference_density_kg_m3 = input.reference_density_kg_m3;
  next.card_young_pa = input.card_young_pa;
  next.poisson_ratio = input.poisson_ratio;
  next.contact_modulus_pa = input.contact_modulus_pa;
  next.tension_cutoff_pa = input.tension_cutoff_pa;
  next.tension_flag = input.tension_flag;
  next.failure_mode = input.failure_mode;
  next.curve_rate_s_inverse = input.curve_rate_s_inverse;
  next.filter_cutoff_hz = input.filter_cutoff_hz;
  next.material_viscosity_flag = 2;
  if (next.tension_flag == 0) {
    next.tension_flag = 1;
  }
  if (next.contact_modulus_pa == 0) {
    next.contact_modulus_pa = next.card_young_pa;
  }
  if (next.tension_cutoff_pa <= 0) {
    next.tension_cutoff_pa = 1e20;  // CONSTANT_MOD EP20, finite native SI sentinel.
  }
  next.curve_scale = input.curve_scale;
  if (next.curve_scale == 0) {
    next.curve_scale = input.curve_scale_dimension * 1.0;
  }
  next.hysteresis = ::fabs(input.hysteresis);
  next.loading_flag = next.hysteresis == 0 ? 1 : 2;
  next.shape = input.shape == 0 ? 1.0 : input.shape;
  next.alpha = input.alpha == 0 ? 1.0 : input.alpha;
  if (next.hysteresis == 0) {
    next.hysteresis = 1;
  }
  if (next.reference_density_kg_m3 == 0) {
    next.reference_density_kg_m3 = next.density_kg_m3;
  }
  // NFUNC1 forces ISMOOTH0 and hence ISRATE0, irrespective of input Ismooth.
  next.smooth = 0;
  next.rate_flag = 0;
  if (next.poisson_ratio != 0 || next.hysteresis != 1 || next.shape != 1 ||
      next.alpha != 1 || (next.loading_flag != 1 && next.loading_flag != 2) ||
      next.tension_flag != 2 ||
      next.failure_mode != 0 || next.curve_rate_s_inverse != 0 ||
      next.filter_cutoff_hz != 0 || (input.smooth != 0 && input.smooth != 1)) {
    return Status::UnsupportedProfile;
  }

  next.initial_shear_pa = .5 * next.card_young_pa / (1 + next.poisson_ratio);
  next.contact_bulk_pa = next.contact_modulus_pa / 3 / (1 - 2 * next.poisson_ratio);
  next.history_count = 10;
  next.cursor_count = 3;
  if (!Positive(next.initial_shear_pa) || !Positive(next.contact_bulk_pa) ||
      !Positive(next.curve_scale)) {
    return Status::NonfiniteResult;
  }
  output = next;
  return Status::Ok;
}

TL_LAW90_HD inline Status CurveSlopes(
    CurveView curve, double factor, UpdatedValues& output) noexcept {
  if (!curve.compression_strain || !curve.stress_pa || curve.count < 2 ||
      curve.count > 1024 || curve.compression_strain[0] != 0) {
    return Status::InvalidCurve;
  }
  for (std::uint32_t i = 0; i < curve.count; ++i) {
    if (!tl::math::Finite(curve.compression_strain[i]) ||
        !tl::math::Finite(curve.stress_pa[i]) || curve.stress_pa[i] < 0 ||
        (i && (curve.compression_strain[i] <= curve.compression_strain[i - 1] ||
               curve.stress_pa[i] < curve.stress_pa[i - 1]))) {
      return Status::InvalidCurve;
    }
  }
  UpdatedValues next;
  next.minimum_curve_slope_pa = 1e20;
  for (std::uint32_t i = 0; i + 1 < curve.count; ++i) {
    const double dx = curve.compression_strain[i + 1] - curve.compression_strain[i];
    const double dy = curve.stress_pa[i + 1] - curve.stress_pa[i];
    const double slope = factor * dy / dx;
    if (!tl::math::Finite(slope)) {
      return Status::InvalidCurve;
    }
    next.maximum_curve_slope_pa = Maximum(next.maximum_curve_slope_pa, slope);
    next.minimum_curve_slope_pa = Minimum(next.minimum_curve_slope_pa, slope);
    next.average_curve_slope_pa = next.average_curve_slope_pa + slope;
    if (curve.compression_strain[i + 1] == 0 || curve.compression_strain[i] == 0) {
      next.initial_curve_slope_pa = Maximum(next.initial_curve_slope_pa, factor * dy / dx);
    } else if (curve.compression_strain[0] >= 0) {
      const double initial_dx = curve.compression_strain[1] - curve.compression_strain[0];
      const double initial_dy = curve.stress_pa[1] - curve.stress_pa[0];
      next.initial_curve_slope_pa = Maximum(
          next.initial_curve_slope_pa, factor * initial_dy / initial_dx);
    }
  }
  next.average_curve_slope_pa = next.average_curve_slope_pa / (curve.count - 1);
  if (!tl::math::Finite(next.average_curve_slope_pa)) {
    return Status::InvalidCurve;
  }
  output = next;
  return Status::Ok;
}
}  // namespace detail

// Both output and its borrowed backing remain unchanged on rejection. The
// caller must keep that backing immutable and alive; this value owns no source.
TL_LAW90_HD inline Status PrepareSI(
    const PreparationInput& input, CurveView curve,
    PreparedMaterial& output) noexcept {
  PreparedMaterial next;
  Status status = detail::ReadSelectedSI(input, next.reader_);
  if (status != Status::Ok) {
    return status;
  }
  status = detail::CurveSlopes(curve, next.reader_.curve_scale, next.updated_);
  if (status != Status::Ok) {
    return status;
  }
  UpdatedValues& values = next.updated_;
  values.young_pa = next.reader_.card_young_pa;
  if (values.young_pa < values.initial_curve_slope_pa) {
    values.young_pa = values.initial_curve_slope_pa;
  }
  if (values.maximum_curve_slope_pa <= values.young_pa) {
    values.maximum_modulus_pa = values.young_pa;
  } else {
    values.maximum_modulus_pa = detail::Minimum(
        values.maximum_curve_slope_pa, 100 * values.young_pa);
  }
  values.hourglass_modulus_pa = detail::Maximum(values.young_pa, values.maximum_modulus_pa);
  values.maximum_strain = 1;
  values.bulk_pa = values.young_pa / 3 / (1 - 2 * next.reader_.poisson_ratio);
  values.shear_pa = .5 * values.young_pa / (1 + next.reader_.poisson_ratio);
  if (!detail::Positive(values.young_pa) || !detail::Positive(values.maximum_modulus_pa) ||
      !detail::Positive(values.hourglass_modulus_pa) || !detail::Positive(values.bulk_pa) ||
      !detail::Positive(values.shear_pa)) {
    return Status::NonfiniteResult;
  }
  next.curve_ = curve;
  next.initialized_ = true;
  output = next;
  return Status::Ok;
}
}  // namespace tl::material::law90
