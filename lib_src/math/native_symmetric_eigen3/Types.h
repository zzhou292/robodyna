// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss VALPVEC_V, Copyright (C) 2026 Siemens.
#pragma once
#include "../Fixed3.h"
#include "../Quaternion.h"
#if defined(__CUDACC__)
#define TL_NATIVE_SPECTRUM_HD __host__ __device__
#else
#define TL_NATIVE_SPECTRUM_HD
#endif

namespace tl::math {
// Native principal order, including the coordinate-order near-triple fallback.
// Matrix storage is row-major; its columns are the three native directions.
struct NativeSymmetricSpectrum3 {
  double value[3]{};
  Matrix3 vectors{};
};

namespace native_symmetric_eigen3 {
enum class DirectionPath { NearTriple, Distinct, Repeated };
struct Work {
  double roots[3]{};
  double a[3][3]{};
  double b[3][3]{};
  double magnitude[3]{};
  double invariant = 0;
  double near_triple = 0;
  double tolerance1 = 0;
  double tolerance2 = 0;
  bool compression_swap = false;
};
TL_NATIVE_SPECTRUM_HD inline double Maximum(double a, double b) noexcept {
  return a > b ? a : b;
}
TL_NATIVE_SPECTRUM_HD inline double Minimum(double a, double b) noexcept {
  return a < b ? a : b;
}
TL_NATIVE_SPECTRUM_HD inline double NativePrecisionFactor() noexcept {
  // precision.c stores a default REAL, and SQRT(FLMIN) is REAL too.
  return 2 * static_cast<double>(::sqrtf(2.2e-16f));
}
TL_NATIVE_SPECTRUM_HD inline unsigned LargestColumn(const double (&magnitude)[3]) noexcept {
  const double largest = Maximum(Maximum(magnitude[0], magnitude[1]), magnitude[2]);
  if (magnitude[0] == largest) return 0;
  if (magnitude[1] == largest) return 1;
  return 2;
}
TL_NATIVE_SPECTRUM_HD inline bool ColumnMagnitudes(const double (&a)[3][3],
                                        double (&magnitude)[3]) noexcept {
  for (unsigned c = 0; c < 3; ++c) {
    magnitude[c] = ::sqrt(a[0][c]*a[0][c] + a[1][c]*a[1][c] + a[2][c]*a[2][c]);
    if (!Finite(magnitude[c])) return false;
  }
  return true;
}
TL_NATIVE_SPECTRUM_HD inline void CoordinateDirections(NativeSymmetricSpectrum3& result) noexcept {
  result.vectors.v[0] = 1;
  result.vectors.v[3] = 0;
  result.vectors.v[6] = 0;
  result.vectors.v[1] = 0;
  result.vectors.v[4] = 1;
  result.vectors.v[7] = 0;
}
TL_NATIVE_SPECTRUM_HD inline DirectionPath Path(const Work& work) noexcept {
  if (work.invariant < work.near_triple) return DirectionPath::NearTriple;
  if (work.magnitude[LargestColumn(work.magnitude)] > work.tolerance1)
    return DirectionPath::Distinct;
  return DirectionPath::Repeated;
}
} // namespace native_symmetric_eigen3
} // namespace tl::math
