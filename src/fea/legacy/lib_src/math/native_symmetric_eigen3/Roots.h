// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from complete pinned VALPVEC_V:1930–2016; see the owning README.
#pragma once
#include "Types.h"

namespace tl::math::native_symmetric_eigen3 {
TL_NATIVE_SPECTRUM_HD inline bool Roots(const double (&tensor)[6], Work& work,
                            NativeSymmetricSpectrum3& result) noexcept {
  constexpr double third = 1.0 / 3.0;
  constexpr double em10 = 1.0 / 1e10;
  constexpr double em20 = 1.0 / 1e20;
  const double ttpi = ::acos(-0.5);
  const double ftpi = 2 * ttpi;
  double cs[6];
  for (unsigned k = 0; k < 6; ++k) cs[k] = tensor[k];
  const double pressure = -(cs[0] + cs[1] + cs[2]) * third;
  for (unsigned k = 0; k < 3; ++k) cs[k] = cs[k] + pressure;
  work.invariant = cs[3]*cs[3] + cs[4]*cs[4] + cs[5]*cs[5]
      - cs[0]*cs[1] - cs[1]*cs[2] - cs[0]*cs[2];
  double norm = ::fabs(cs[0]);
  for (unsigned k = 1; k < 6; ++k) norm = Maximum(norm, ::fabs(cs[k]));
  work.near_triple = em10 * norm;
  const double aa = Maximum(work.invariant, work.near_triple);
  const double bb = cs[0]*(cs[4]*cs[4]) + cs[1]*(cs[5]*cs[5])
      + cs[2]*(cs[3]*cs[3]) - cs[0]*cs[1]*cs[2] - 2*cs[3]*cs[4]*cs[5];
  double cc = -::sqrt(27 / Maximum(em20, aa)) * bb * 0.5 / Maximum(em20, aa);
  if (!Finite(pressure) || !Finite(work.invariant) || !Finite(cc)) return false;
  cc = Minimum(cc, 1);
  cc = Maximum(cc, -1);
  const double angle = ::acos(cc) * third;
  const double amplitude = 2 * ::sqrt(aa * third);
  work.roots[0] = amplitude * ::cos(angle);
  work.roots[1] = amplitude * ::cos(angle + ftpi);
  work.roots[2] = amplitude * ::cos(angle + ttpi);
  for (unsigned k = 0; k < 3; ++k) {
    result.value[k] = work.roots[k] - pressure;
    if (!Finite(result.value[k])) return false;
  }
  if (::fabs(work.roots[2]) > ::fabs(work.roots[0]) &&
      work.invariant > work.near_triple) {
    const double saved = work.roots[0];
    work.roots[0] = work.roots[2];
    work.roots[2] = saved;
    work.compression_swap = true;
  }
  const double maximum = Maximum(::fabs(work.roots[0]), ::fabs(work.roots[2]));
  const double precision = NativePrecisionFactor();
  work.tolerance1 = Maximum(em20, precision * (maximum*maximum));
  work.tolerance2 = precision * maximum / 3;
  for (unsigned k = 0; k < 3; ++k) work.a[k][k] = cs[k] - work.roots[0];
  work.a[0][1] = cs[3];
  work.a[1][0] = cs[3];
  work.a[1][2] = cs[4];
  work.a[2][1] = cs[4];
  work.a[0][2] = cs[5];
  work.a[2][0] = cs[5];
  // Native columns are A1 cross A2, A2 cross A3, A3 cross A1.
  for (unsigned c = 0; c < 3; ++c) {
    const unsigned next = (c + 1) % 3;
    work.b[0][c] = work.a[1][c]*work.a[2][next] - work.a[2][c]*work.a[1][next];
    work.b[1][c] = work.a[2][c]*work.a[0][next] - work.a[0][c]*work.a[2][next];
    work.b[2][c] = work.a[0][c]*work.a[1][next] - work.a[1][c]*work.a[0][next];
  }
  return Finite(work.tolerance1) && Finite(work.tolerance2) &&
      ColumnMagnitudes(work.b, work.magnitude);
}
} // namespace tl::math::native_symmetric_eigen3
