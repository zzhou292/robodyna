// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from complete pinned VALPVEC_V double-root branch:2115–2166.
#pragma once
#include "Types.h"

namespace tl::math::native_symmetric_eigen3 {
TL_NATIVE_SPECTRUM_HD inline bool RepeatedDirections(Work& work,
                                         NativeSymmetricSpectrum3& result) noexcept {
  double* v = result.vectors.v;
  for (unsigned c = 0; c < 3; ++c) {
    work.magnitude[c] = ::sqrt(work.a[0][c]*work.a[0][c] + work.a[1][c]*work.a[1][c]);
    if (!Finite(work.magnitude[c])) return false;
  }
  const unsigned column = LargestColumn(work.magnitude);
  const double bottom = Maximum(Maximum(::fabs(work.a[2][0]), ::fabs(work.a[2][1])),
                                ::fabs(work.a[2][2]));
  if (bottom < work.tolerance2) {
    const double inverse = 1 / work.magnitude[column];
    v[0] = 0;
    v[3] = 0;
    v[6] = 1;
    v[1] = -work.a[1][column] * inverse;
    v[4] = work.a[0][column] * inverse;
    v[7] = 0;
  } else if (work.magnitude[column] > work.tolerance2) {
    const double inverse = 1 / work.magnitude[column];
    v[0] = -work.a[1][column] * inverse;
    v[3] = work.a[0][column] * inverse;
    v[6] = 0;
    v[1] = -work.a[2][column] * v[3];
    v[4] = work.a[2][column] * v[0];
    v[7] = work.a[0][column]*v[3] - work.a[1][column]*v[0];
    const double normalize = 1 / ::sqrt(v[1]*v[1] + v[4]*v[4] + v[7]*v[7]);
    v[1] = v[1] * normalize;
    v[4] = v[4] * normalize;
    v[7] = v[7] * normalize;
  } else {
    CoordinateDirections(result);
  }
  return true;
}
} // namespace tl::math::native_symmetric_eigen3
