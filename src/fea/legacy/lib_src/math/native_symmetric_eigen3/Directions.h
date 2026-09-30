// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from complete pinned VALPVEC_V distinct-root branch:2050–2111.
#pragma once
#include "Types.h"

namespace tl::math::native_symmetric_eigen3 {
TL_NATIVE_SPECTRUM_HD inline bool DistinctDirections(Work& work,
                                         NativeSymmetricSpectrum3& result) noexcept {
  double* v = result.vectors.v;
  unsigned column = LargestColumn(work.magnitude);
  double inverse = 1 / work.magnitude[column];
  for (unsigned r = 0; r < 3; ++r) v[3*r] = work.b[r][column] * inverse;
  for (unsigned r = 0; r < 3; ++r)
    work.a[r][r] = work.a[r][r] + work.roots[0] - work.roots[2];
  for (unsigned c = 0; c < 3; ++c) {
    work.b[0][c] = work.a[1][c]*v[6] - work.a[2][c]*v[3];
    work.b[1][c] = work.a[2][c]*v[0] - work.a[0][c]*v[6];
    work.b[2][c] = work.a[0][c]*v[3] - work.a[1][c]*v[0];
  }
  if (!ColumnMagnitudes(work.b, work.magnitude)) return false;
  column = LargestColumn(work.magnitude);
  double magnitude = ::sqrt(v[0]*v[0] + v[3]*v[3]);
  if (work.magnitude[column] > work.tolerance2) {
    inverse = 1 / work.magnitude[column];
    for (unsigned r = 0; r < 3; ++r) v[3*r+2] = work.b[r][column] * inverse;
    v[1] = v[5]*v[6] - v[3]*v[8];
    v[4] = v[8]*v[0] - v[6]*v[2];
    v[7] = v[2]*v[3] - v[0]*v[5];
    magnitude = 1 / ::sqrt(v[1]*v[1] + v[4]*v[4] + v[7]*v[7]);
    v[1] = v[1] * magnitude;
    v[4] = v[4] * magnitude;
    v[7] = v[7] * magnitude;
  } else if (magnitude > work.tolerance2) {
    v[1] = -v[3] / magnitude;
    v[4] = v[0] / magnitude;
    v[7] = 0;
  } else {
    v[1] = 1;
    v[4] = 0;
    v[7] = 0;
  }
  return true;
}
} // namespace tl::math::native_symmetric_eigen3
